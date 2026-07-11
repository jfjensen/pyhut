"""
Concrete drivers for the Duckiebot HUT v3.1 on a Jetson Nano.

Implements the interfaces in hut.interfaces using only the bare-metal
primitives in hut.lowlevel (raw I2C + sysfs GPIO, std lib only).

Confirmed hardware facts this is built on (verified on real hardware):
    I2C bus            /dev/i2c-1
    Motors  PCA9685    0x60   LEFT in1=ch10 in2=ch9 pwm=ch8 (all on PCA)
                              RIGHT in1=gpio38 in2=gpio200 pwm=ch13 (hybrid)
    IMU    MPU9250     0x68   accel/gyro/temp
    Mag    AK8963      0x0c   behind the MPU, visible only in bypass mode
    ToF    VL53L0X     0x29   (present only when wired / XSHUT released)
    OLED   SSD1306     0x3c   128x64
    Encoders                  Hall, single channel, 135 ticks/rev
                              left gpio79, right gpio76

TB6612 stop state: both direction lines LOW is de-energised coast (the safe
rest state). One line high with PWM low is a short brake that heats the chip,
so stop() always pulls both lines low.
"""


import threading
import time
from typing import List, Optional, Sequence

from . import font
from .interfaces import (
    Display,
    Drivetrain,
    IMU,
    IMUReading,
    Magnetometer,
    MagReading,
    RangeSensor,
    Robot,
    Sampler,
    WheelEncoder,
)
from .lowlevel import EdgeCounter, Gpio, I2CDevice
from .sampler import ThreadedSampler
from .calibration import Calibration, CalibratedIMU, CalibratedMagnetometer

# Default board addresses / bus -------------------------------------------------
I2C_BUS = "/dev/i2c-1"
ADDR_MOTORS = 0x60
ADDR_IMU = 0x68
ADDR_MAG = 0x0C
ADDR_TOF = 0x29
ADDR_OLED = 0x3C

# Right-motor direction GPIOs and encoder GPIOs, as sysfs numbers on this unit.
GPIO_RIGHT_IN1 = 38     # board pin 33
GPIO_RIGHT_IN2 = 200    # board pin 31
GPIO_ENC_LEFT = 79      # board pin 12
GPIO_ENC_RIGHT = 76     # board pin 35

ENCODER_TICKS_PER_REV = 135


# ===========================================================================
# PCA9685 16-channel PWM (motor board at 0x60)
# ===========================================================================
class PCA9685:
    MODE1 = 0x00
    LED0_ON_L = 0x06

    def __init__(self, bus: str = I2C_BUS, address: int = ADDR_MOTORS):
        self._dev = I2CDevice(bus, address)
        # Wake: clear sleep, then enable register auto-increment. The default
        # ~200 Hz output is fine for the motors, so frequency is left as-is.
        self._dev.write_reg(self.MODE1, 0x00)
        self._dev.write_reg(self.MODE1, 0x20)
        time.sleep(0.005)

    def _channel_reg(self, channel: int) -> int:
        return self.LED0_ON_L + 4 * channel

    def set_pwm(self, channel: int, on: int, off: int) -> None:
        self._dev.write_reg(
            self._channel_reg(channel),
            bytes([on & 0xFF, (on >> 8) & 0x0F, off & 0xFF, (off >> 8) & 0x0F]),
        )

    def full_on(self, channel: int) -> None:
        # ON_H bit 4 set => output constantly high.
        self._dev.write_reg(self._channel_reg(channel), bytes([0x00, 0x10, 0x00, 0x00]))

    def full_off(self, channel: int) -> None:
        # OFF_H bit 4 set => output constantly low.
        self._dev.write_reg(self._channel_reg(channel), bytes([0x00, 0x00, 0x00, 0x10]))

    def set_duty(self, channel: int, duty: int) -> None:
        """duty in 0..4095."""
        duty = max(0, min(4095, int(duty)))
        if duty == 0:
            self.full_off(channel)
        elif duty >= 4095:
            self.full_on(channel)
        else:
            self.set_pwm(channel, 0, duty)

    def close(self) -> None:
        self._dev.close()


# ===========================================================================
# Drivetrain: background control thread + watchdog (sole hardware owner)
# ===========================================================================
class DuckiebotDrivetrain(Drivetrain):
    """Two-wheel drive with the watchdog/threading design.

    The control thread is the ONLY thing that touches motor hardware; it
    re-asserts the latest target at `rate_hz` and zeros the wheels if no fresh
    target has arrived within `watchdog_timeout`. set_target() only updates two
    locked variables, so a stalled perception loop stops the robot on its own.
    """

    MAX_DUTY = 4095

    def __init__(
        self,
        pca: Optional[PCA9685] = None,
        bus: str = I2C_BUS,
        rate_hz: float = 30.0,
        watchdog_timeout: float = 0.3,
    ):
        self._pca = pca or PCA9685(bus, ADDR_MOTORS)
        self._owns_pca = pca is None
        self._r_in1 = Gpio(GPIO_RIGHT_IN1, "out", False)
        self._r_in2 = Gpio(GPIO_RIGHT_IN2, "out", False)

        self._lock = threading.Lock()
        self._left = 0.0
        self._right = 0.0
        self._stamp = time.monotonic()
        self._rate = rate_hz
        self._timeout = watchdog_timeout
        self._stop_evt = threading.Event()
        self._thread = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()

    def set_target(self, left: float, right: float) -> None:
        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))
        with self._lock:
            self._left = left
            self._right = right
            self._stamp = time.monotonic()

    def stop(self) -> None:
        self.set_target(0.0, 0.0)

    def close(self) -> None:
        self._stop_evt.set()
        self._thread.join(timeout=1.0)
        self._apply_left(0.0)   # final stop from the main thread
        self._apply_right(0.0)
        self._r_in1.close()
        self._r_in2.close()
        if self._owns_pca:
            self._pca.close()

    def _control_loop(self) -> None:
        period = 1.0 / self._rate
        while not self._stop_evt.is_set():
            with self._lock:
                left, right, stamp = self._left, self._right, self._stamp
            if (time.monotonic() - stamp) > self._timeout:
                left = right = 0.0
            try:
                self._apply_left(left)
                self._apply_right(right)
            except OSError:
                pass  # transient bus hiccup: skip a tick, keep watchdog alive
            time.sleep(period)

    def _apply_left(self, speed: float) -> None:
        # LEFT: in1=ch10, in2=ch9, pwm=ch8 (all PCA channels)
        if speed > 0:
            self._pca.full_on(10)
            self._pca.full_off(9)
        elif speed < 0:
            self._pca.full_off(10)
            self._pca.full_on(9)
        else:
            self._pca.full_off(10)   # both low = de-energised stop
            self._pca.full_off(9)
        self._pca.set_duty(8, abs(speed) * self.MAX_DUTY)

    def _apply_right(self, speed: float) -> None:
        # RIGHT: in1=gpio, in2=gpio, pwm=ch13
        if speed > 0:
            self._r_in1.set_high()
            self._r_in2.set_low()
        elif speed < 0:
            self._r_in1.set_low()
            self._r_in2.set_high()
        else:
            self._r_in1.set_low()    # both low = de-energised stop
            self._r_in2.set_low()
        self._pca.set_duty(13, abs(speed) * self.MAX_DUTY)


# ===========================================================================
# MPU9250 IMU (+ AK8963 magnetometer behind it)
# ===========================================================================
class MPU9250(IMU):
    """6-axis IMU. Also exposes the AK8963 magnetometer via read_mag().

    The mag has its own address (0x0c) but only appears on the main bus once
    the MPU is put in bypass mode. Use `as_magnetometer()` to get an object
    that satisfies the Magnetometer interface.
    """

    SMPLRT_DIV = 0x19
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C
    INT_PIN_CFG = 0x37
    ACCEL_XOUT_H = 0x3B
    PWR_MGMT_1 = 0x6B
    WHO_AM_I = 0x75

    AK_WIA = 0x00
    AK_ST1 = 0x02
    AK_HXL = 0x03
    AK_CNTL1 = 0x0A
    AK_ASAX = 0x10

    ACCEL_SCALE = 16384.0  # LSB/g at +/-2g
    GYRO_SCALE = 131.0     # LSB/(deg/s) at +/-250 dps

    def __init__(self, bus: str = I2C_BUS, address: int = ADDR_IMU):
        self._dev = I2CDevice(bus, address)
        self.who_am_i = self._dev.read_u8(self.WHO_AM_I)
        self._dev.write_u8(self.PWR_MGMT_1, 0x01)   # wake, auto clock
        time.sleep(0.01)
        self._dev.write_u8(self.GYRO_CONFIG, 0x00)  # +/-250 dps
        self._dev.write_u8(self.ACCEL_CONFIG, 0x00) # +/-2 g
        self._dev.write_u8(self.INT_PIN_CFG, 0x02)  # bypass: expose AK8963 at 0x0c
        time.sleep(0.05)  # let first temp conversion settle; first sample is junk

        self._mag: Optional[I2CDevice] = None
        self._mag_adj = (1.0, 1.0, 1.0)
        self._init_magnetometer(bus)

    def _init_magnetometer(self, bus: str) -> None:
        try:
            mag = I2CDevice(bus, ADDR_MAG)
            if mag.read_u8(self.AK_WIA) != 0x48:
                mag.close()
                return
            mag.write_u8(self.AK_CNTL1, 0x00)   # power down
            time.sleep(0.01)
            mag.write_u8(self.AK_CNTL1, 0x0F)   # fuse ROM access
            time.sleep(0.01)
            asa = mag.read_reg(self.AK_ASAX, 3)
            self._mag_adj = tuple(((a - 128) * 0.5 / 128.0) + 1.0 for a in asa)
            mag.write_u8(self.AK_CNTL1, 0x00)
            time.sleep(0.01)
            mag.write_u8(self.AK_CNTL1, 0x16)   # 16-bit, continuous mode 2 (100 Hz)
            time.sleep(0.01)
            self._mag = mag
        except OSError:
            self._mag = None  # degrade gracefully; accel/gyro still work

    @staticmethod
    def _s16(hi: int, lo: int) -> int:
        v = (hi << 8) | lo
        return v - 65536 if v & 0x8000 else v

    def read(self) -> IMUReading:
        raw = self._dev.read_reg(self.ACCEL_XOUT_H, 14)
        ax = self._s16(raw[0], raw[1]) / self.ACCEL_SCALE
        ay = self._s16(raw[2], raw[3]) / self.ACCEL_SCALE
        az = self._s16(raw[4], raw[5]) / self.ACCEL_SCALE
        traw = self._s16(raw[6], raw[7])
        gx = self._s16(raw[8], raw[9]) / self.GYRO_SCALE
        gy = self._s16(raw[10], raw[11]) / self.GYRO_SCALE
        gz = self._s16(raw[12], raw[13]) / self.GYRO_SCALE
        temp_c = traw / 333.87 + 21.0 if self.who_am_i == 0x71 else traw / 340.0 + 36.53
        return IMUReading((ax, ay, az), (gx, gy, gz), temp_c)

    @property
    def has_magnetometer(self) -> bool:
        return self._mag is not None

    def read_mag(self) -> MagReading:
        if self._mag is None:
            raise RuntimeError("magnetometer not available")
        # 6 data bytes (little-endian) + ST2; reading ST2 completes the sample.
        raw = self._mag.read_reg(self.AK_HXL, 7)
        mx = self._s16(raw[1], raw[0]) * 0.15 * self._mag_adj[0]
        my = self._s16(raw[3], raw[2]) * 0.15 * self._mag_adj[1]
        mz = self._s16(raw[5], raw[4]) * 0.15 * self._mag_adj[2]
        return MagReading((mx, my, mz))

    def as_magnetometer(self) -> Optional["Magnetometer"]:
        return _MagView(self) if self.has_magnetometer else None

    def close(self) -> None:
        if self._mag is not None:
            self._mag.close()
        self._dev.close()


class _MagView(Magnetometer):
    """Adapts an MPU9250 to the standalone Magnetometer interface."""

    def __init__(self, mpu: MPU9250):
        self._mpu = mpu

    def read(self) -> MagReading:
        return self._mpu.read_mag()


# ===========================================================================
# Wheel encoder (single Hall channel)
# ===========================================================================
class HallWheelEncoder(WheelEncoder):
    ticks_per_rev = ENCODER_TICKS_PER_REV

    def __init__(self, gpio_number: int):
        self._counter = EdgeCounter(gpio_number)

    @property
    def ticks(self) -> int:
        return self._counter.count

    def reset(self) -> None:
        self._counter.reset()

    def close(self) -> None:
        self._counter.close()


# ===========================================================================
# VL53L0X time-of-flight range sensor
# ===========================================================================
# The VL53L0X class below and its module-level helpers are a port of Adafruit's
# CircuitPython VL53L0X driver, used under the MIT License. The ST tuning
# sequence is reproduced verbatim. See the NOTICES file for the full license.
#
# SPDX-FileCopyrightText: 2017 Tony DiCola for Adafruit Industries
# SPDX-License-Identifier: MIT
# Adapted from https://github.com/adafruit/Adafruit_CircuitPython_VL53L0X
import math as _math

_SYSRANGE_START = 0x00
_SYSTEM_SEQUENCE_CONFIG = 0x01
_SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A
_SYSTEM_INTERRUPT_CLEAR = 0x0B
_GPIO_HV_MUX_ACTIVE_HIGH = 0x84
_RESULT_INTERRUPT_STATUS = 0x13
_RESULT_RANGE_STATUS = 0x14
_MSRC_CONFIG_CONTROL = 0x60
_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44
_PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50
_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51
_FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70
_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71
_MSRC_CONFIG_TIMEOUT_MACROP = 0x46
_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0
_GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6
_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E
_DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F
_VCSEL_PERIOD_PRE_RANGE = 0
_VCSEL_PERIOD_FINAL_RANGE = 1


def _decode_timeout(val: int) -> float:
    return float(val & 0xFF) * _math.pow(2.0, ((val & 0xFF00) >> 8)) + 1


def _encode_timeout(timeout_mclks: float) -> int:
    timeout_mclks = int(timeout_mclks) & 0xFFFF
    if timeout_mclks > 0:
        ls_byte = timeout_mclks - 1
        ms_byte = 0
        while ls_byte > 255:
            ls_byte >>= 1
            ms_byte += 1
        return ((ms_byte << 8) | (ls_byte & 0xFF)) & 0xFFFF
    return 0


def _timeout_mclks_to_us(timeout_period_mclks: int, vcsel_period_pclks: int) -> int:
    macro_period_ns = ((2304 * vcsel_period_pclks * 1655) + 500) // 1000
    return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns // 2)) // 1000


def _timeout_us_to_mclks(timeout_period_us: int, vcsel_period_pclks: int) -> int:
    macro_period_ns = ((2304 * vcsel_period_pclks * 1655) + 500) // 1000
    return ((timeout_period_us * 1000) + (macro_period_ns // 2)) // macro_period_ns


class VL53L0X(RangeSensor):
    """Time-of-flight distance sensor.

    read_mm() returns a single measurement (starts, polls, and stops a one-shot
    ranging sequence on every call). For fast sweeps, use continuous mode:
    start_continuous() once, then read_continuous() repeatedly, then
    stop_continuous(). read_continuous() checks RESULT_RANGE_STATUS and returns
    None on an invalid / out-of-range measurement instead of a garbage value.
    """

    def __init__(self, bus: str = I2C_BUS, address: int = ADDR_TOF, io_timeout_s: float = 0.1):
        self._dev = I2CDevice(bus, address)
        self.io_timeout_s = io_timeout_s
        self._data_ready = False
        self._continuous_mode = False
        if (self._r8(0xC0) != 0xEE or self._r8(0xC1) != 0xAA or self._r8(0xC2) != 0x10):
            raise RuntimeError("VL53L0X ID check failed. Check wiring / XSHUT.")
        for pair in ((0x88, 0x00), (0x80, 0x01), (0xFF, 0x01), (0x00, 0x00)):
            self._w8(pair[0], pair[1])
        self._stop_variable = self._r8(0x91)
        for pair in ((0x00, 0x01), (0xFF, 0x00), (0x80, 0x00)):
            self._w8(pair[0], pair[1])
        self._w8(_MSRC_CONFIG_CONTROL, self._r8(_MSRC_CONFIG_CONTROL) | 0x12)
        self.signal_rate_limit = 0.25
        self._w8(_SYSTEM_SEQUENCE_CONFIG, 0xFF)
        spad_count, spad_is_aperture = self._get_spad_info()
        ref_spad_map = bytearray(7)
        ref_spad_map[0] = _GLOBAL_CONFIG_SPAD_ENABLES_REF_0
        ref_spad_map[1:7] = self._dev.read_reg(_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 6)
        for pair in (
            (0xFF, 0x01), (_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00),
            (_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C), (0xFF, 0x00),
            (_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4),
        ):
            self._w8(pair[0], pair[1])
        first_spad = 12 if spad_is_aperture else 0
        spads_enabled = 0
        for i in range(48):
            if i < first_spad or spads_enabled == spad_count:
                ref_spad_map[1 + (i // 8)] &= ~(1 << (i % 8))
            elif (ref_spad_map[1 + (i // 8)] >> (i % 8)) & 0x1 > 0:
                spads_enabled += 1
        self._dev.write(bytes(ref_spad_map))
        for pair in (
            (0xFF, 0x01), (0x00, 0x00), (0xFF, 0x00), (0x09, 0x00), (0x10, 0x00),
            (0x11, 0x00), (0x24, 0x01), (0x25, 0xFF), (0x75, 0x00), (0xFF, 0x01),
            (0x4E, 0x2C), (0x48, 0x00), (0x30, 0x20), (0xFF, 0x00), (0x30, 0x09),
            (0x54, 0x00), (0x31, 0x04), (0x32, 0x03), (0x40, 0x83), (0x46, 0x25),
            (0x60, 0x00), (0x27, 0x00), (0x50, 0x06), (0x51, 0x00), (0x52, 0x96),
            (0x56, 0x08), (0x57, 0x30), (0x61, 0x00), (0x62, 0x00), (0x64, 0x00),
            (0x65, 0x00), (0x66, 0xA0), (0xFF, 0x01), (0x22, 0x32), (0x47, 0x14),
            (0x49, 0xFF), (0x4A, 0x00), (0xFF, 0x00), (0x7A, 0x0A), (0x7B, 0x00),
            (0x78, 0x21), (0xFF, 0x01), (0x23, 0x34), (0x42, 0x00), (0x44, 0xFF),
            (0x45, 0x26), (0x46, 0x05), (0x40, 0x40), (0x0E, 0x06), (0x20, 0x1A),
            (0x43, 0x40), (0xFF, 0x00), (0x34, 0x03), (0x35, 0x44), (0xFF, 0x01),
            (0x31, 0x04), (0x4B, 0x09), (0x4C, 0x05), (0x4D, 0x04), (0xFF, 0x00),
            (0x44, 0x00), (0x45, 0x20), (0x47, 0x08), (0x48, 0x28), (0x67, 0x00),
            (0x70, 0x04), (0x71, 0x01), (0x72, 0xFE), (0x76, 0x00), (0x77, 0x00),
            (0xFF, 0x01), (0x0D, 0x01), (0xFF, 0x00), (0x80, 0x01), (0x01, 0xF8),
            (0xFF, 0x01), (0x8E, 0x01), (0x00, 0x01), (0xFF, 0x00), (0x80, 0x00),
        ):
            self._w8(pair[0], pair[1])
        self._w8(_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04)
        self._w8(_GPIO_HV_MUX_ACTIVE_HIGH, self._r8(_GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10)
        self._w8(_SYSTEM_INTERRUPT_CLEAR, 0x01)
        self._measurement_timing_budget_us = self.measurement_timing_budget
        self._w8(_SYSTEM_SEQUENCE_CONFIG, 0xE8)
        self.measurement_timing_budget = self._measurement_timing_budget_us
        self._w8(_SYSTEM_SEQUENCE_CONFIG, 0x01)
        self._perform_single_ref_calibration(0x40)
        self._w8(_SYSTEM_SEQUENCE_CONFIG, 0x02)
        self._perform_single_ref_calibration(0x00)
        self._w8(_SYSTEM_SEQUENCE_CONFIG, 0xE8)

    # raw register I/O
    def _r8(self, reg: int) -> int:
        return self._dev.read_reg(reg, 1)[0]

    def _r16(self, reg: int) -> int:
        b = self._dev.read_reg(reg, 2)
        return (b[0] << 8) | b[1]

    def _w8(self, reg: int, val: int) -> None:
        self._dev.write(bytes([reg & 0xFF, val & 0xFF]))

    def _w16(self, reg: int, val: int) -> None:
        self._dev.write(bytes([reg & 0xFF, (val >> 8) & 0xFF, val & 0xFF]))

    def _get_spad_info(self):
        for pair in ((0x80, 0x01), (0xFF, 0x01), (0x00, 0x00), (0xFF, 0x06)):
            self._w8(pair[0], pair[1])
        self._w8(0x83, self._r8(0x83) | 0x04)
        for pair in ((0xFF, 0x07), (0x81, 0x01), (0x80, 0x01), (0x94, 0x6B), (0x83, 0x00)):
            self._w8(pair[0], pair[1])
        start = time.monotonic()
        while self._r8(0x83) == 0x00:
            if self.io_timeout_s > 0 and (time.monotonic() - start) >= self.io_timeout_s:
                raise RuntimeError("Timeout waiting for VL53L0X!")
        self._w8(0x83, 0x01)
        tmp = self._r8(0x92)
        count = tmp & 0x7F
        is_aperture = ((tmp >> 7) & 0x01) == 1
        for pair in ((0x81, 0x00), (0xFF, 0x06)):
            self._w8(pair[0], pair[1])
        self._w8(0x83, self._r8(0x83) & ~0x04)
        for pair in ((0xFF, 0x01), (0x00, 0x01), (0xFF, 0x00), (0x80, 0x00)):
            self._w8(pair[0], pair[1])
        return (count, is_aperture)

    def _perform_single_ref_calibration(self, vhv_init_byte: int) -> None:
        self._w8(_SYSRANGE_START, 0x01 | vhv_init_byte & 0xFF)
        start = time.monotonic()
        while (self._r8(_RESULT_INTERRUPT_STATUS) & 0x07) == 0:
            if self.io_timeout_s > 0 and (time.monotonic() - start) >= self.io_timeout_s:
                raise RuntimeError("Timeout waiting for VL53L0X!")
        self._w8(_SYSTEM_INTERRUPT_CLEAR, 0x01)
        self._w8(_SYSRANGE_START, 0x00)

    def _get_vcsel_pulse_period(self, t: int) -> int:
        if t == _VCSEL_PERIOD_PRE_RANGE:
            return ((self._r8(_PRE_RANGE_CONFIG_VCSEL_PERIOD) + 1) & 0xFF) << 1
        if t == _VCSEL_PERIOD_FINAL_RANGE:
            return ((self._r8(_FINAL_RANGE_CONFIG_VCSEL_PERIOD) + 1) & 0xFF) << 1
        return 255

    def _get_sequence_step_enables(self):
        cfg = self._r8(_SYSTEM_SEQUENCE_CONFIG)
        return (
            (cfg >> 4) & 0x1 > 0, (cfg >> 3) & 0x1 > 0, (cfg >> 2) & 0x1 > 0,
            (cfg >> 6) & 0x1 > 0, (cfg >> 7) & 0x1 > 0,
        )

    def _get_sequence_step_timeouts(self, pre_range: bool):
        pre_pclks = self._get_vcsel_pulse_period(_VCSEL_PERIOD_PRE_RANGE)
        msrc_mclks = (self._r8(_MSRC_CONFIG_TIMEOUT_MACROP) + 1) & 0xFF
        msrc_us = _timeout_mclks_to_us(msrc_mclks, pre_pclks)
        pre_mclks = _decode_timeout(self._r16(_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI))
        pre_us = _timeout_mclks_to_us(pre_mclks, pre_pclks)
        final_pclks = self._get_vcsel_pulse_period(_VCSEL_PERIOD_FINAL_RANGE)
        final_mclks = _decode_timeout(self._r16(_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI))
        if pre_range:
            final_mclks -= pre_mclks
        final_us = _timeout_mclks_to_us(final_mclks, final_pclks)
        return (msrc_us, pre_us, final_us, final_pclks, pre_mclks)

    @property
    def signal_rate_limit(self) -> float:
        return self._r16(_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7)

    @signal_rate_limit.setter
    def signal_rate_limit(self, val: float) -> None:
        assert 0.0 <= val <= 511.99
        self._w16(_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, int(val * (1 << 7)))

    @property
    def measurement_timing_budget(self) -> int:
        budget = 1910 + 960
        tcc, dss, msrc, pre_range, final_range = self._get_sequence_step_enables()
        msrc_us, pre_us, final_us, _, _ = self._get_sequence_step_timeouts(pre_range)
        if tcc:
            budget += msrc_us + 590
        if dss:
            budget += 2 * (msrc_us + 690)
        elif msrc:
            budget += msrc_us + 660
        if pre_range:
            budget += pre_us + 660
        if final_range:
            budget += final_us + 550
        self._measurement_timing_budget_us = budget
        return budget

    @measurement_timing_budget.setter
    def measurement_timing_budget(self, budget_us: int) -> None:
        assert budget_us >= 20000
        used = 1320 + 960
        tcc, dss, msrc, pre_range, final_range = self._get_sequence_step_enables()
        steps = self._get_sequence_step_timeouts(pre_range)
        msrc_us, pre_us = steps[0], steps[1]
        final_pclks, pre_mclks = steps[3], steps[4]
        if tcc:
            used += msrc_us + 590
        if dss:
            used += 2 * (msrc_us + 690)
        elif msrc:
            used += msrc_us + 660
        if pre_range:
            used += pre_us + 660
        if final_range:
            used += 550
            if used > budget_us:
                raise ValueError("Requested timeout too big.")
            final_timeout_us = budget_us - used
            final_timeout_mclks = _timeout_us_to_mclks(final_timeout_us, final_pclks)
            if pre_range:
                final_timeout_mclks += pre_mclks
            self._w16(_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, _encode_timeout(final_timeout_mclks))
            self._measurement_timing_budget_us = budget_us

    @property
    def _ready(self) -> bool:
        if not self._data_ready:
            self._data_ready = self._r8(_RESULT_INTERRUPT_STATUS) & 0x07 != 0
        return self._data_ready

    # measurement timing budget (speed / accuracy tradeoff) ----------------
    # The measurement_timing_budget property (getter + setter) above reads and
    # writes the sensor's per-measurement time in microseconds. Longer budgets
    # give more accurate readings; the minimum is 20000 us (20 ms). Set this
    # before starting continuous ranging, e.g.:
    #     tof.measurement_timing_budget = 20000   # fast sweep
    #     tof.measurement_timing_budget = 200000  # slow, accurate

    # range-status decoding -------------------------------------------------
    # RESULT_RANGE_STATUS (0x14) holds the device range status in bits [6:3].
    # A decoded status of 11 ("valid") is the only good measurement; anything
    # else (sigma / signal fail, min/max range fail, phase failures, etc.) means
    # the range value is not trustworthy. See VL53L0X_GetRangeStatus in ST's API.
    def _range_status(self) -> int:
        return (self._r8(_RESULT_RANGE_STATUS) & 0x78) >> 3

    def _do_range_measurement(self) -> None:
        # Adapted from do_range_measurement / readRangeSingleMillimeters in the
        # Adafruit CircuitPython VL53L0X driver (pololu code).
        for pair in (
            (0x80, 0x01), (0xFF, 0x01), (0x00, 0x00), (0x91, self._stop_variable),
            (0x00, 0x01), (0xFF, 0x00), (0x80, 0x00), (_SYSRANGE_START, 0x01),
        ):
            self._w8(pair[0], pair[1])
        start = time.monotonic()
        while (self._r8(_SYSRANGE_START) & 0x01) > 0:
            if self.io_timeout_s > 0 and (time.monotonic() - start) >= self.io_timeout_s:
                raise RuntimeError("Timeout waiting for VL53L0X!")

    def _read_range(self):
        # Adapted from read_range / readRangeContinuousMillimeters in the
        # Adafruit CircuitPython VL53L0X driver (pololu code). Waits for a
        # reading, checks the range status, then clears the interrupt. Returns
        # the distance in mm, or None if the measurement was invalid.
        start = time.monotonic()
        while not self._ready:
            if self.io_timeout_s > 0 and (time.monotonic() - start) >= self.io_timeout_s:
                raise RuntimeError("Timeout waiting for VL53L0X!")
        status = self._range_status()
        range_mm = self._r16(_RESULT_RANGE_STATUS + 10)
        self._w8(_SYSTEM_INTERRUPT_CLEAR, 0x01)
        self._data_ready = False
        if status != 11:
            return None
        return range_mm

    @property
    def is_continuous_mode(self) -> bool:
        """True while continuous ranging is active."""
        return self._continuous_mode

    def read_mm(self) -> int:
        """Single-shot distance in millimetres.

        Runs a full one-shot start/poll sequence. Returns the raw range value;
        on an invalid / out-of-range measurement it returns 0 rather than a
        garbage distance. Use read_continuous() to distinguish "invalid" (None)
        from a genuine reading during a fast sweep.
        """
        self._do_range_measurement()
        mm = self._read_range()
        return 0 if mm is None else mm

    def start_continuous(self) -> None:
        """Start continuous back-to-back ranging.

        After this returns, call read_continuous() to fetch each reading as it
        becomes ready (no per-sample start/poll/stop overhead), then
        stop_continuous() when done.
        """
        # Adapted from start_continuous / startContinuous in the Adafruit
        # CircuitPython VL53L0X driver (pololu code).
        for pair in (
            (0x80, 0x01), (0xFF, 0x01), (0x00, 0x00), (0x91, self._stop_variable),
            (0x00, 0x01), (0xFF, 0x00), (0x80, 0x00), (_SYSRANGE_START, 0x02),
        ):
            self._w8(pair[0], pair[1])
        start = time.monotonic()
        while (self._r8(_SYSRANGE_START) & 0x01) > 0:
            if self.io_timeout_s > 0 and (time.monotonic() - start) >= self.io_timeout_s:
                raise RuntimeError("Timeout waiting for VL53L0X!")
        self._continuous_mode = True

    def read_continuous(self):
        """Return the next continuous reading in mm, or None if invalid.

        Blocks (up to io_timeout_s) until the sensor's next measurement is
        ready. Checks RESULT_RANGE_STATUS: returns the distance in millimetres
        on a valid measurement, or None on an invalid / out-of-range one.
        start_continuous() must have been called first.
        """
        if not self._continuous_mode:
            raise RuntimeError("read_continuous() requires start_continuous() first.")
        return self._read_range()

    def stop_continuous(self) -> None:
        """Stop continuous ranging and restore single-shot mode."""
        # Adapted from stop_continuous / stopContinuous in the Adafruit
        # CircuitPython VL53L0X driver (pololu code).
        for pair in (
            (_SYSRANGE_START, 0x01), (0xFF, 0x01), (0x00, 0x00),
            (0x91, 0x00), (0x00, 0x01), (0xFF, 0x00),
        ):
            self._w8(pair[0], pair[1])
        self._continuous_mode = False
        # Restore the sensor to single-ranging mode, mirroring upstream.
        self._do_range_measurement()

    def close(self) -> None:
        if self._continuous_mode:
            try:
                self.stop_continuous()
            except Exception:
                pass
        self._dev.close()


# ===========================================================================
# SSD1306 128x64 OLED
# ===========================================================================
class SSD1306(Display):
    WIDTH = 128
    HEIGHT = 64
    _CMD = 0x00
    _DATA = 0x40

    def __init__(self, bus: str = I2C_BUS, address: int = ADDR_OLED):
        self._dev = I2CDevice(bus, address)
        self._pages = self.HEIGHT // 8
        self._buf = bytearray(self.WIDTH * self._pages)
        for c in (
            0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0x8D, 0x14,
            0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12, 0x81, 0xCF, 0xD9, 0xF1,
            0xDB, 0x40, 0xA4, 0xA6, 0xAF,
        ):
            self._cmd(c)
        self.clear()

    def _cmd(self, c: int) -> None:
        self._dev.write(bytes([self._CMD, c & 0xFF]))

    def _set_pixel(self, x: int, y: int, on: bool = True) -> None:
        if 0 <= x < self.WIDTH and 0 <= y < self.HEIGHT:
            idx = x + (y // 8) * self.WIDTH
            bit = 1 << (y % 8)
            if on:
                self._buf[idx] |= bit
            else:
                self._buf[idx] &= ~bit

    def _draw_char(self, x: int, y: int, ch: str) -> None:
        cols = font.glyph(ch)
        for cx, colbyte in enumerate(cols):
            for ry in range(font.GLYPH_HEIGHT):
                if colbyte & (1 << ry):
                    self._set_pixel(x + cx, y + ry, True)

    def text(self, lines: Sequence[str]) -> None:
        self._buf = bytearray(self.WIDTH * self._pages)
        line_h = font.GLYPH_HEIGHT + 1   # 8 px row pitch
        char_w = font.GLYPH_WIDTH + 1    # 6 px advance
        for row, line in enumerate(lines):
            y = row * line_h
            if y + font.GLYPH_HEIGHT > self.HEIGHT:
                break
            for col, ch in enumerate(line):
                x = col * char_w
                if x + font.GLYPH_WIDTH > self.WIDTH:
                    break
                self._draw_char(x, y, ch)
        self.show()

    def show(self) -> None:
        self._cmd(0x21); self._cmd(0); self._cmd(self.WIDTH - 1)   # column range
        self._cmd(0x22); self._cmd(0); self._cmd(self._pages - 1)  # page range
        # Stream the framebuffer in chunks (kernel I2C has a transfer-size cap).
        chunk = 16
        for i in range(0, len(self._buf), chunk):
            self._dev.write(bytes([self._DATA]) + bytes(self._buf[i:i + chunk]))

    def clear(self) -> None:
        self._buf = bytearray(self.WIDTH * self._pages)
        self.show()

    def close(self) -> None:
        self._dev.close()


# ===========================================================================
# The robot facade
# ===========================================================================
class DuckiebotHUT(Robot):
    """Assembles the Duckiebot HUT v3.1 devices behind the Robot interface.

    Sensors are probed lazily and tolerantly: if a unit isn't on the bus (a
    common case for the ToF), its accessor returns None instead of raising, so
    the rest of the robot still comes up.
    """

    def __init__(self, bus: str = I2C_BUS, enable: Sequence[str] = ("all",),
                 calibration=None):
        self._bus = bus
        want = set(enable)
        all_ = "all" in want

        # Resolve the optional calibration. None (default) means "raw, as before"
        # -- a fresh/uncalibrated robot behaves exactly as it did. A Calibration
        # is used as-is; a str/path is loaded tolerantly (a missing file loads as
        # identity, so startup never fails on a fresh robot).
        self._calibration = self._resolve_calibration(calibration)

        self._pca = PCA9685(bus, ADDR_MOTORS)
        self._drivetrain = DuckiebotDrivetrain(pca=self._pca, bus=bus)

        self._imu: Optional[MPU9250] = None
        self._mag: Optional[Magnetometer] = None
        if all_ or "imu" in want:
            self._imu = self._try(lambda: MPU9250(bus, ADDR_IMU))
            if self._imu is not None:
                self._mag = self._imu.as_magnetometer()

        # Calibrated views. With no calibration these ARE the raw devices, so the
        # sampler below and the imu/magnetometer accessors are unchanged. With a
        # calibration, the raw drivers stay untouched -- correction is applied by
        # the wrappers, which still satisfy the IMU/Magnetometer interfaces, so
        # everything downstream (the sampler included) consumes calibrated data.
        self._imu_view = self._imu  # type: Optional[IMU]
        self._mag_view = self._mag  # type: Optional[Magnetometer]
        if self._calibration is not None:
            if self._imu is not None:
                self._imu_view = CalibratedIMU(self._imu, self._calibration)
            if self._mag is not None:
                self._mag_view = CalibratedMagnetometer(self._mag, self._calibration)

        self._left_enc = self._try(lambda: HallWheelEncoder(GPIO_ENC_LEFT)) \
            if (all_ or "encoders" in want) else None
        self._right_enc = self._try(lambda: HallWheelEncoder(GPIO_ENC_RIGHT)) \
            if (all_ or "encoders" in want) else None

        self._tof = self._try(lambda: VL53L0X(bus, ADDR_TOF)) \
            if (all_ or "tof" in want) else None

        self._display = self._try(lambda: SSD1306(bus, ADDR_OLED)) \
            if (all_ or "display" in want) else None

        # One sampler owns the read side of every sensor that came up. It is left
        # STOPPED: bringing the robot up, calibrating, or driving shouldn't pay
        # for sampling until a consumer calls bot.sampler.start(). None if there's
        # nothing to sample. Built over the views so its Sample stream carries
        # calibrated data whenever a calibration was supplied.
        sensors = (self._imu_view, self._mag_view, self._left_enc,
                   self._right_enc, self._tof)
        if any(s is not None for s in sensors):
            self._sampler = ThreadedSampler(
                imu=self._imu_view,
                magnetometer=self._mag_view,
                left_encoder=self._left_enc,
                right_encoder=self._right_enc,
                range_sensor=self._tof,
            )  # type: Optional[Sampler]
        else:
            self._sampler = None

    @staticmethod
    def _resolve_calibration(calibration):
        """Accept None / a Calibration / a path string and return a Calibration
        or None. A path is loaded tolerantly so a missing file degrades to an
        identity model rather than failing startup."""
        if calibration is None:
            return None
        if isinstance(calibration, Calibration):
            return calibration
        if isinstance(calibration, str):
            return Calibration.load_or_default(calibration)
        raise TypeError(
            "calibration must be None, a Calibration, or a path string")

    @staticmethod
    def _try(factory):
        try:
            return factory()
        except (OSError, RuntimeError):
            return None

    @property
    def drivetrain(self) -> Drivetrain:
        return self._drivetrain

    @property
    def imu(self) -> Optional[IMU]:
        return self._imu_view

    @property
    def magnetometer(self) -> Optional[Magnetometer]:
        return self._mag_view

    @property
    def calibration(self):
        """The Calibration in effect, or None if the robot is running raw."""
        return self._calibration

    @property
    def left_encoder(self) -> Optional[WheelEncoder]:
        return self._left_enc

    @property
    def right_encoder(self) -> Optional[WheelEncoder]:
        return self._right_enc

    @property
    def range_sensor(self) -> Optional[RangeSensor]:
        return self._tof

    @property
    def sampler(self) -> Optional[Sampler]:
        return self._sampler

    @property
    def display(self) -> Optional[Display]:
        return self._display

    def close(self) -> None:
        # Drivetrain first (it stops the wheels), then the sampler (so it isn't
        # mid-read when we close the device fds it reads), then the devices.
        self._drivetrain.close()
        if self._sampler is not None:
            self._sampler.stop()
        for dev in (self._imu, self._left_enc, self._right_enc, self._tof, self._display):
            if dev is not None:
                try:
                    dev.close()
                except OSError:
                    pass