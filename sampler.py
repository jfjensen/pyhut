"""
A background sensor sampler for any robot built on pyhut's interfaces.

The drivetrain dedicates one thread to owning the motors; this dedicates threads
to owning the *sensors*. The raw drivers are pull-only and their readings carry
no time (IMUReading/MagReading have no timestamp), so nothing downstream can line
up an IMU sample with a range or an encoder count. ThreadedSampler reads the
enabled sensors on a fixed cadence, stamps each cross-sensor snapshot with one
``time.monotonic()`` clock, and publishes it as a :class:`Sample` -- both as a
thread-safe ``latest()`` and as a callback/queue stream.

Design, mirroring DuckiebotDrivetrain:

  * The sampler is the sole reader of the sensors it owns. While it runs, get
    data through latest()/subscribe(), not by calling sensor.read() yourself --
    two threads must not interleave the write-then-read halves of one I2C
    transaction on a shared device fd.
  * A fast loop reads the IMU, magnetometer and encoders together. The IMU and
    mag are quick burst reads; encoder ticks are a lock-free atomic int read.
  * The ToF is slow (single-shot, >= 20 ms per read), so it gets its OWN thread
    at its own, lower cadence. The fast loop never calls it; it just folds in the
    most recent cached range. That keeps the fast loop fast.
  * It only ever reads. It issues no motor commands, so it does not contend with
    the drivetrain control thread; they touch different I2C devices and the
    kernel serialises bus transactions.

Std-lib only, Python 3.6-compatible (no dataclasses, no namedtuple defaults=).
This module introduces no register sequences: it orchestrates the existing,
already-verified driver read methods and nothing more.
"""


import threading
import time
from typing import Callable, List, Optional, Tuple

from .interfaces import (
    IMU,
    Magnetometer,
    RangeSensor,
    Sample,
    Sampler,
    WheelEncoder,
)


class ThreadedSampler(Sampler):
    """Threaded Sampler over the IMU / Magnetometer / WheelEncoder / RangeSensor
    interfaces. Any of the sensors may be None; only the ones supplied are read.

    Parameters
    ----------
    imu, magnetometer, left_encoder, right_encoder, range_sensor
        The sensors to sample. Pass only what's fitted; None is fine.
    imu_rate_hz
        Cadence of the fast loop (IMU/mag/encoders + Sample emission).
    tof_rate_hz
        Target cadence of the separate ToF loop. The real rate is capped by the
        sensor's per-read cost (>= 20 ms), so values above ~40 Hz have no effect.
    """

    def __init__(
        self,
        imu=None,                 # type: Optional[IMU]
        magnetometer=None,        # type: Optional[Magnetometer]
        left_encoder=None,        # type: Optional[WheelEncoder]
        right_encoder=None,       # type: Optional[WheelEncoder]
        range_sensor=None,        # type: Optional[RangeSensor]
        imu_rate_hz=50.0,         # type: float
        tof_rate_hz=15.0,         # type: float
    ):
        self._imu = imu
        self._mag = magnetometer
        self._left_enc = left_encoder
        self._right_enc = right_encoder
        self._tof = range_sensor

        self._imu_period = 1.0 / imu_rate_hz if imu_rate_hz > 0 else 0.02
        self._tof_period = 1.0 / tof_rate_hz if tof_rate_hz > 0 else 0.0

        # State shared with consumers, guarded by _lock.
        self._lock = threading.Lock()
        self._latest = None                       # type: Optional[Sample]
        self._tof_cache = (None, None)            # type: Tuple[Optional[int], Optional[float]]

        # Subscriber registry, guarded by its own lock so notifying never holds
        # the state lock (a callback may legitimately call latest()).
        self._sub_lock = threading.Lock()
        self._subscribers = []                    # type: List[Callable[[Sample], None]]

        # Last-known-good sensor values, touched only by the fast loop thread, so
        # a single glitched read reuses the previous value instead of nulling the
        # stream (the same "skip a tick" tolerance the drivetrain loop uses).
        self._last_accel = None                   # type: Optional[Tuple[float, float, float]]
        self._last_gyro = None                    # type: Optional[Tuple[float, float, float]]
        self._last_temp = None                    # type: Optional[float]
        self._last_mag = None                     # type: Optional[Tuple[float, float, float]]

        # Previous encoder snapshot for tick-rate differentiation (fast-loop only).
        self._prev_t = None                       # type: Optional[float]
        self._prev_left = None                    # type: Optional[int]
        self._prev_right = None                   # type: Optional[int]

        self._stop_evt = threading.Event()
        self._imu_thread = None                   # type: Optional[threading.Thread]
        self._tof_thread = None                   # type: Optional[threading.Thread]
        self._running = False
        self._start_lock = threading.Lock()       # guards start()/stop() races

    # -- lifecycle ----------------------------------------------------------
    def start(self) -> None:
        with self._start_lock:
            if self._running:
                return
            self._stop_evt.clear()
            # Reset differentiation history so the first post-start rate is None,
            # not a huge value spanning the idle gap.
            self._prev_t = None
            self._prev_left = None
            self._prev_right = None

            self._imu_thread = threading.Thread(
                target=self._imu_loop, name="sampler-imu", daemon=True)
            self._imu_thread.start()
            if self._tof is not None:
                self._tof_thread = threading.Thread(
                    target=self._tof_loop, name="sampler-tof", daemon=True)
                self._tof_thread.start()
            self._running = True

    def stop(self) -> None:
        with self._start_lock:
            if not self._running:
                return
            self._stop_evt.set()
            for th in (self._imu_thread, self._tof_thread):
                if th is not None:
                    th.join(timeout=1.0)
            self._imu_thread = None
            self._tof_thread = None
            self._running = False

    # -- consumer API -------------------------------------------------------
    def latest(self) -> Optional[Sample]:
        with self._lock:
            return self._latest

    def subscribe(self, callback: Callable[[Sample], None]) -> Callable[[], None]:
        with self._sub_lock:
            self._subscribers.append(callback)

        def _unsubscribe():
            with self._sub_lock:
                try:
                    self._subscribers.remove(callback)
                except ValueError:
                    pass

        return _unsubscribe

    # -- fast loop: IMU + mag + encoders, emits the Sample stream -----------
    def _imu_loop(self) -> None:
        while not self._stop_evt.is_set():
            t = time.monotonic()

            accel, gyro, temp = self._last_accel, self._last_gyro, self._last_temp
            if self._imu is not None:
                try:
                    r = self._imu.read()
                    accel, gyro, temp = r.accel, r.gyro, r.temp_c
                    self._last_accel, self._last_gyro, self._last_temp = accel, gyro, temp
                except OSError:
                    pass  # transient bus hiccup: reuse last, keep the cadence

            mag = self._last_mag
            if self._mag is not None:
                try:
                    mag = self._mag.read().field
                    self._last_mag = mag
                except OSError:
                    pass

            enc_left = self._left_enc.ticks if self._left_enc is not None else None
            enc_right = self._right_enc.ticks if self._right_enc is not None else None
            left_rate = self._rate(self._prev_left, enc_left, self._prev_t, t)
            right_rate = self._rate(self._prev_right, enc_right, self._prev_t, t)
            self._prev_left, self._prev_right, self._prev_t = enc_left, enc_right, t

            with self._lock:
                tof_mm, tof_t = self._tof_cache

            sample = Sample(
                t,
                accel, gyro, temp,
                mag,
                enc_left, enc_right, left_rate, right_rate,
                tof_mm, tof_t,
            )
            with self._lock:
                self._latest = sample
            self._notify(sample)

            self._stop_evt.wait(self._imu_period)

    @staticmethod
    def _rate(prev_count, count, prev_t, t):
        # type: (Optional[int], Optional[int], Optional[float], float) -> Optional[float]
        """Ticks/second from two counts over their dt. None when unknowable or
        when the counter was reset under us (negative delta)."""
        if count is None or prev_count is None or prev_t is None:
            return None
        dt = t - prev_t
        if dt <= 0:
            return None
        d = count - prev_count
        if d < 0:
            return None  # encoder.reset() happened mid-stream; skip this tick
        return d / dt

    # -- slow loop: ToF on its own cadence, never blocks the fast loop ------
    def _tof_loop(self) -> None:
        while not self._stop_evt.is_set():
            start = time.monotonic()
            try:
                mm = self._tof.read_mm()
                ts = time.monotonic()
                # A None range means "no valid measurement" (Stage 3 status
                # filtering plugs in here); keep the previous cached value.
                if mm is not None:
                    with self._lock:
                        self._tof_cache = (mm, ts)
            except (OSError, RuntimeError):
                pass  # bus hiccup or read timeout: keep last cached range
            # Pace to the target cadence, but never sleep negative: the read
            # itself may already exceed the period.
            elapsed = time.monotonic() - start
            self._stop_evt.wait(max(0.0, self._tof_period - elapsed))

    # -- notification -------------------------------------------------------
    def _notify(self, sample: Sample) -> None:
        with self._sub_lock:
            subscribers = list(self._subscribers)  # snapshot; call outside lock
        for cb in subscribers:
            try:
                cb(sample)
            except Exception:
                # A misbehaving consumer must never kill the sampler thread.
                pass
