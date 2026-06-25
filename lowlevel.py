"""
Bare-metal Linux I/O primitives, shared by the concrete hardware drivers.

No external packages: raw I2C through /dev/i2c-* with the I2C_SLAVE ioctl, and
GPIO through sysfs with persistent file descriptors. This is the platform layer;
the device drivers in duckiebot.py are written entirely on top of it.
"""


import fcntl
import os
import select
import threading

I2C_SLAVE = 0x0703  # linux/i2c-dev.h


def _write_sysfs(path, value):
    """Write a short string to a sysfs attribute via a raw, immediately-closed fd."""
    fd = os.open(path, os.O_WRONLY)
    try:
        os.write(fd, value.encode())
    finally:
        os.close(fd)


def _export_gpio(number):
    """Export a GPIO line if it isn't already exported."""
    if not os.path.exists("/sys/class/gpio/gpio{}".format(number)):
        try:
            _write_sysfs("/sys/class/gpio/export", str(number))
        except OSError:
            pass  # already exported, or claimed by another driver


class I2CDevice:
    """One open handle to a single I2C slave on a bus.

    Each instance opens its own fd and pins it to one address, so several
    devices on the same bus never clobber each other's slave selection.
    """

    def __init__(self, bus_path: str, address: int):
        self.address = address
        self._fd = os.open(bus_path, os.O_RDWR)
        fcntl.ioctl(self._fd, I2C_SLAVE, address)

    # raw transfers ---------------------------------------------------------
    def write(self, data: bytes) -> None:
        os.write(self._fd, bytes(data))

    def read(self, length: int) -> bytes:
        return os.read(self._fd, length)

    # register helpers ------------------------------------------------------
    def write_reg(self, reg: int, data) -> None:
        if isinstance(data, int):
            data = bytes([data & 0xFF])
        os.write(self._fd, bytes([reg & 0xFF]) + bytes(data))

    def read_reg(self, reg: int, length: int = 1) -> bytes:
        # write register pointer, then read. i2c-dev turns these two syscalls
        # into the conventional write-then-read the parts expect.
        os.write(self._fd, bytes([reg & 0xFF]))
        return os.read(self._fd, length)

    def read_u8(self, reg: int) -> int:
        return self.read_reg(reg, 1)[0]

    def write_u8(self, reg: int, val: int) -> None:
        self.write_reg(reg, val & 0xFF)

    def close(self) -> None:
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None


class Gpio:
    """A sysfs GPIO line driven as an output via a persistent value fd.

    The value fd is opened raw (os.open) and written with lseek(0)+write; do
    NOT use open(path, 'w', buffering=0), which raises on Python 3 text mode.
    """

    def __init__(self, number: int, direction: str = "out", initial: bool = False):
        self.number = number
        _export_gpio(number)
        _write_sysfs("/sys/class/gpio/gpio{}/direction".format(number), direction)
        self._fd = os.open("/sys/class/gpio/gpio{}/value".format(number), os.O_RDWR)
        if direction == "out":
            self.set(initial)

    def set(self, high: bool) -> None:
        os.lseek(self._fd, 0, os.SEEK_SET)
        os.write(self._fd, b"1" if high else b"0")

    def set_high(self) -> None:
        self.set(True)

    def set_low(self) -> None:
        self.set(False)

    def close(self) -> None:
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None


class EdgeCounter:
    """Counts rising edges on a sysfs GPIO input in a background thread.

    sysfs is told edge='rising'; a poll() on the value fd wakes on each edge
    (POLLPRI) and bumps the count. Polling in pure Python from the main loop
    would miss edges at speed, so the wait lives on its own thread.
    """

    def __init__(self, number: int):
        self.number = number
        self._count = 0
        self._stop = threading.Event()
        _export_gpio(number)
        _write_sysfs("/sys/class/gpio/gpio{}/direction".format(number), "in")
        _write_sysfs("/sys/class/gpio/gpio{}/edge".format(number), "rising")
        self._fd = os.open("/sys/class/gpio/gpio{}/value".format(number), os.O_RDONLY)
        self._poll = select.poll()
        self._poll.register(self._fd, select.POLLPRI | select.POLLERR)
        os.lseek(self._fd, 0, os.SEEK_SET)
        os.read(self._fd, 8)  # clear any initial pending state
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        while not self._stop.is_set():
            events = self._poll.poll(200)  # ms; lets us notice _stop
            if not events:
                continue
            os.lseek(self._fd, 0, os.SEEK_SET)
            os.read(self._fd, 8)  # required to re-arm the interrupt
            self._count += 1

    @property
    def count(self) -> int:
        return self._count

    def reset(self) -> None:
        self._count = 0

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None
