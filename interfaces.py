"""
Hardware-agnostic interfaces for a small differential-drive robot.

These abstract base classes are the contract the rest of your code talks to.
Swapping the Duckiebot HUT for different hardware means writing new concrete
classes that implement these ABCs -- nothing above this layer has to change.

Conventions used throughout:
    * Wheel speeds are floats in [-1.0, 1.0]; sign is direction, magnitude is
      fraction of full power.
    * Linear acceleration is returned in g, angular rate in deg/s, magnetic
      field in microtesla (uT), distance in millimetres.
"""


from abc import ABC, abstractmethod
from collections import namedtuple
from typing import Callable, Optional, Sequence, Tuple


# ---------------------------------------------------------------------------
# Reading value objects (namedtuples: immutable, attribute access, py3.6-safe)
# ---------------------------------------------------------------------------
# IMUReading.accel/gyro are (x, y, z); accel in g, gyro in deg/s.
# temp_c is degrees Celsius or None if the part doesn't report it.
IMUReading = namedtuple("IMUReading", ["accel", "gyro", "temp_c"])
IMUReading.__new__.__defaults__ = (None,)

# MagReading.field is (x, y, z) in microtesla.
MagReading = namedtuple("MagReading", ["field"])

# ---------------------------------------------------------------------------
# Sample: one timestamped, cross-sensor snapshot produced by a Sampler.
# ---------------------------------------------------------------------------
# Every field except `t` is Optional and defaults to None, so a Sample is valid
# even when a sensor isn't fitted or a single read glitched. This is the type
# the raw readings above lack: IMUReading/MagReading carry no time, so nothing
# downstream can correlate them -- a Sample stamps them with one monotonic clock.
#
#   t              float    time.monotonic() when this snapshot was taken
#   accel          (x,y,z)  g, from the IMU
#   gyro           (x,y,z)  deg/s, from the IMU
#   temp_c         float    IMU die temperature, degrees C
#   mag            (x,y,z)  uT, from the magnetometer
#   enc_left       int      left wheel total tick count
#   enc_right      int      right wheel total tick count
#   enc_left_rate  float    left ticks/second, differentiated over dt
#   enc_right_rate float    right ticks/second, differentiated over dt
#   tof_mm         int      most recent range, mm (its own slow cadence)
#   tof_t          float    time.monotonic() when tof_mm was read
#
# tof_mm/tof_t come from the ToF's own cadence, so tof_t < t in general; a
# consumer that wants only fresh ranges watches tof_t for changes.
Sample = namedtuple("Sample", [
    "t",
    "accel", "gyro", "temp_c",
    "mag",
    "enc_left", "enc_right", "enc_left_rate", "enc_right_rate",
    "tof_mm", "tof_t",
])
# Only `t` is required; the remaining ten fields default to None (py3.6: set
# defaults via __new__, since namedtuple(defaults=...) is 3.7+).
Sample.__new__.__defaults__ = (None,) * 10


# ---------------------------------------------------------------------------
# Actuators
# ---------------------------------------------------------------------------
class Drivetrain(ABC):
    """A two-wheel differential drive.

    Implementations are expected to be safe by construction: if the caller
    stops issuing targets, the wheels must stop on their own (watchdog). The
    caller only ever sets a target; it never blocks on hardware.
    """

    @abstractmethod
    def set_target(self, left: float, right: float) -> None:
        """Request wheel speeds, each in [-1.0, 1.0]. Cheap and non-blocking."""

    @abstractmethod
    def stop(self) -> None:
        """Bring both wheels to an immediate, de-energised stop."""

    @abstractmethod
    def close(self) -> None:
        """Release all hardware. The object is unusable afterwards."""

    def __enter__(self) -> "Drivetrain":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


# ---------------------------------------------------------------------------
# Sensors
# ---------------------------------------------------------------------------
class IMU(ABC):
    @abstractmethod
    def read(self) -> IMUReading:
        """Return one accel/gyro(/temp) sample."""

    def close(self) -> None:  # optional override
        pass

    def __enter__(self) -> "IMU":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


class Magnetometer(ABC):
    @abstractmethod
    def read(self) -> MagReading:
        """Return one 3-axis field sample in uT."""

    def heading_deg(self) -> float:
        """Tilt-naive compass heading in [0, 360) from the X/Y field.

        This is a convenience default; it assumes the sensor is roughly level.
        Override for a tilt-compensated heading.
        """
        import math
        mx, my, _ = self.read().field
        return (math.degrees(math.atan2(my, mx)) + 360.0) % 360.0

    def close(self) -> None:
        pass


class WheelEncoder(ABC):
    """A single-channel tick counter for one wheel.

    Most cheap robot encoders (the Duckiebot's included) are single Hall
    channels: they count edges but cannot sense direction on their own.
    Direction, if needed, is supplied externally (e.g. last commanded sign).
    """

    #: Encoder ticks per full wheel revolution. Implementations set this.
    ticks_per_rev: int = 1

    @property
    @abstractmethod
    def ticks(self) -> int:
        """Total edges counted since construction or last reset."""

    @abstractmethod
    def reset(self) -> None:
        """Zero the tick counter."""

    def revolutions(self) -> float:
        return self.ticks / float(self.ticks_per_rev)

    def close(self) -> None:
        pass

    def __enter__(self) -> "WheelEncoder":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


class RangeSensor(ABC):
    @abstractmethod
    def read_mm(self) -> int:
        """Distance to the nearest object in millimetres."""

    def close(self) -> None:
        pass

    def __enter__(self) -> "RangeSensor":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


# ---------------------------------------------------------------------------
# The sampler: turns pull-only sensors into one timestamped stream
# ---------------------------------------------------------------------------
class Sampler(ABC):
    """A background reader that samples the enabled sensors on a fixed cadence
    and publishes a single, timestamped :class:`Sample` stream.

    A Sampler is the *sole owner* of the sensors it reads, in exactly the sense
    the drivetrain's control thread is the sole owner of the motors: while it is
    running, get sensor data from ``latest()``/``subscribe()`` instead of calling
    a sensor's ``read()`` yourself, so two threads never interleave halves of one
    I2C transaction. It only ever reads -- it never drives anything -- so it does
    not contend with the drivetrain thread.

    Lifecycle is explicit (``start()``/``stop()``) rather than implicit, so a
    robot can be brought up, calibrated, or driven without paying for sampling
    until a consumer actually wants the stream. start()/stop() are idempotent and
    the object is reusable as a context manager.
    """

    @abstractmethod
    def start(self) -> None:
        """Begin sampling. Idempotent: a second call while running is a no-op."""

    @abstractmethod
    def stop(self) -> None:
        """Stop sampling and join the worker thread(s). Idempotent."""

    @abstractmethod
    def latest(self) -> Optional["Sample"]:
        """Return the most recent Sample, or None before the first read.

        Thread-safe: takes the internal lock. The returned Sample is immutable,
        so callers may hold and read it freely.
        """

    @abstractmethod
    def subscribe(self, callback: Callable[["Sample"], None]) -> Callable[[], None]:
        """Register ``callback(sample)`` for every new Sample.

        Returns a zero-argument callable that unsubscribes. The callback runs on
        the sampler's thread, so it must be cheap and non-blocking -- a slow
        callback throttles sampling for everyone. To consume from your own thread
        at your own pace, use :meth:`subscribe_queue`.
        """

    def subscribe_queue(self, maxsize: int = 256):
        """Convenience: stream Samples into a ``queue.Queue`` instead of a callback.

        Returns ``(q, unsubscribe)``. The queue is bounded; when a consumer falls
        behind, the *oldest* Sample is dropped so the sampler never blocks and the
        queue always holds the freshest data. Built on subscribe(), so it works
        for any Sampler implementation.
        """
        import queue as _queue

        q = _queue.Queue(maxsize=maxsize)

        def _on_sample(sample):
            try:
                q.put_nowait(sample)
            except _queue.Full:
                try:
                    q.get_nowait()        # drop the oldest, keep the freshest
                    q.put_nowait(sample)
                except (_queue.Empty, _queue.Full):
                    pass

        return q, self.subscribe(_on_sample)

    def close(self) -> None:
        self.stop()

    def __enter__(self) -> "Sampler":
        self.start()
        return self

    def __exit__(self, *exc) -> None:
        self.close()


class Display(ABC):
    @abstractmethod
    def text(self, lines: Sequence[str]) -> None:
        """Replace the screen with the given lines of text and show it."""

    @abstractmethod
    def clear(self) -> None:
        """Blank the screen."""

    def close(self) -> None:
        pass

    def __enter__(self) -> "Display":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


# ---------------------------------------------------------------------------
# The robot facade
# ---------------------------------------------------------------------------
class Robot(ABC):
    """A whole robot: a drivetrain plus whatever sensors are present.

    Sensor accessors may return None when a unit isn't fitted, so callers can
    feature-detect. A different robot is a different Robot subclass; code built
    on this interface doesn't change.
    """

    @property
    @abstractmethod
    def drivetrain(self) -> Drivetrain: ...

    @property
    def imu(self) -> Optional[IMU]:
        return None

    @property
    def magnetometer(self) -> Optional[Magnetometer]:
        return None

    @property
    def left_encoder(self) -> Optional[WheelEncoder]:
        return None

    @property
    def right_encoder(self) -> Optional[WheelEncoder]:
        return None

    @property
    def range_sensor(self) -> Optional[RangeSensor]:
        return None

    @property
    def sampler(self) -> Optional[Sampler]:
        """The robot's timestamped sensor stream, or None if no sensors are fitted.

        Not started automatically: call ``robot.sampler.start()`` (or use it as a
        context manager) when a consumer wants the stream.
        """
        return None

    @property
    def display(self) -> Optional[Display]:
        return None

    @abstractmethod
    def close(self) -> None:
        """Release every owned device."""

    def __enter__(self) -> "Robot":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
