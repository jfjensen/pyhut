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
from typing import Optional, Sequence, Tuple


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
    def display(self) -> Optional[Display]:
        return None

    @abstractmethod
    def close(self) -> None:
        """Release every owned device."""

    def __enter__(self) -> "Robot":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
