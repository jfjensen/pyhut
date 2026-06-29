"""
pyhut -- a small, hardware-agnostic robot control library.

Talk to your robot through the abstract interfaces; pick a concrete backend for
the hardware you actually have. The Duckiebot HUT v3.1 (Jetson Nano) backend
ships here. To support different hardware, implement the same interfaces in a
new module -- nothing above the interface layer changes.

Quick start:

    from pyhut import DuckiebotHUT

    with DuckiebotHUT() as bot:
        bot.drivetrain.set_target(0.4, 0.4)   # drive straight
        if bot.range_sensor:
            print(bot.range_sensor.read_mm(), "mm ahead")
        if bot.display:
            bot.display.text(["HELLO", "WORLD"])
"""

from .interfaces import (
    Display,
    Drivetrain,
    IMU,
    IMUReading,
    Magnetometer,
    MagReading,
    RangeSensor,
    Robot,
    Sample,
    Sampler,
    WheelEncoder,
)
from .duckiebot import (
    DuckiebotDrivetrain,
    DuckiebotHUT,
    HallWheelEncoder,
    MPU9250,
    PCA9685,
    SSD1306,
    VL53L0X,
)
from .sampler import ThreadedSampler

__all__ = [
    # interfaces
    "Robot", "Drivetrain", "IMU", "IMUReading", "Magnetometer", "MagReading",
    "WheelEncoder", "RangeSensor", "Display", "Sample", "Sampler",
    # duckiebot backend
    "DuckiebotHUT", "DuckiebotDrivetrain", "PCA9685", "MPU9250",
    "HallWheelEncoder", "VL53L0X", "SSD1306", "ThreadedSampler",
]
