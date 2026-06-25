# pyhut

A small, hardware-agnostic control library for a differential-drive robot.

You program against abstract interfaces; a concrete backend implements them for
real hardware. The included backend targets the **Duckiebot HUT v3.1 on a
Jetson Nano**, using only raw I2C (`fcntl`) and sysfs GPIO — no external
packages, no Duckietown stack.

## Prerequisites

`pyhut`'s bundled backend talks directly to one specific board on one specific
computer, so you need both:

- **The hardware — Duckietown HUT (v3.1).** This is the I/O board on a
  Duckietown Duckiebot (DB21M / DB21J). Duckiebots and parts are sold through
  the official Duckietown store at <https://get.duckietown.com>. The HUT carries
  the motor driver, IMU, ToF, OLED, and encoder connections this library drives.
- **The operating system — NVIDIA JetPack for Jetson Nano.** This library was
  developed against the stock Jetson Nano Linux image (JetPack 4.x, Jetson Linux
  r32, Ubuntu 18.04, Python 3.6), which provides the `/dev/i2c-1` bus and sysfs
  GPIO layout it relies on. Get the SD-card image from NVIDIA's *Get Started
  with Jetson Nano Developer Kit* page,
  <https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit>;
  older Nano releases live in the
  [Jetson Download Center Archive](https://developer.nvidia.com/embedded/downloads/archive).

If your Duckiebot is already flashed with Duckietown's own SD image, the
underlying OS is still this JetPack base, so `pyhut` runs on it directly — just
don't run it at the same time as the Duckietown stack, since both want the I2C
bus.

## Install

`pyhut` is pure standard library, so there is nothing to download or compile.
Pick whichever fits how you work.

**Option A — drop-in (simplest, recommended on the robot).** Copy the `pyhut`
folder onto the Jetson next to your own code, then import it. No `pip`, no root
needed to install:

```bash
scp -r pyhut <username>@<robot>:~/Python/        # or git clone, or a USB copy
ssh <username>@<robot>
cd ~/Python                               # the folder that CONTAINS pyhut/
python3 -c "import pyhut; print('ok')"    # verify it's importable
```

Keep the whole folder together — the bundled `LICENSE` and `NOTICES` need to
travel with the code.

**Option B — pip (for development or publishing).** From inside the `pyhut`
folder (the one with `pyproject.toml`):

```bash
pip3 install .            # or:  pip3 install -e .   for an editable install
```

This also installs a `pyhut-demo` command. Option B needs a reasonably recent
`pip`/`setuptools`; the stock pip on JetPack 4.x is old, so you may need
`pip3 install --upgrade pip setuptools` first, or just use Option A.

## Run the demo

The demo drives the wheels briefly, prints sensor readings, writes to the OLED,
then stops. It must run as **root**, because it uses raw I2C and sysfs GPIO:

```bash
# Option A (drop-in): from the folder that contains pyhut/
sudo python3 -m pyhut.demo

# Option B (pip-installed): the console script
sudo pyhut-demo
```

You should see the detected devices listed, a stream of IMU / heading / range /
encoder readings, and `HUT READY` on the screen. The watchdog guarantees the
wheels stop if you kill the script mid-run.

## Layout

| File | What it is |
|------|-----------|
| `interfaces.py` | The swappable contract: `Drivetrain`, `IMU`, `Magnetometer`, `WheelEncoder`, `RangeSensor`, `Display`, and the `Robot` facade. |
| `lowlevel.py` | Linux primitives: `I2CDevice`, `Gpio`, `EdgeCounter`. |
| `duckiebot.py` | Concrete drivers + the `DuckiebotHUT` facade. |
| `font.py` | Embedded 5x7 font for the OLED. |
| `demo.py` | Runnable demo (`sudo python3 -m pyhut.demo`). |
| `pyproject.toml` | Packaging metadata for `pip install`. |

## Use

```python
from pyhut import DuckiebotHUT

with DuckiebotHUT() as bot:
    bot.drivetrain.set_target(0.4, 0.4)      # wheel speeds in [-1, 1]
    if bot.range_sensor:
        print(bot.range_sensor.read_mm(), "mm")
    if bot.imu:
        print(bot.imu.read())                # accel (g), gyro (deg/s), temp (C)
    if bot.magnetometer:
        print(bot.magnetometer.heading_deg())
    if bot.display:
        bot.display.text(["HELLO", "WORLD"])
```

The drivetrain runs a background control thread that is the sole owner of the
motor hardware. Your code only calls `set_target`; if it stalls or crashes, a
watchdog zeros the wheels after `watchdog_timeout` (default 0.3 s).

## Swapping hardware

Implement the interfaces in `interfaces.py` for your board (e.g. a
`MyRobotDrivetrain(Drivetrain)` and a `MyRobot(Robot)` facade). Code written
against the interfaces does not change.

## Notes

- Run as root (raw I2C + sysfs GPIO).
- Sensors are probed tolerantly: a unit that isn't on the bus (the ToF is often
  absent until its XSHUT line is released) makes its accessor return `None`
  rather than failing the whole robot.
- LED control is intentionally omitted.

## License

This project's own code is released under the MIT License — see [`LICENSE`](LICENSE).
(Set the copyright holder in that file before publishing.)

The `VL53L0X` time-of-flight driver in `duckiebot.py` is a port of Adafruit's
MIT-licensed CircuitPython driver and is used under its own terms. The required
attribution and full license text are in [`NOTICES`](NOTICES), which also records
the upstream Pololu / ST credit chain. Keep both files with the source when you
distribute it.
