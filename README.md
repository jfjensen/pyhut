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
scp -r pyhut jfj@<robot>:~/Python/        # or git clone, or a USB copy
ssh jfj@<robot>
cd ~/Python                               # the folder that CONTAINS pyhut/
python3 -c "import pyhut; print('ok')"    # verify it's importable
```

Keep the whole folder together — the bundled `LICENSE` and `NOTICES` need to
travel with the code.

**Option B — pip (for development or publishing).** From inside the `pyhut`
folder (the one with `setup.py`):

```bash
pip3 install .            # or:  pip3 install -e .   for an editable install
```

This also installs a `pyhut-demo` command. The package ships both classic
(`setup.py` / `setup.cfg`) and modern (`pyproject.toml`) packaging, so it
installs on the older pip bundled with JetPack 4.x as well as on current
tooling. If pip still complains, upgrade it first with
`pip3 install --upgrade "pip<22" setuptools` (the last versions that support
Python 3.6), or just use Option A.

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
| `interfaces.py` | The swappable contract: `Drivetrain`, `IMU`, `Magnetometer`, `WheelEncoder`, `RangeSensor`, `Display`, `Sampler`, and the `Robot` facade. |
| `lowlevel.py` | Linux primitives: `I2CDevice`, `Gpio`, `EdgeCounter`. |
| `duckiebot.py` | Concrete drivers + the `DuckiebotHUT` facade. |
| `sampler.py` | `ThreadedSampler`: reads the enabled sensors on a fixed cadence and publishes one timestamped `Sample` stream. |
| `calibration.py` | `Calibration` model (gyro bias, mag hard/soft-iron), the fit/collect functions, and the `CalibratedIMU` / `CalibratedMagnetometer` wrappers. |
| `calibrate.py` | Interactive calibration tool (`sudo python3 -m pyhut.calibrate`) that drives `calibration.py` and saves a JSON model. |
| `battery.py` | Duckiebattery telemetry and software power-off over USB (raw `termios`, no `pyserial`). |
| `font.py` | Embedded 5x7 font for the OLED. |
| `demo.py` | Runnable demo (`sudo python3 -m pyhut.demo`). |
| `test_calibration.py` | Offline tests for `calibration.py` against fake sensors — no hardware required. |
| `pyproject.toml`, `setup.py`, `setup.cfg` | Packaging metadata for `pip install` (classic + modern, py3.6-compatible). |

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

## Sensor sampling

The raw drivers are pull-only and their readings carry no timestamp, so
nothing downstream can line up an IMU sample with a range or an encoder count
taken at a different moment. `bot.sampler` (a `ThreadedSampler`) fixes that: it
reads the enabled sensors on a fixed cadence and publishes each cross-sensor
snapshot as one timestamped `Sample`.

```python
with DuckiebotHUT() as bot:
    with bot.sampler:                      # starts the sampler; stops on exit
        time.sleep(0.1)
        s = bot.sampler.latest()           # most recent Sample, or None
        if s is not None:
            print(s.t, s.accel, s.gyro, s.mag, s.enc_left, s.tof_mm)
```

A `Sample` has `t` (the `time.monotonic()` clock) plus `accel`, `gyro`,
`temp_c`, `mag`, `enc_left`, `enc_right`, `enc_left_rate`, `enc_right_rate`,
`tof_mm`, and `tof_t` — every field but `t` is `None` when that sensor isn't
fitted or a single read glitched. `bot.sampler` is `None` if no sensors came
up at all.

While the sampler is running it is the **sole owner** of the sensors it reads;
get data through `latest()` / `subscribe()` / `subscribe_queue()` instead of
calling a sensor's own `read()`, so two threads never interleave halves of one
I2C transaction. It costs nothing until started, and it never touches the
motors, so it doesn't contend with the drivetrain's control thread.

## Calibration

The raw IMU and magnetometer are uncorrected: the gyro carries a constant
zero-rate bias, and the compass heading swings with the robot's own hard/soft
iron signature. `pyhut.calibration` fits a small model from real samples and
applies it through wrappers (`CalibratedIMU`, `CalibratedMagnetometer`) that
still satisfy the `IMU` / `Magnetometer` interfaces — so `bot.sampler` and
`bot.imu` / `bot.magnetometer` automatically read calibrated data once a
calibration is supplied. With none supplied, everything behaves exactly as
before (identity, pass-through).

Run the interactive tool to capture one and save it as JSON:

```bash
sudo python3 -m pyhut.calibrate                  # saves ./pyhut_calibration.json
sudo python3 -m pyhut.calibrate /etc/pyhut/calibration.json
```

It walks two phases off the background sampler: hold the robot still while it
averages the gyro to find the zero-rate bias, then hand-turn the robot through
a full revolution (don't drive the motors — they sit next to the magnetometer
and corrupt the reading) while a live coverage meter tracks the sweep, fitting
the hard-iron center and soft-iron scale. It finishes by replaying raw-vs-
calibrated gyro and heading from the live stream so you can see the payoff.

Load the saved file on later boots:

```python
with DuckiebotHUT(calibration="/etc/pyhut/calibration.json") as bot:
    ...  # bot.imu, bot.magnetometer, bot.sampler are now corrected
```

A missing or corrupt calibration file loads as an identity model rather than
failing startup, so a fresh robot always comes up. The raw drivers are never
modified — correction happens only in the wrapper layer, so it's opt-in.

## Battery

The Duckiebattery on a DB21 is not a dumb power bank. It is a smart pack with
its own microcontroller that reports over a USB serial link and can cut its own
output on command. The five charge LEDs on the pack are usually buried inside
the chassis once the robot is assembled, so reading the state of charge over USB
is, in practice, the only way to see it. `pyhut` talks to that link directly,
with no Duckietown software and no `pyserial`, in the same bare-metal spirit as
the rest of the library.

The pack is found by its USB identity (VID:PID `04d8:ecb7`), not by a fixed
`/dev/ttyACM0`, because the node number is whatever the kernel happened to
assign. A background thread owns the port and is the sole reader and writer, so
a charge read never collides with a shutdown command. Note that the data line of
the battery cable has to actually reach the Jetson for any of this to work. A
charge-only cable carries power but no data, and then nothing shows up.

Reading the charge:

```python
from pyhut import Duckiebattery

with Duckiebattery() as bat:
    bat.request_info(wait=True)              # learn firmware version, serial
    print("firmware", bat.firmware_version)
    while True:
        r = bat.latest()                     # most recent reading, or None
        if r:
            print(r.percentage, "%", "charging" if r.charging else "on battery")
        time.sleep(1)
```

A few things to note:

- `bat.latest()` returns a `BatteryReading` with `percentage`, `charging`,
  `cell_voltage`, `input_voltage`, `current`, `temperature`, the two USB rail
  voltages, `cycle_count`, and `time_to_empty_s`. All units are already
  converted from the pack's raw millivolts, milliamps, Kelvin, and minutes.
- The convenience shortcuts `bat.percentage`, `bat.charging`, and `bat.current`
  read the latest value for you.
- `bat.is_full()` is deliberately cautious. A pack sits at 100% for a while
  before the cell is actually topped off, so it also waits for the charge
  current to taper toward zero before it returns `True`.
- If your firmware frames things slightly differently than this driver expects,
  pass `on_raw=print` to watch the actual serial lines, and adjust `_parse_frame`
  in `battery.py`. That is the one place the wire format lives.

Shutting the pack down in software:

```python
import os
from pyhut import Duckiebattery

with Duckiebattery() as bat:
    if bat.turn_off(timeout=20, wait=True):  # arm the pack's output cutoff
        os.system("sudo poweroff")           # then halt the Jetson
```

`turn_off()` does not halt the Jetson itself. It arms the pack's output
cutoff, and the countdown starts the instant you arm it, not when you call
`poweroff`. So the sequence is: arm, then halt, and the OS has to finish
halting before the countdown expires, or the rail drops mid-halt. When the
rail does go away the pack returns to its OFF/idle state, and that is what
matters: a plain `sudo poweroff` on its own leaves the pack's output live, the
pack never resets, and on the next attempt the side button looks dead because
it only ever *starts* an output that is already on.

So, once the pack is off, you power the robot back on the normal way, by pressing
the button on the side of the battery. There is no separate power-on command,
because there is nothing left awake on the Jetson to receive one. The whole point
of the clean power-off is to leave the pack in the state where the side button
works again.

One firmware wrinkle worth knowing. On firmware 2.0.0 the timeout you pass is
sent to the pack directly, so `turn_off(timeout=20)` really does mean twenty
seconds. On 2.0.1 and newer (mine reports 2.0.2), Duckietown switched to a bare
command with no duration, and the pack uses its own built-in delay instead. So on
a newer pack the `timeout` is quietly ignored, which is why a shutdown can feel
faster than the number you asked for. If you need a known, longer window on newer
firmware, pass `force_timed=True` to send the timed form anyway, and check the
return value, since a pack that refuses the timed command does not acknowledge,
and you do not want to `poweroff` into a cutoff that was never armed. After the
call, `bat.last_ttl_sec` tells you the delay the pack actually scheduled, when it
reports one.

The command sent over USB depends on the pack's firmware, and `turn_off()` picks
the right one for you (a plain `QQ` on firmware 2.0.1 and newer, a timed `Q20`
form on 2.0.0). Older or unknown firmware is treated as the timed form, which is
the safe default.

There is also a small command-line tool for trying this out without writing any
code. It must run as **root**, because it opens the USB serial device:

```bash
sudo python3 -m pyhut.battery                # live readings, Ctrl-C to stop
sudo python3 -m pyhut.battery --raw          # also echo the raw serial frames
sudo python3 -m pyhut.battery --shutdown 20  # arm the cutoff, print the ACK, exit
```

The `--shutdown` form arms the cutoff and prints the pack's acknowledgement,
including the scheduled delay when the firmware reports one. Note that arming
starts the countdown immediately, so once you run it, follow with `sudo poweroff`
right away. The tool does not halt for you, on purpose, but the rail will still
drop when the timer expires.

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
- The battery driver needs the USB *data* line of the battery cable to reach the
  Jetson. A charge-only cable gives power but no telemetry, and then the pack
  never appears. If `find_battery_ports()` returns an empty list, check that
  first, then check that the pack is actually on and not sitting in protection
  mode.

## License

This project's own code is released under the MIT License — see [`LICENSE`](LICENSE).
(Set the copyright holder in that file before publishing.)

The `VL53L0X` time-of-flight driver in `duckiebot.py` is a port of Adafruit's
MIT-licensed CircuitPython driver and is used under its own terms. The required
attribution and full license text are in [`NOTICES`](NOTICES), which also records
the upstream Pololu / ST credit chain. Keep both files with the source when you
distribute it.