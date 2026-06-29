#!/usr/bin/env python3
"""
Demo for the hut library on a Duckiebot HUT v3.1. Run with sudo:
    sudo python3 -m pyhut.demo
Drives forward briefly, prints sensor readings, writes to the OLED, then stops.
The watchdog guarantees the wheels stop if this script is killed mid-run.

Sensor readings come from the background Sampler: one timestamped snapshot
per loop. The drivetrain keeps driving (it owns the motors); 
the sampler owns the sensor reads -- so the loop feeds the
watchdog with set_target while pulling data from sampler.latest().
"""
import math
import time

from . import DuckiebotHUT


def _heading_deg(mag):
    """Tilt-naive compass heading from the raw mag field, like
    Magnetometer.heading_deg(). The Sample carries the raw field, not a heading,
    because a calibrated heading is Stage 2a's job."""
    if mag is None:
        return None
    mx, my, _ = mag
    return (math.degrees(math.atan2(my, mx)) + 360.0) % 360.0


def main() -> None:
    with DuckiebotHUT() as bot:
        print("Devices present:",
              "imu" if bot.imu else "-",
              "mag" if bot.magnetometer else "-",
              "encoders" if bot.left_encoder else "-",
              "tof" if bot.range_sensor else "-",
              "oled" if bot.display else "-")
        if bot.display:
            bot.display.text(["HUT READY", "", "demo running"])

        # Start the sampler; it becomes the sole reader of the sensors. Don't
        # call bot.imu.read()/range_sensor.read_mm() directly while it runs.
        sampler = bot.sampler
        if sampler is not None:
            sampler.start()
            time.sleep(0.05)  # let the first fast-loop sample land

        try:
            # Drive straight for ~2 s; re-assert the target so the watchdog stays fed.
            t0 = None
            for _ in range(20):
                bot.drivetrain.set_target(0.4, 0.4)

                sm = sampler.latest() if sampler is not None else None
                if sm is not None:
                    if t0 is None:
                        t0 = sm.t  # first snapshot is the time origin for display
                    print("t=%.6f s (monotonic)  +%.3f s" % (sm.t, sm.t - t0))
                    if sm.accel is not None and sm.gyro is not None:
                        print("accel %.2f %.2f %.2f g  gyro %.1f %.1f %.1f dps  %.1f C"
                              % (sm.accel + sm.gyro + (sm.temp_c or 0.0,)))
                    heading = _heading_deg(sm.mag)
                    if heading is not None:
                        print("heading %.0f deg" % heading)
                    if sm.tof_mm is not None:
                        if sm.tof_t is not None:
                            print("range %d mm  (tof_t=%.6f s, %.0f ms before this snapshot)"
                                  % (sm.tof_mm, sm.tof_t, (sm.t - sm.tof_t) * 1000.0))
                        else:
                            print("range %d mm" % sm.tof_mm)
                    if sm.enc_left is not None and sm.enc_right is not None:
                        print("ticks L=%d R=%d  (rate L=%.0f R=%.0f /s)"
                              % (sm.enc_left, sm.enc_right,
                                 sm.enc_left_rate or 0.0, sm.enc_right_rate or 0.0))
                time.sleep(0.1)
        finally:
            bot.drivetrain.stop()
            if sampler is not None:
                sampler.stop()

        if bot.display:
            bot.display.text(["DONE"])
        time.sleep(0.3)
    print("Hardware released.")


if __name__ == "__main__":
    main()