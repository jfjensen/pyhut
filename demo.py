#!/usr/bin/env python3
"""
Demo for the hut library on a Duckiebot HUT v3.1. Run with sudo:

    sudo python3 -m pyhut.demo

Drives forward briefly, prints sensor readings, writes to the OLED, then stops.
The watchdog guarantees the wheels stop if this script is killed mid-run.
"""

import time

from . import DuckiebotHUT


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

        # Drive straight for ~2 s; re-assert the target so the watchdog stays fed.
        for _ in range(20):
            bot.drivetrain.set_target(0.4, 0.4)
            if bot.imu:
                r = bot.imu.read()
                print("accel %.2f %.2f %.2f g  gyro %.1f %.1f %.1f dps  %.1f C"
                      % (*r.accel, *r.gyro, r.temp_c or 0.0))
            if bot.magnetometer:
                print("heading %.0f deg" % bot.magnetometer.heading_deg())
            if bot.range_sensor:
                print("range %d mm" % bot.range_sensor.read_mm())
            if bot.left_encoder and bot.right_encoder:
                print("ticks L=%d R=%d" % (bot.left_encoder.ticks, bot.right_encoder.ticks))
            time.sleep(0.1)

        bot.drivetrain.stop()
        if bot.display:
            bot.display.text(["DONE"])
        time.sleep(0.3)
    print("Hardware released.")


if __name__ == "__main__":
    main()
