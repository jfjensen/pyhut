#!/usr/bin/env python3
"""
Calibration tool for the Duckiebot HUT v3.1. Run with sudo:

    sudo python3 -m pyhut.calibrate                 # save to ./pyhut_calibration.json
    sudo python3 -m pyhut.calibrate /etc/pyhut/calibration.json
    sudo python3 -m pyhut.calibrate --gyro-seconds 3 --mag-seconds 25 out.json

Two phases, both reading off the background Sampler (which is the sole owner of
the sensors while it runs, so this tool never calls a sensor's read() directly):

  1. Gyro bias  -- hold the robot perfectly still; it averages the gyro to find
     the zero-rate offset.
  2. Hard/soft-iron -- turn the robot slowly through at least one full
     revolution BY HAND while a live coverage meter climbs to ~100%. Spinning the
     wheels sits next to the magnetometer and can corrupt it (that's the Stage 2b
     EMI question), so a hand spin is the right move here.

It writes a std-lib-JSON calibration file, then demonstrates the payoff by
applying the fresh model to the live stream: gyro should collapse toward zero and
the heading should stop swinging with the robot's own magnetic signature. On the
next boot, load it with:

    with DuckiebotHUT(calibration="<that file>") as bot: ...

and bot.imu / bot.magnetometer / bot.sampler are corrected, with the raw drivers
untouched. A fresh robot with no file still works -- correction is opt-in.
"""

import argparse
import math
import sys
import time

from . import DuckiebotHUT
from .calibration import (
    DEFAULT_PATH,
    Calibration,
    calibrate_gyro_bias,
    calibrate_magnetometer,
    update_calibration,
)


def _heading_deg(field):
    """Tilt-naive compass heading from an (x, y, z) field tuple, or None."""
    if field is None:
        return None
    mx, my, _ = field
    return (math.degrees(math.atan2(my, mx)) + 360.0) % 360.0


class _MagCoverageMeter(object):
    """A cheap live readout of how much of the XY circle the spin has swept.

    Tracks running min/max on X and Y and counts filled heading wedges around the
    running midpoint. Designed to run as an on_sample callback on the sampler
    thread, so it stays cheap and throttles its own printing.
    """

    def __init__(self, out, sectors=36, print_period_s=0.25):
        self._out = out
        self._sectors = sectors
        self._period = print_period_s
        self._mnx = self._mny = None
        self._mxx = self._mxy = None
        self._seen = set()
        self._last_print = 0.0

    def __call__(self, field):
        x, y, _ = field
        self._mnx = x if self._mnx is None else min(self._mnx, x)
        self._mxx = x if self._mxx is None else max(self._mxx, x)
        self._mny = y if self._mny is None else min(self._mny, y)
        self._mxy = y if self._mxy is None else max(self._mxy, y)
        cx = (self._mnx + self._mxx) / 2.0
        cy = (self._mny + self._mxy) / 2.0
        dx, dy = x - cx, y - cy
        if dx or dy:
            ang = math.atan2(dy, dx)
            idx = int((ang + math.pi) / (2.0 * math.pi) * self._sectors) % self._sectors
            self._seen.add(idx)
        now = time.monotonic()
        if now - self._last_print >= self._period:
            self._last_print = now
            cov = len(self._seen) / float(self._sectors)
            self._out("  ...coverage %3.0f%%  (keep turning)" % (cov * 100.0))


def run_calibration(sampler, has_imu, has_mag,
                    gyro_seconds=2.0, mag_seconds=20.0,
                    prompt=input, out=print):
    """Run the two phases off ``sampler`` and return a :class:`Calibration`.

    Pure orchestration: takes the sampler (already started) plus capability
    flags, so it can be exercised without hardware. ``prompt`` and ``out`` are
    injected for the same reason. Whichever sensor isn't present is skipped, and
    its part of the model stays identity.
    """
    cal = Calibration.default()

    if has_imu:
        prompt("\n[1/2] GYRO BIAS -- set the robot down, hands off, and press Enter...")
        out("  sampling gyro for %.0fs (stay still)..." % gyro_seconds)
        gf = calibrate_gyro_bias(sampler, duration_s=gyro_seconds)
        out("  gyro bias  %+.3f %+.3f %+.3f dps   (std %.3f %.3f %.3f, n=%d)"
            % (gf.bias + gf.std + (gf.n,)))
        if not gf.still_ok:
            out("  WARNING: gyro moved during capture (std too high) -- redo for a clean bias.")
        cal = update_calibration(base=cal, gyro=gf)
    else:
        out("\n[1/2] GYRO BIAS -- no IMU present, skipping.")

    if has_mag:
        prompt("\n[2/2] HARD/SOFT-IRON -- press Enter, then slowly hand-turn the robot "
               "through a FULL circle...")
        out("  sampling field for %.0fs while you rotate..." % mag_seconds)
        meter = _MagCoverageMeter(out)
        mf = calibrate_magnetometer(sampler, duration_s=mag_seconds, on_sample=meter)
        out("  hard-iron  %+.2f %+.2f %+.2f uT" % mf.hard_iron)
        out("  soft-iron   %.3f  %.3f  %.3f      (axes_ok %s)"
            % (mf.soft_iron + (mf.axes_ok,)))
        out("  coverage   %.0f%%  over n=%d samples" % (mf.coverage * 100.0, mf.n))
        if mf.coverage < 0.8:
            out("  WARNING: under 80%% of the circle swept -- turn more fully and redo.")
        pinned = [axis for axis, ok in zip("XYZ", mf.axes_ok) if not ok]
        if pinned:
            out("  note: %s under-swept/ill-conditioned -- scale left at 1.0 (expected for "
                "Z in a flat spin; harmless for heading, which uses X/Y)."
                % "/".join(pinned))
        cal = update_calibration(base=cal, mag=mf)
    else:
        out("\n[2/2] HARD/SOFT-IRON -- no magnetometer present, skipping.")

    return cal


def _circular_std_deg(headings):
    """Circular standard deviation (deg) of a list of headings in degrees.

    Plain std mishandles the 0/360 wrap; this uses the mean-resultant length, so
    a tight cluster near 359/1 deg reads as small, not huge.
    """
    if len(headings) < 2:
        return 0.0
    c = sum(math.cos(math.radians(h)) for h in headings) / len(headings)
    s = sum(math.sin(math.radians(h)) for h in headings) / len(headings)
    r = math.hypot(c, s)
    if r <= 0.0:
        return float("nan")
    return math.degrees(math.sqrt(-2.0 * math.log(min(1.0, r))))


def _show_payoff(sampler, cal, out, seconds=3.0):
    """Show raw vs. calibrated gyro/heading from the live stream, then a verdict.

    Keep the robot STILL during this window. As well as the per-sample lines, it
    summarises the corrected horizontal field radius and the heading jitter for
    raw vs. calibrated -- the numbers that say whether the compass is usable. Note
    that raw heading often looks *steadier* simply because a large hard-iron
    offset lengthens the vector; the calibrated jitter is the honest noise floor.
    """
    out("\nPayoff (raw -> calibrated), hold still, reading the live stream for %.0fs:"
        % seconds)
    raw_h, cal_h, cal_r = [], [], []
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        sm = sampler.latest()
        if sm is not None:
            if sm.gyro is not None:
                cg = cal.apply_gyro(sm.gyro)
                out("  gyro  %+6.2f %+6.2f %+6.2f  ->  %+6.2f %+6.2f %+6.2f dps"
                    % (sm.gyro + cg))
            if sm.mag is not None:
                cm = cal.apply_mag(sm.mag)
                hr, hc = _heading_deg(sm.mag), _heading_deg(cm)
                raw_h.append(hr)
                cal_h.append(hc)
                cal_r.append(math.hypot(cm[0], cm[1]))
                out("  head  %5.0f deg  ->  %5.0f deg" % (hr, hc))
        time.sleep(0.5)

    if cal_r:
        out("\nMagnetometer usability (while still):")
        out("  corrected horizontal field ~%.1f uT" % (sum(cal_r) / len(cal_r)))
        out("  heading jitter  raw %.1f deg  ->  calibrated %.1f deg  (circular std)"
            % (_circular_std_deg(raw_h), _circular_std_deg(cal_h)))
        out("  (a larger raw vector reads as steadier; the calibrated jitter is the\n"
            "   true noise floor. A few degrees is normal; many degrees -> coarse\n"
            "   heading only, which feeds the Stage 2b 'is the mag usable live' call.)")


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Calibrate the Duckiebot HUT IMU gyro bias and magnetometer "
                    "hard/soft-iron, then save a JSON model.")
    parser.add_argument("path", nargs="?", default=DEFAULT_PATH,
                        help="output JSON path (default: %(default)s)")
    parser.add_argument("--gyro-seconds", type=float, default=2.0,
                        help="still-capture duration for gyro bias (default: 2)")
    parser.add_argument("--mag-seconds", type=float, default=20.0,
                        help="rotation-capture duration for hard/soft-iron (default: 20)")
    parser.add_argument("--no-payoff", action="store_true",
                        help="skip the before/after demonstration at the end")
    args = parser.parse_args(argv)

    with DuckiebotHUT() as bot:
        if bot.sampler is None or (bot.imu is None and bot.magnetometer is None):
            print("No IMU or magnetometer present -- nothing to calibrate.")
            return 1

        has_imu = bot.imu is not None
        has_mag = bot.magnetometer is not None
        print("Calibrating:",
              "gyro" if has_imu else "-",
              "magnetometer" if has_mag else "-")
        if bot.display:
            bot.display.text(["CALIBRATING", "follow the", "prompts"])

        sampler = bot.sampler
        sampler.start()
        time.sleep(0.05)  # let the first fast-loop sample land
        try:
            cal = run_calibration(
                sampler, has_imu, has_mag,
                gyro_seconds=args.gyro_seconds, mag_seconds=args.mag_seconds)
            cal.save(args.path)
            print("\nSaved calibration to %s" % args.path)
            if not args.no_payoff:
                _show_payoff(sampler, cal, print)
        finally:
            sampler.stop()

        if bot.display:
            bot.display.text(["CALIBRATED", "", "saved"])
        time.sleep(0.3)

    print("\nNext boot:  DuckiebotHUT(calibration=%r)" % args.path)
    print("Hardware released.")
    return 0


if __name__ == "__main__":
    sys.exit(main())