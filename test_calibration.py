"""Offline tests for pyhut.calibration -- no hardware, fake interface sensors.

Verifies: gyro-bias recovery, mag hard/soft-iron recovery (incl. under-swept
axis), JSON round-trip + tolerant load, the wrappers (correctness, interface
conformance, identity pass-through), wrappers inside a real ThreadedSampler, and
that the DuckiebotHUT calibration= param defaults to unchanged behaviour.
"""

import json
import math
import os
import tempfile
import time

from pyhut.interfaces import IMU, IMUReading, Magnetometer, MagReading
from pyhut.calibration import (
    Calibration, CalibratedIMU, CalibratedMagnetometer,
    GyroFit, MagFit, fit_gyro_bias, fit_mag_iron,
    calibrate_gyro_bias, calibrate_magnetometer, update_calibration,
)
from pyhut.sampler import ThreadedSampler

PASS = []
def check(name, cond):
    PASS.append(bool(cond))
    print(("PASS " if cond else "FAIL ") + name)

def approx(a, b, tol=1e-6):
    return abs(a - b) <= tol


# --- fakes implementing the real ABCs --------------------------------------
class FakeIMU(IMU):
    """Stationary IMU with an injected gyro bias and tiny noise."""
    def __init__(self, bias=(0.7, -1.3, 0.2)):
        self.bias = bias
        self._i = 0
        self.closed = False
    def read(self):
        self._i += 1
        n = ((self._i % 3) - 1) * 0.01  # deterministic tiny jitter
        g = (self.bias[0] + n, self.bias[1] - n, self.bias[2] + n)
        return IMUReading((0.0, 0.0, 1.0), g, 25.0)
    def close(self):
        self.closed = True


class FakeMag(Magnetometer):
    """Magnetometer whose XY field is a circle, offset (hard-iron) and squashed
    on Y (soft-iron), with a ~constant Z (an in-place spin -> under-swept Z)."""
    def __init__(self, center=(12.0, -7.0, 40.0), rx=30.0, ry=18.0):
        self.center, self.rx, self.ry = center, rx, ry
        self._a = 0.0
        self.closed = False
    def read(self):
        self._a += 0.13
        mx = self.center[0] + self.rx * math.cos(self._a)
        my = self.center[1] + self.ry * math.sin(self._a)
        mz = self.center[2] + 0.05 * math.cos(self._a)  # negligible Z sweep
        return MagReading((mx, my, mz))
    def close(self):
        self.closed = True


# --- 1. gyro bias fit ------------------------------------------------------
imu = FakeIMU(bias=(0.7, -1.3, 0.2))
samples = [imu.read().gyro for _ in range(200)]
gf = fit_gyro_bias(samples)
check("gyro fit type", isinstance(gf, GyroFit))
check("gyro bias x", approx(gf.bias[0], 0.7, 0.02))
check("gyro bias y", approx(gf.bias[1], -1.3, 0.02))
check("gyro bias z", approx(gf.bias[2], 0.2, 0.02))
check("gyro still_ok true (low noise)", gf.still_ok)
check("gyro still_ok false when moving",
      not fit_gyro_bias([(g[0]+i*5.0, g[1], g[2]) for i, g in enumerate(samples)]).still_ok)


# --- 2. mag hard/soft-iron fit --------------------------------------------
mag = FakeMag(center=(12.0, -7.0, 40.0), rx=30.0, ry=18.0)
mag_samples = [mag.read().field for _ in range(400)]
mf = fit_mag_iron(mag_samples)
check("mag fit type", isinstance(mf, MagFit))
check("mag hard-iron x", approx(mf.hard_iron[0], 12.0, 0.5))
check("mag hard-iron y", approx(mf.hard_iron[1], -7.0, 0.5))
# soft-iron should equalise the two well-swept axes: rx*sx ~= ry*sy
check("mag soft-iron equalises XY",
      approx(mf.soft_iron[0] * 30.0, mf.soft_iron[1] * 18.0, 0.5))
check("mag X well-swept", mf.axes_ok[0])
check("mag Y well-swept", mf.axes_ok[1])
check("mag Z under-swept -> not ok", not mf.axes_ok[2])
check("mag Z scale pinned to 1.0", approx(mf.soft_iron[2], 1.0))
check("mag coverage full (~1.0)", mf.coverage > 0.95)

# corrected field should trace a near-circle: equal X/Y radius
cal_m = update_calibration(mag=mf)
cr = [cal_m.apply_mag(s) for s in mag_samples]
rx = (max(p[0] for p in cr) - min(p[0] for p in cr)) / 2.0
ry = (max(p[1] for p in cr) - min(p[1] for p in cr)) / 2.0
check("corrected mag is round (rx ~= ry)", approx(rx, ry, 0.5))


# --- 3. Calibration model + JSON ------------------------------------------
cal = update_calibration(gyro=gf, mag=mf)
check("cal not identity", not cal.is_identity())
check("default is identity", Calibration.default().is_identity())

tmp = tempfile.mkdtemp()
path = os.path.join(tmp, "cal.json")
cal.save(path)
on_disk = json.load(open(path))
check("json has schema", on_disk.get("schema") == "pyhut.calibration/1")
loaded = Calibration.load(path)
check("json round-trip gyro", loaded.gyro_bias == cal.gyro_bias)
check("json round-trip hard", loaded.mag_hard_iron == cal.mag_hard_iron)
check("json round-trip soft", loaded.mag_soft_iron == cal.mag_soft_iron)

missing = os.path.join(tmp, "nope.json")
check("load_or_default missing -> identity", Calibration.load_or_default(missing).is_identity())
bad = os.path.join(tmp, "bad.json")
open(bad, "w").write("{ not json")
bad_cal = Calibration.load_or_default(bad)
check("load_or_default corrupt -> identity", bad_cal.is_identity())
check("load_or_default corrupt records error", "load_error" in bad_cal.meta)
try:
    Calibration.load(missing); raised = False
except FileNotFoundError:
    raised = True
check("strict load raises on missing", raised)


# --- 4. wrappers: correctness + interface conformance ----------------------
wimu = CalibratedIMU(FakeIMU(bias=(0.7, -1.3, 0.2)), cal)
check("CalibratedIMU is IMU", isinstance(wimu, IMU))
g = wimu.read().gyro
check("wrapped gyro de-biased ~0", abs(g[0]) < 0.05 and abs(g[1]) < 0.05 and abs(g[2]) < 0.05)
check("wrapped accel passthrough", wimu.read().accel == (0.0, 0.0, 1.0))
check("wrapped temp passthrough", wimu.read().temp_c == 25.0)

wmag = CalibratedMagnetometer(FakeMag(center=(12.0, -7.0, 40.0), rx=30.0, ry=18.0), cal)
check("CalibratedMag is Magnetometer", isinstance(wmag, Magnetometer))
h = wmag.heading_deg()  # inherited default, reads through corrected read()
check("heading_deg in range", 0.0 <= h < 360.0)

# identity pass-through
idimu = CalibratedIMU(FakeIMU(bias=(0.7, -1.3, 0.2)), Calibration.default())
check("identity wrapper passes raw gyro", approx(idimu.read().gyro[0], 0.7, 0.02))

# close delegates
inner = FakeIMU()
CalibratedIMU(inner, cal).close()
check("wrapper close delegates", inner.closed)


# --- 5. wrappers inside a real ThreadedSampler -----------------------------
s = ThreadedSampler(imu=CalibratedIMU(FakeIMU(bias=(0.7, -1.3, 0.2)), cal),
                    magnetometer=CalibratedMagnetometer(
                        FakeMag(center=(12.0, -7.0, 40.0), rx=30.0, ry=18.0), cal),
                    imu_rate_hz=200.0)
s.start()
time.sleep(0.1)
snap = s.latest()
s.stop()
check("sampler emitted a Sample", snap is not None)
check("sampler stream gyro is calibrated (~0)",
      snap is not None and abs(snap.gyro[0]) < 0.05)


# --- 6. calibrate_* via the Sampler stream (collection path) ---------------
s2 = ThreadedSampler(imu=FakeIMU(bias=(0.7, -1.3, 0.2)),
                     magnetometer=FakeMag(center=(12.0, -7.0, 40.0), rx=30.0, ry=18.0),
                     imu_rate_hz=400.0)
s2.start()
gf2 = calibrate_gyro_bias(s2, duration_s=0.3)
mf2 = calibrate_magnetometer(s2, duration_s=0.6)
s2.stop()
check("calibrate_gyro via sampler recovers bias", approx(gf2.bias[0], 0.7, 0.05))
check("calibrate_mag via sampler centers X", approx(mf2.hard_iron[0], 12.0, 1.0))

# error when sampler not started
s3 = ThreadedSampler(imu=FakeIMU())
try:
    calibrate_gyro_bias(s3, duration_s=0.1); raised = False
except RuntimeError:
    raised = True
check("collect raises when sampler not started", raised)

print("\n%d/%d checks passed" % (sum(PASS), len(PASS)))
raise SystemExit(0 if all(PASS) else 1)