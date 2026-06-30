"""
Reusable sensor calibration for any robot built on pyhut's interfaces.

The raw drivers return raw data on purpose -- ``MPU9250.read()`` does no
gyro-bias removal and ``Magnetometer.heading_deg()`` is a bare ``atan2`` with no
hard/soft-iron correction -- so any fusion built directly on them inherits a
constant gyro drift and a compass that swings with the robot's own magnetic
signature. This module fixes that *without touching the drivers*: it fits a small
calibration model from real samples, persists it as JSON, and applies it through
thin wrappers that still satisfy the IMU / Magnetometer interfaces. Calibration
is therefore opt-in -- you wrap a sensor only if you want correction, and a fresh
robot with no calibration file still works (the model defaults to identity).

What's here:

  * :class:`Calibration` -- the model: gyro bias, magnetometer hard-iron offset
    (center) and soft-iron per-axis scale, plus std-lib ``json`` load/save. An
    absent file loads as identity, so startup never fails on a fresh robot.
  * :func:`fit_gyro_bias` / :func:`calibrate_gyro_bias` -- average the gyro while
    the robot is still to recover the zero-rate offset.
  * :func:`fit_mag_iron` / :func:`calibrate_magnetometer` -- fit the hard-iron
    center and soft-iron scale from field samples taken through a full rotation.
  * :class:`CalibratedIMU` / :class:`CalibratedMagnetometer` -- the wrappers that
    apply the model at ``read()`` time while remaining drop-in IMU / Magnetometer
    instances (so e.g. a ThreadedSampler built over them yields a *calibrated*
    Sample stream, and ``heading_deg()`` becomes corrected for free).

Design notes:

  * **Hardware-agnostic.** Nothing here imports a concrete driver or
    ``lowlevel``; it speaks only the ABCs in :mod:`pyhut.interfaces`. It is HAL
    infrastructure, not Duckiebot-specific behaviour.
  * **Use the Sampler.** The fit routines collect from the Stage 1
    :class:`~pyhut.interfaces.Sampler` stream when one is given, because while a
    sampler runs it is the *sole owner* of the sensors -- reading the device
    directly at the same time would interleave halves of an I2C transaction. A
    direct-IMU / direct-Magnetometer fallback exists for when no sampler is
    running.
  * **Std-lib only, Python 3.6-compatible.** Uses ``json``, ``math``,
    ``statistics``, ``threading``, ``time``; no dataclasses, no
    ``namedtuple(defaults=...)``, no 3.7+ features.
  * **No register sequences.** This module never pokes hardware; it averages and
    transforms numbers that the existing, already-verified drivers produced.

Soft-iron scope: this fits the *diagonal* soft-iron correction (one scale per
axis), which is what "soft-iron scale" means and what a single-axis spin can
actually constrain. It does not fit the off-diagonal cross terms of a full 3x3
soft-iron matrix; that needs an ellipsoid least-squares fit and orientation
coverage a flat in-place spin doesn't provide. See :func:`fit_mag_iron`.
"""


import json
import math
import statistics
import threading
import time
from collections import namedtuple

from .interfaces import IMU, IMUReading, Magnetometer, MagReading


# Default on-disk location for a saved model. Callers may pass any path; this is
# only a convenience default for tools. Left unopinionated (cwd-relative) so the
# library stays hardware-/project-agnostic.
DEFAULT_PATH = "pyhut_calibration.json"

# Schema tag written into every file so a future format change can be detected
# rather than silently mis-read.
SCHEMA = "pyhut.calibration/1"


# ---------------------------------------------------------------------------
# Fit result value objects (namedtuples: immutable, py3.6-safe)
# ---------------------------------------------------------------------------
# GyroFit.bias is (x, y, z) deg/s; std is the per-axis sample std-dev (deg/s);
# n is the sample count; still_ok is False when the spread suggests the robot was
# not actually still during the capture.
GyroFit = namedtuple("GyroFit", ["bias", "std", "n", "still_ok"])

# MagFit.hard_iron is the center (x, y, z) in uT to subtract; soft_iron is the
# per-axis scale (x, y, z), dimensionless, to multiply after centering; coverage
# is the fraction [0, 1] of the XY circle the samples swept; n is the sample
# count; axes_ok flags, per axis, whether the swept range was large enough to
# trust its scale (an under-swept axis -- e.g. Z in a flat in-place spin -- keeps
# scale 1.0).
MagFit = namedtuple("MagFit", ["hard_iron", "soft_iron", "coverage", "n", "axes_ok"])


# ---------------------------------------------------------------------------
# The calibration model
# ---------------------------------------------------------------------------
class Calibration(object):
    """The persisted correction parameters, plus the math that applies them.

    Three independent corrections, each defaulting to a no-op so an
    uncalibrated model is a faithful pass-through:

        gyro_bias       (x, y, z) deg/s  subtracted from the raw gyro
        mag_hard_iron   (x, y, z) uT     subtracted from the raw field (center)
        mag_soft_iron   (x, y, z) scale  multiplied after centering (sphere-fit)

    The corrected magnetometer field is ``(raw - hard) * soft``; the corrected
    gyro is ``raw - bias``. ``meta`` is a free-form dict of fit diagnostics
    (sample counts, coverage, timestamps) carried along for the record; it does
    not affect the math.
    """

    def __init__(self, gyro_bias=(0.0, 0.0, 0.0),
                 mag_hard_iron=(0.0, 0.0, 0.0),
                 mag_soft_iron=(1.0, 1.0, 1.0),
                 meta=None):
        self.gyro_bias = self._triple(gyro_bias)
        self.mag_hard_iron = self._triple(mag_hard_iron)
        self.mag_soft_iron = self._triple(mag_soft_iron)
        self.meta = dict(meta) if meta else {}

    # -- construction helpers ----------------------------------------------
    @staticmethod
    def _triple(v):
        # Coerce any 3-sequence to a tuple of three floats; clear error if not.
        t = tuple(float(x) for x in v)
        if len(t) != 3:
            raise ValueError("expected 3 values, got %d" % len(t))
        return t

    @classmethod
    def default(cls):
        """An identity calibration: every correction is a no-op."""
        return cls()

    def is_identity(self):
        """True when applying this model changes nothing (a fresh robot)."""
        return (self.gyro_bias == (0.0, 0.0, 0.0)
                and self.mag_hard_iron == (0.0, 0.0, 0.0)
                and self.mag_soft_iron == (1.0, 1.0, 1.0))

    # -- the corrections ----------------------------------------------------
    def apply_gyro(self, gyro):
        """Return the bias-corrected gyro tuple for a raw ``(gx, gy, gz)``."""
        b = self.gyro_bias
        return (gyro[0] - b[0], gyro[1] - b[1], gyro[2] - b[2])

    def apply_mag(self, field):
        """Return the hard/soft-iron-corrected field for a raw ``(mx, my, mz)``.

        ``(raw - hard_iron) * soft_iron``, component-wise.
        """
        h, s = self.mag_hard_iron, self.mag_soft_iron
        return ((field[0] - h[0]) * s[0],
                (field[1] - h[1]) * s[1],
                (field[2] - h[2]) * s[2])

    # -- JSON persistence ---------------------------------------------------
    def to_dict(self):
        """A plain dict ready for ``json.dump`` (lists, not tuples)."""
        return {
            "schema": SCHEMA,
            "gyro_bias": list(self.gyro_bias),
            "mag_hard_iron": list(self.mag_hard_iron),
            "mag_soft_iron": list(self.mag_soft_iron),
            "meta": self.meta,
        }

    @classmethod
    def from_dict(cls, d):
        """Build from a parsed dict. Missing fields fall back to identity, so a
        partial or older file still loads as a valid (if weaker) model. A wrong
        schema raises, since silently mis-reading numbers is worse than failing.
        """
        if not isinstance(d, dict):
            raise ValueError("calibration data must be a JSON object")
        schema = d.get("schema", SCHEMA)
        if schema != SCHEMA:
            raise ValueError(
                "unsupported calibration schema %r (expected %r)" % (schema, SCHEMA))
        return cls(
            gyro_bias=d.get("gyro_bias", (0.0, 0.0, 0.0)),
            mag_hard_iron=d.get("mag_hard_iron", (0.0, 0.0, 0.0)),
            mag_soft_iron=d.get("mag_soft_iron", (1.0, 1.0, 1.0)),
            meta=d.get("meta", None),
        )

    def save(self, path=DEFAULT_PATH):
        """Write the model to ``path`` as pretty-printed JSON."""
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=2, sort_keys=True)
            f.write("\n")
        return path

    @classmethod
    def load(cls, path=DEFAULT_PATH):
        """Load the model from ``path`` (strict).

        Raises ``FileNotFoundError`` if the file is absent and ``ValueError`` if
        it is present but unparseable / wrong-schema. For a startup path that
        must never fail on a fresh robot, use :meth:`load_or_default`.
        """
        with open(path, "r") as f:
            data = json.load(f)
        return cls.from_dict(data)

    @classmethod
    def load_or_default(cls, path=DEFAULT_PATH):
        """Load from ``path``, or return an identity model if that's not possible.

        This is the robust startup entry point: a missing file (fresh robot) or a
        corrupt one yields a working pass-through calibration instead of raising,
        so the robot always comes up. The reason, if any, is recorded under
        ``meta['load_error']`` for diagnostics.
        """
        try:
            return cls.load(path)
        except FileNotFoundError:
            return cls.default()
        except (ValueError, OSError) as exc:
            cal = cls.default()
            cal.meta["load_error"] = str(exc)
            return cal

    def __repr__(self):
        return ("Calibration(gyro_bias=%r, mag_hard_iron=%r, mag_soft_iron=%r)"
                % (self.gyro_bias, self.mag_hard_iron, self.mag_soft_iron))


# ---------------------------------------------------------------------------
# Sample collection (prefers the Sampler; falls back to a direct sensor)
# ---------------------------------------------------------------------------
def _is_sampler(obj):
    return hasattr(obj, "subscribe") and hasattr(obj, "latest")


def _collect_from_sampler(sampler, field, duration_s, on_sample):
    """Collect ``sample.<field>`` values from a running Sampler for a duration.

    Subscribes to the stream (so every emitted Sample is seen, not just the
    latest), gathers the chosen field, then unsubscribes. Values that are None
    (a sensor not fitted, or a glitched read the sampler couldn't fill) are
    skipped. Raises if the window produced nothing -- the usual cause is a
    sampler that was never started.
    """
    out = []
    lock = threading.Lock()

    def _cb(sample):
        v = getattr(sample, field)
        if v is None:
            return
        with lock:
            out.append(v)
        if on_sample is not None:
            on_sample(v)

    unsubscribe = sampler.subscribe(_cb)
    try:
        time.sleep(duration_s)
    finally:
        unsubscribe()
        # Let any in-flight callback finish before we read the list back.
        time.sleep(0.01)
    with lock:
        collected = list(out)
    if not collected:
        raise RuntimeError(
            "sampler produced no %s samples in %.2fs -- is it started?"
            % (field, duration_s))
    return collected


def _collect_from_sensor(sensor, attr, duration_s, rate_hz, on_sample):
    """Poll a sensor's ``read()`` directly for a duration (no sampler).

    ``attr`` selects the tuple on the reading (``"gyro"`` for an IMU, ``"field"``
    for a Magnetometer). Only use this when NO sampler owns the sensor; while a
    sampler runs it is the sole reader and you must collect from its stream.
    """
    out = []
    period = 1.0 / rate_hz if rate_hz > 0 else 0.0
    end = time.monotonic() + duration_s
    while time.monotonic() < end:
        try:
            v = getattr(sensor.read(), attr)
        except OSError:
            v = None  # transient bus hiccup; skip this tick
        if v is not None:
            out.append(v)
            if on_sample is not None:
                on_sample(v)
        if period:
            time.sleep(period)
    if not out:
        raise RuntimeError("sensor produced no samples in %.2fs" % duration_s)
    return out


# ---------------------------------------------------------------------------
# Gyroscope: zero-rate bias
# ---------------------------------------------------------------------------
def fit_gyro_bias(gyro_samples, max_std=1.0):
    """Fit the gyro zero-rate offset from samples taken while the robot is still.

    The bias is simply the per-axis mean: a stationary gyro should read zero, so
    whatever it averages to is the offset to subtract. ``max_std`` (deg/s) is the
    stillness gate -- if any axis's sample std-dev exceeds it, the robot probably
    moved and ``still_ok`` comes back False so the caller can retry rather than
    bake motion into the bias.

    Parameters
    ----------
    gyro_samples : iterable of (gx, gy, gz)
        Raw gyro readings in deg/s.
    max_std : float
        Per-axis std-dev (deg/s) above which the capture is judged not-still.

    Returns
    -------
    GyroFit
    """
    samples = list(gyro_samples)
    n = len(samples)
    if n == 0:
        raise ValueError("no gyro samples to fit")
    xs = [s[0] for s in samples]
    ys = [s[1] for s in samples]
    zs = [s[2] for s in samples]
    bias = (statistics.mean(xs), statistics.mean(ys), statistics.mean(zs))
    if n >= 2:
        std = (statistics.pstdev(xs), statistics.pstdev(ys), statistics.pstdev(zs))
    else:
        std = (0.0, 0.0, 0.0)
    still_ok = all(s <= max_std for s in std)
    return GyroFit(bias=bias, std=std, n=n, still_ok=still_ok)


def calibrate_gyro_bias(source, duration_s=2.0, rate_hz=50.0,
                        max_std=1.0, on_sample=None):
    """Collect gyro samples from ``source`` while still, then fit the bias.

    ``source`` may be a running :class:`~pyhut.interfaces.Sampler` (preferred --
    the gyro is read from its Sample stream so the sampler stays the sole sensor
    owner) or a bare :class:`~pyhut.interfaces.IMU` (polled directly; only do
    this when no sampler is running). Keep the robot motionless for the whole
    ``duration_s``.

    Returns a :class:`GyroFit`. Build a :class:`Calibration` from it with
    :func:`update_calibration` or by hand.
    """
    if _is_sampler(source):
        samples = _collect_from_sampler(source, "gyro", duration_s, on_sample)
    else:
        samples = _collect_from_sensor(source, "gyro", duration_s, rate_hz, on_sample)
    return fit_gyro_bias(samples, max_std=max_std)


# ---------------------------------------------------------------------------
# Magnetometer: hard-iron center + soft-iron (diagonal) scale
# ---------------------------------------------------------------------------
def _xy_coverage(samples, center, sectors=36):
    """Fraction of the XY circle (around ``center``) the samples swept.

    Bins each sample's heading into ``sectors`` equal wedges and returns
    filled/total. 1.0 means a clean full turn; a partial rotation reads low.
    """
    seen = set()
    cx, cy = center[0], center[1]
    for s in samples:
        dx, dy = s[0] - cx, s[1] - cy
        if dx == 0.0 and dy == 0.0:
            continue
        ang = math.atan2(dy, dx)  # [-pi, pi]
        idx = int((ang + math.pi) / (2.0 * math.pi) * sectors) % sectors
        seen.add(idx)
    return len(seen) / float(sectors)


def fit_mag_iron(mag_samples, min_range_frac=0.3, max_scale=2.5):
    """Fit hard-iron center and soft-iron (diagonal) scale from a rotation sweep.

    Method (the standard spinning min/max calibration, std-lib only):

      * **Hard-iron / center** -- per axis, the midpoint of the observed
        extremes, ``(max + min) / 2``. Subtracting it re-centres the field
        ellipsoid on the origin. The midpoint (not the mean) is used so uneven
        angular sampling doesn't pull the centre. (Min/max keys off extremes, so
        it is sensitive to tilt and outliers during a hand spin; for a
        tilt-robust centre, an ellipsoid least-squares fit is the upgrade.)
      * **Soft-iron / scale** -- per axis, the semi-range ``(max - min) / 2`` is
        that axis's radius. Scaling each axis by ``avg_radius / radius`` stretches
        the ellipsoid back to a sphere, which is what makes the heading honest.

    Conditioning guards (both default an axis to scale 1.0 and ``axes_ok`` False,
    so a poorly observed axis is left uncorrected rather than wildly mis-scaled):

      * ``min_range_frac`` -- an axis whose semi-range is below this fraction of
        the largest axis's is treated as under-swept and excluded from the target
        radius. This is the Z axis in a flat in-place spin (the Duckiebot's case):
        rotating about vertical sweeps X/Y through a full circle but barely moves
        Z. Heading uses X/Y, which a spin does constrain, so this is correct
        behaviour, not a failure.
      * ``max_scale`` -- even a "well-swept" axis whose implied scale falls
        outside ``[1/max_scale, max_scale]`` is too far from the others to trust,
        so it too is pinned. This catches an axis that passes the range test but
        is still ill-conditioned.

    This fits only the diagonal soft-iron terms; see the module docstring.

    Parameters
    ----------
    mag_samples : iterable of (mx, my, mz)
        Raw field samples in uT, taken through (ideally) a full rotation.
    min_range_frac : float
        Under-sweep threshold, as a fraction of the largest axis's semi-range.
    max_scale : float
        Largest correction factor any axis may receive; beyond it the axis is
        pinned to 1.0 as ill-conditioned.

    Returns
    -------
    MagFit
    """
    samples = list(mag_samples)
    n = len(samples)
    if n < 2:
        raise ValueError("need at least 2 mag samples to fit, got %d" % n)

    mins = [min(s[i] for s in samples) for i in range(3)]
    maxs = [max(s[i] for s in samples) for i in range(3)]
    center = tuple((maxs[i] + mins[i]) / 2.0 for i in range(3))
    semi = [(maxs[i] - mins[i]) / 2.0 for i in range(3)]

    max_semi = max(semi)
    if max_semi <= 0.0:
        raise ValueError("degenerate mag samples: zero field range on every axis")

    # Average radius over the well-swept axes only, so an under-swept axis can't
    # drag the target sphere radius down (which would mis-scale the good axes too).
    well_swept = [semi[i] >= min_range_frac * max_semi and semi[i] > 0.0
                  for i in range(3)]
    good = [semi[i] for i in range(3) if well_swept[i]]
    avg_radius = statistics.mean(good)

    scale = []
    axes_ok = []
    for i in range(3):
        s = avg_radius / semi[i] if well_swept[i] else None
        if s is not None and (1.0 / max_scale) <= s <= max_scale:
            scale.append(s)
            axes_ok.append(True)
        else:
            scale.append(1.0)       # under-swept or ill-conditioned: no scaling
            axes_ok.append(False)

    coverage = _xy_coverage(samples, center)
    return MagFit(hard_iron=center, soft_iron=tuple(scale),
                  coverage=coverage, n=n, axes_ok=tuple(axes_ok))


def calibrate_magnetometer(source, duration_s=20.0, rate_hz=50.0,
                           min_range_frac=0.2, on_sample=None):
    """Collect field samples from ``source`` while the robot turns, then fit.

    Rotate the robot through at least one full, slow revolution during the
    ``duration_s`` window -- by hand, or via a motion primitive driven by the
    caller. (Driving the motors is the caller's job; this routine only reads, and
    on the real HUT the motors sit next to the magnetometer, so prefer a hand
    spin or characterise motor EMI first.)

    ``source`` may be a running :class:`~pyhut.interfaces.Sampler` (preferred) or
    a bare :class:`~pyhut.interfaces.Magnetometer`. ``on_sample`` is called with
    each raw field tuple as it arrives -- handy for a live coverage readout that
    tells the operator when a full turn is done.

    Returns a :class:`MagFit`; check ``coverage`` before trusting it.
    """
    if _is_sampler(source):
        samples = _collect_from_sampler(source, "mag", duration_s, on_sample)
    else:
        samples = _collect_from_sensor(source, "field", duration_s, rate_hz, on_sample)
    return fit_mag_iron(samples, min_range_frac=min_range_frac)


# ---------------------------------------------------------------------------
# Folding fits into a Calibration
# ---------------------------------------------------------------------------
def update_calibration(base=None, gyro=None, mag=None):
    """Return a new :class:`Calibration` combining ``base`` with fresh fits.

    ``base`` is the model to start from (default: identity). ``gyro`` is a
    :class:`GyroFit` (or a raw 3-tuple) and ``mag`` is a :class:`MagFit` -- each,
    if given, overrides the matching part of the model. Diagnostics from the fits
    are recorded under ``meta`` for the record. The two sensors are independent,
    so you can refit one without disturbing the other.
    """
    base = base if base is not None else Calibration.default()
    gyro_bias = base.gyro_bias
    hard = base.mag_hard_iron
    soft = base.mag_soft_iron
    meta = dict(base.meta)

    if gyro is not None:
        if isinstance(gyro, GyroFit):
            gyro_bias = gyro.bias
            meta["gyro"] = {"std": list(gyro.std), "n": gyro.n,
                            "still_ok": gyro.still_ok, "t": time.time()}
        else:
            gyro_bias = gyro

    if mag is not None:
        if isinstance(mag, MagFit):
            hard = mag.hard_iron
            soft = mag.soft_iron
            meta["mag"] = {"coverage": mag.coverage, "n": mag.n,
                           "axes_ok": list(mag.axes_ok), "t": time.time()}
        else:
            raise TypeError("mag must be a MagFit")

    return Calibration(gyro_bias=gyro_bias, mag_hard_iron=hard,
                       mag_soft_iron=soft, meta=meta)


# ---------------------------------------------------------------------------
# The opt-in wrappers: apply a Calibration behind the sensor interfaces
# ---------------------------------------------------------------------------
class CalibratedIMU(IMU):
    """An :class:`~pyhut.interfaces.IMU` that bias-corrects the gyro on read.

    Wraps any inner IMU and a :class:`Calibration`; ``read()`` returns the inner
    reading with the gyro de-biased (accel and temperature pass through untouched
    -- accel calibration is out of scope here). Because it *is* an IMU, anything
    that takes an IMU -- the ThreadedSampler included -- accepts it unchanged, and
    an identity calibration makes it a transparent pass-through. The raw driver is
    never modified; you opt in purely by wrapping.
    """

    def __init__(self, imu, calibration):
        self._imu = imu
        self._cal = calibration

    @property
    def calibration(self):
        return self._cal

    def read(self):
        r = self._imu.read()
        return IMUReading(r.accel, self._cal.apply_gyro(r.gyro), r.temp_c)

    def close(self):
        self._imu.close()


class CalibratedMagnetometer(Magnetometer):
    """A :class:`~pyhut.interfaces.Magnetometer` with hard/soft-iron correction.

    Wraps any inner Magnetometer and a :class:`Calibration`; ``read()`` returns
    the inner field re-centred and sphere-scaled. ``heading_deg()`` is inherited
    from the interface and reads through this corrected ``read()``, so the compass
    is automatically de-skewed -- no override needed. Identity calibration is a
    transparent pass-through, and the raw driver is untouched.
    """

    def __init__(self, magnetometer, calibration):
        self._mag = magnetometer
        self._cal = calibration

    @property
    def calibration(self):
        return self._cal

    def read(self):
        r = self._mag.read()
        return MagReading(self._cal.apply_mag(r.field))

    def close(self):
        self._mag.close()