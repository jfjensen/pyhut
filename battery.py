"""
Duckiebattery driver: state-of-charge telemetry and software power-off over USB.

The Duckiebattery on a DB21 (DB21M / DB21J / DBR4) is a smart pack with its own
microcontroller that reports over a USB CDC-ACM serial link and can cut its own
output on command. That link is the same one the Duckietown stack uses; this
module talks to it directly, with no Duckietown software and no pyserial -- just
a raw tty configured through termios, in the same bare-metal spirit as the I2C
and GPIO in lowlevel.py.

Two things this gets you that the buttons alone do not:

  * The charge level in software. The pack's five LEDs are usually buried in the
    chassis, so reading SOC over USB is the only way to see it once the robot is
    assembled. Point ``latest()`` at it and print the percentage on the OLED.
  * A clean software power-off. ``turn_off()`` tells the pack to drop its output
    after a short delay; you then halt the Jetson. When the rail goes away the
    pack returns to its OFF/idle state, which is what makes the side button work
    as a power-on again next time. Without it, a ``sudo poweroff`` leaves the
    output live, the pack never resets, and the side button appears dead.

Design, mirroring DuckiebotDrivetrain and ThreadedSampler:

  * One background thread is the SOLE owner of the serial port. The pack streams
    data frames on its own cadence; the worker reads them, publishes the latest
    reading, and is also the only writer -- so a ``??`` info request or a ``Q``
    shutdown command never collides with the read side. Consumers get data
    through ``latest()`` / ``subscribe()``, never by touching the port.
  * The pack is found by USB VID:PID, not by a hard-coded ``/dev/ttyACM0``, since
    the node number is whatever the kernel assigned. In its normal ("ready") mode
    it enumerates as 04d8:ecb7. During a firmware flash it switches to a SAM-BA
    bootloader at 16d0:0557, so if it ever "disappears" from here, that is why.

Std-lib only and Python 3.6-compatible (no dataclasses, no walrus, no pyserial).
The wire format is parsed to match the behaviour of Duckietown's own driver; the
one spot to adjust if a firmware revision frames things differently is
``_parse_frame`` (and pass ``on_raw=`` to watch the actual lines go by).
"""


import glob
import json
import os
import re
import select
import termios
import threading
import time
from collections import deque, namedtuple
from typing import Callable, Deque, Dict, List, Optional


# ---------------------------------------------------------------------------
# USB identity and link parameters (from Duckietown's battery_drivers)
# ---------------------------------------------------------------------------
BATTERY_VID = "04d8"          # normal "ready" mode
BATTERY_PID = "ecb7"
BATTERY_BOOT_VID = "16d0"     # SAM-BA bootloader, only while flashing firmware
BATTERY_BOOT_PID = "0557"
BATTERY_BAUD = 9600


# ---------------------------------------------------------------------------
# Reading value object (namedtuple: immutable, attribute access, py3.6-safe)
# ---------------------------------------------------------------------------
# Units are already converted from the pack's raw mV / mA / Kelvin / minutes:
#
#   t                  float  time.monotonic() when this reading was parsed
#   present            bool   True for a real reading (always True here)
#   charging           bool   True when charger voltage is present on the input
#   percentage         int    state of charge, 0..100
#   cell_voltage       float  battery cell voltage, volts
#   input_voltage      float  charger/input voltage, volts (>0 means plugged in)
#   current            float  pack current, amps (sign per firmware convention)
#   temperature        float  cell temperature, degrees C
#   usb_out_1_voltage  float  USB OUT-1 ("muscles") rail, volts
#   usb_out_2_voltage  float  USB OUT-2 ("brain") rail, volts
#   cycle_count        int    charge cycles the pack has logged
#   time_to_empty_s    int    pack's own runtime estimate, seconds
#
# Every field except `t` and the two bools defaults to None, so a partial frame
# still produces a valid reading instead of throwing.
BatteryReading = namedtuple("BatteryReading", [
    "t", "present", "charging", "percentage",
    "cell_voltage", "input_voltage", "current", "temperature",
    "usb_out_1_voltage", "usb_out_2_voltage",
    "cycle_count", "time_to_empty_s",
])
# py3.6: defaults via __new__, since namedtuple(defaults=...) is 3.7+. `t`,
# `present` and `charging` are always supplied; the remaining nine default None.
BatteryReading.__new__.__defaults__ = (None,) * 9


# ---------------------------------------------------------------------------
# Bare-metal serial line: a raw tty at a fixed baud, no pyserial
# ---------------------------------------------------------------------------
class _SerialLine:
    """One open CDC-ACM tty, configured raw at `baud`, read line-by-line.

    This is the serial equivalent of lowlevel.I2CDevice: it owns one fd and does
    its own framing. termios puts the port in raw 8N1 with no flow control;
    reads wait on select() so a silent pack times out instead of blocking the
    worker forever.
    """

    def __init__(self, path: str, baud: int = BATTERY_BAUD, read_timeout: float = 2.0):
        self.path = path
        self._timeout = read_timeout
        self._buf = b""
        # O_NONBLOCK so open() never stalls waiting on carrier; we gate every
        # read on select() anyway.
        self._fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            self._configure(baud)
        except Exception:
            os.close(self._fd)
            self._fd = None
            raise

    def _configure(self, baud: int) -> None:
        # attrs = [iflag, oflag, cflag, lflag, ispeed, ospeed, cc]
        iflag, oflag, cflag, lflag, ispeed, ospeed, cc = termios.tcgetattr(self._fd)
        speed = getattr(termios, "B%d" % baud)
        # raw mode, by hand (Python's termios has no cfmakeraw)
        iflag &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK | termios.ISTRIP
                   | termios.INLCR | termios.IGNCR | termios.ICRNL | termios.IXON)
        iflag |= termios.IGNPAR
        oflag &= ~termios.OPOST
        lflag &= ~(termios.ECHO | termios.ECHONL | termios.ICANON
                   | termios.ISIG | termios.IEXTEN)
        cflag &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE
                   | getattr(termios, "CRTSCTS", 0))
        cflag |= termios.CS8 | termios.CLOCAL | termios.CREAD
        cc = list(cc)
        cc[termios.VMIN] = 0
        cc[termios.VTIME] = 0
        termios.tcsetattr(
            self._fd, termios.TCSANOW,
            [iflag, oflag, cflag, lflag, speed, speed, cc],
        )
        termios.tcflush(self._fd, termios.TCIOFLUSH)

    def write(self, data: bytes) -> None:
        os.write(self._fd, bytes(data))

    def readline(self, timeout: Optional[float] = None) -> Optional[bytes]:
        """Return one newline-terminated line (without the newline), or None on
        timeout. Bytes are buffered across calls so a split frame still lands."""
        deadline = time.monotonic() + (self._timeout if timeout is None else timeout)
        while b"\n" not in self._buf:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            r, _, _ = select.select([self._fd], [], [], remaining)
            if not r:
                return None
            try:
                chunk = os.read(self._fd, 256)
            except OSError:
                # EAGAIN despite select, or a transient USB hiccup
                continue
            if chunk:
                self._buf += chunk
        line, _, self._buf = self._buf.partition(b"\n")
        return line

    def close(self) -> None:
        if self._fd is not None:
            try:
                os.close(self._fd)
            finally:
                self._fd = None


def find_battery_ports(vid: str = BATTERY_VID, pid: str = BATTERY_PID) -> List[str]:
    """Return the /dev/ttyACM* nodes whose USB device matches `vid`:`pid`.

    Walks sysfs from each tty up to its USB device directory and reads
    idVendor/idProduct, so it does not depend on pyserial. Returns [] if the
    pack is not present (off, in protection mode, or data line not connected).
    """
    matches = []  # type: List[str]
    vid, pid = vid.lower(), pid.lower()
    for dev in sorted(glob.glob("/dev/ttyACM*")):
        link = "/sys/class/tty/%s/device" % os.path.basename(dev)
        try:
            node = os.path.realpath(link)
        except OSError:
            continue
        # The tty's device is a USB interface; idVendor/idProduct live a level
        # or two up on the parent USB device. Walk up until we find them.
        for _ in range(6):
            v = os.path.join(node, "idVendor")
            p = os.path.join(node, "idProduct")
            if os.path.isfile(v) and os.path.isfile(p):
                try:
                    with open(v) as fv, open(p) as fp:
                        if fv.read().strip().lower() == vid and fp.read().strip().lower() == pid:
                            matches.append(dev)
                except OSError:
                    pass
                break
            parent = os.path.dirname(node)
            if parent == node:
                break
            node = parent
    return matches


# ---------------------------------------------------------------------------
# Frame parsing
# ---------------------------------------------------------------------------
def _parse_frame(raw: bytes) -> Optional[Dict]:
    """Turn one raw serial line into a dict, or None if it is not a frame.

    The pack emits brace-delimited frames. This mirrors the cleanup Duckietown's
    driver does: strip NULs, glue negative numbers back to their sign, and if two
    frames arrived concatenated ("}{"), keep the first. JSON is tried first; if
    the firmware uses unquoted keys (they contain spaces, parens and "%"), a
    tolerant flow-mapping parse picks up the slack.
    """
    try:
        s = raw.decode("utf-8", "ignore")
    except Exception:
        return None
    s = re.sub(r"\x00\s*", "", s)      # drop stray NULs and the space after
    s = re.sub(r"-\s+", "-", s)        # "- 5" -> "-5"
    s = s.strip()
    if "}{" in s:
        s = s.split("}{")[0] + "}"
    if not (s.startswith("{") and s.endswith("}")):
        return None
    try:
        return json.loads(s)
    except ValueError:
        return _loose_mapping(s)


def _loose_mapping(s: str) -> Optional[Dict]:
    """Parse `{ key: value, key: value }` with unquoted, unit-bearing keys."""
    inner = s[1:-1].strip()
    if not inner:
        return {}
    out = {}  # type: Dict
    for part in inner.split(","):
        if ":" not in part:
            continue
        key, _, val = part.partition(":")
        key = key.strip().strip('"').strip("'")
        if key:
            out[key] = _coerce(val.strip().strip('"').strip("'"))
    return out or None


def _coerce(v: str):
    try:
        return int(v)
    except ValueError:
        pass
    try:
        return float(v)
    except ValueError:
        return v


def _semver_ge(a: str, b: str) -> bool:
    """True if version string `a` >= `b`, comparing dotted integer parts."""
    def parts(x):
        out = []
        for tok in str(x).split("."):
            try:
                out.append(int(tok))
            except ValueError:
                out.append(0)
        return out
    pa, pb = parts(a), parts(b)
    n = max(len(pa), len(pb))
    pa += [0] * (n - len(pa))
    pb += [0] * (n - len(pb))
    return pa >= pb


# ---------------------------------------------------------------------------
# One pending write-and-wait exchange with the pack (info request, turn-off)
# ---------------------------------------------------------------------------
class _Interaction:
    def __init__(self, name: str, command: bytes, check: Callable[[Dict], bool]):
        self.name = name
        self.command = command
        self.check = check
        self.sent = False
        self.result = None        # type: Optional[Dict]
        self.done = threading.Event()


# ---------------------------------------------------------------------------
# The driver
# ---------------------------------------------------------------------------
class Duckiebattery:
    """Background reader/controller for a Duckiebattery on USB.

    Lifecycle is explicit (``start()`` / ``stop()``), idempotent, and works as a
    context manager, exactly like ThreadedSampler. While running, the worker
    thread owns the serial port; read state through ``latest()`` and the
    convenience properties, and shut the pack down through ``turn_off()``.

    Parameters
    ----------
    port
        A specific ``/dev/ttyACM*`` to use. Default None auto-discovers by
        USB VID:PID, which is the right choice in almost all cases.
    on_raw
        Optional ``callback(str)`` invoked with every cleaned serial line before
        parsing. Handy for seeing the actual wire format if a firmware revision
        frames things differently than this driver expects.
    logger
        Optional logging.Logger. If None, the driver stays quiet.
    reopen_interval
        Seconds to wait before re-scanning for the pack after it goes away
        (turned off, entered protection mode, or cable pulled).
    """

    def __init__(
        self,
        port=None,                # type: Optional[str]
        on_raw=None,              # type: Optional[Callable[[str], None]]
        logger=None,              # type: Optional[object]
        vid=BATTERY_VID,          # type: str
        pid=BATTERY_PID,          # type: str
        reopen_interval=5.0,      # type: float
    ):
        self._fixed_port = port
        self._on_raw = on_raw
        self._log = logger
        self._vid = vid
        self._pid = pid
        self._reopen = reopen_interval

        # State shared with consumers, guarded by _lock.
        self._lock = threading.Lock()
        self._latest = None       # type: Optional[BatteryReading]
        self._info = None         # type: Optional[Dict]
        self._last_ack = None     # type: Optional[Dict]

        # Subscriber registry, its own lock so notifying never holds _lock.
        self._sub_lock = threading.Lock()
        self._subscribers = []    # type: List[Callable[[BatteryReading], None]]

        # Pending interactions, drained one at a time by the worker (sole writer).
        self._pending = deque()   # type: Deque[_Interaction]
        self._pending_lock = threading.Lock()
        self._info_requested = False

        self._stop_evt = threading.Event()
        self._thread = None       # type: Optional[threading.Thread]
        self._running = False
        self._start_lock = threading.Lock()

    # -- lifecycle ----------------------------------------------------------
    def start(self) -> "Duckiebattery":
        with self._start_lock:
            if self._running:
                return self
            self._stop_evt.clear()
            self._thread = threading.Thread(
                target=self._work, name="duckiebattery", daemon=True)
            self._thread.start()
            self._running = True
            return self

    def stop(self) -> None:
        with self._start_lock:
            if not self._running:
                return
            self._stop_evt.set()
            if self._thread is not None:
                self._thread.join(timeout=2.0)
            self._thread = None
            self._running = False

    def close(self) -> None:
        self.stop()

    def __enter__(self) -> "Duckiebattery":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.close()

    # -- consumer API -------------------------------------------------------
    def latest(self) -> Optional[BatteryReading]:
        with self._lock:
            return self._latest

    @property
    def info(self) -> Optional[Dict]:
        """Pack identity once known: ``{"version", "serial_number", "boot"}``."""
        with self._lock:
            return self._info

    @property
    def firmware_version(self) -> Optional[str]:
        info = self.info
        return info.get("version") if info else None

    @property
    def serial_number(self) -> Optional[str]:
        info = self.info
        return info.get("serial_number") if info else None

    @property
    def percentage(self) -> Optional[int]:
        r = self.latest()
        return r.percentage if r else None

    @property
    def charging(self) -> Optional[bool]:
        r = self.latest()
        return r.charging if r else None

    @property
    def current(self) -> Optional[float]:
        r = self.latest()
        return r.current if r else None

    def is_full(self, current_eps: float = 0.05) -> Optional[bool]:
        """True when the pack reads 100% and is no longer pulling charge current.

        SOC alone sits at 100% for a while before the cell is actually topped
        off, so when a reading is available this also waits for the charge
        current to taper toward zero. Returns None before the first reading.
        """
        r = self.latest()
        if r is None or r.percentage is None:
            return None
        if r.percentage < 100:
            return False
        if r.current is None:
            return True
        return abs(r.current) <= current_eps

    def subscribe(self, callback: Callable[[BatteryReading], None]) -> Callable[[], None]:
        """Register ``callback(reading)`` for every new reading; returns an
        unsubscribe callable. The callback runs on the worker thread, so keep it
        cheap and non-blocking."""
        with self._sub_lock:
            self._subscribers.append(callback)

        def _unsubscribe():
            with self._sub_lock:
                try:
                    self._subscribers.remove(callback)
                except ValueError:
                    pass

        return _unsubscribe

    # -- control: software power-off ---------------------------------------
    def turn_off(self, timeout: int = 20, wait: bool = True, wait_timeout: float = 6.0,
                 force_timed: bool = False) -> bool:
        """Tell the pack to cut its output, then return.

        This does NOT halt the Jetson; it arms the pack. The intended sequence is
        ``turn_off()`` then ``os.system("sudo poweroff")``: the OS halts inside
        the countdown, the pack drops the rail when the timer fires, and the pack
        is left OFF so the side button powers it on again next time.

        Which command goes over the wire depends on the pack's firmware:

          * 2.0.0 takes ``Q`` plus a two-digit second count, so ``timeout`` is
            honoured directly and the pack acknowledges with ``TTL(sec)``.
          * 2.0.1 and newer take a bare ``QQ`` with NO duration. On those, the
            pack uses its own built-in delay and ``timeout`` is ignored. That is
            usually why a shutdown feels faster than the 20 you asked for.

        ``force_timed=True`` sends the timed ``Q{nn}`` form even on 2.0.1+, so you
        can test whether your firmware still honours a chosen delay. It might, in
        which case you get a controllable countdown back. It might ignore the
        command entirely, in which case no acknowledgement arrives and this
        returns False, so check the return value before you call ``poweroff``.

        After a successful call, ``last_ack`` holds the raw acknowledgement frame
        and ``last_ttl_sec`` the delay the pack actually scheduled, if it reported
        one. Returns True if the pack acknowledged (when ``wait``), False on
        timeout or if no pack is present.
        """
        if not self._running:
            raise RuntimeError("Duckiebattery is not started")

        version = self.firmware_version
        if version is None:
            # We need the firmware version to pick the command; ask for it.
            self.request_info(wait=True, wait_timeout=wait_timeout)
            version = self.firmware_version

        timed = force_timed or version is None or not _semver_ge(version, "2.0.1")
        if timed:
            secs = max(1, min(99, int(timeout)))
            command = ("Q%s" % str(secs).zfill(2)).encode("ascii")
            # Accept any TTL(sec) ack, so we can SEE what the pack scheduled even
            # if it clamps the value to something other than what we asked for.
            check = lambda d: d.get("TTL(sec)") is not None
        else:
            command = b"QQ"
            # 2.0.1+ acknowledges with QACK; some firmware also echoes a TTL.
            check = lambda d: ("QACK" in d) or (d.get("TTL(sec)") is not None)

        with self._lock:
            self._last_ack = None
        it = self._enqueue(_Interaction("turn_off", command, check))
        if not wait:
            return True
        ok = it.done.wait(wait_timeout)
        if ok:
            with self._lock:
                self._last_ack = it.result
        return ok

    @property
    def last_ack(self) -> Optional[Dict]:
        """The raw frame the pack returned to the last ``turn_off()``, or None.

        Look for a ``TTL(sec)`` key to see the real scheduled cutoff delay.
        """
        with self._lock:
            return self._last_ack

    @property
    def last_ttl_sec(self) -> Optional[int]:
        """The cutoff delay the pack acknowledged last (seconds), or None if it
        did not report one (a bare ``QACK`` carries no duration)."""
        ack = self.last_ack
        if not ack:
            return None
        ttl = ack.get("TTL(sec)")
        return int(ttl) if ttl is not None else None

    def request_info(self, wait: bool = True, wait_timeout: float = 6.0) -> Optional[Dict]:
        """Ask the pack for its identity (firmware version, serial, boot data).

        The worker requests this once on connect anyway; call this only if you
        want to block until it is known.
        """
        it = self._enqueue(_Interaction(
            "get_info", b"??", lambda d: "FirmwareVersion" in d))
        if wait:
            it.done.wait(wait_timeout)
        return self.info

    # -- worker -------------------------------------------------------------
    def _enqueue(self, interaction: "_Interaction") -> "_Interaction":
        with self._pending_lock:
            self._pending.append(interaction)
        return interaction

    def _work(self) -> None:
        while not self._stop_evt.is_set():
            ports = [self._fixed_port] if self._fixed_port else find_battery_ports(self._vid, self._pid)
            if not ports:
                self._warn("No Duckiebattery found on USB. Retrying.")
                self._stop_evt.wait(self._reopen)
                continue

            for port in ports:
                if self._stop_evt.is_set():
                    return
                try:
                    line = _SerialLine(port, BATTERY_BAUD)
                except OSError as e:
                    self._warn("Could not open %s: %s" % (port, e))
                    continue
                try:
                    self._read_loop(line)
                finally:
                    line.close()

            # Lost the pack (or every candidate failed); pause before re-scanning.
            self._stop_evt.wait(self._reopen)

    def _read_loop(self, line: "_SerialLine") -> None:
        seen_data = False
        self._info_requested = False
        active = None  # type: Optional[_Interaction]

        while not self._stop_evt.is_set():
            # Promote a queued interaction once data is flowing (the pack ignores
            # commands sent before it has started streaming).
            if active is None and seen_data:
                with self._pending_lock:
                    active = self._pending.popleft() if self._pending else None
            if active is not None and not active.sent:
                try:
                    line.write(active.command)
                    active.sent = True
                except OSError:
                    return  # port died; let _work reopen

            raw = line.readline()
            if raw is None:
                # No data within the read timeout. If we never saw any, the node
                # exists but is not the pack (or it is silent); bail and re-scan.
                if not seen_data:
                    return
                continue

            if self._on_raw is not None:
                try:
                    self._on_raw(raw.decode("utf-8", "ignore").strip())
                except Exception:
                    pass

            frame = _parse_frame(raw)
            if not frame:
                continue

            seen_data = seen_data or ("SOC(%)" in frame)

            if "SOC(%)" in frame:
                self._publish(self._to_reading(frame))
            elif "FirmwareVersion" in frame:
                with self._lock:
                    self._info = _format_info(frame)

            # A frame of any kind may be the acknowledgement we are waiting on.
            if active is not None and active.sent and active.check(frame):
                active.result = frame
                active.done.set()
                active = None

            # Once we have seen data but never asked who we are, do it once.
            if seen_data and not self._info_requested and self.info is None:
                self._info_requested = True
                self._enqueue(_Interaction(
                    "get_info", b"??", lambda d: "FirmwareVersion" in d))

    # -- helpers ------------------------------------------------------------
    def _publish(self, reading: "BatteryReading") -> None:
        with self._lock:
            self._latest = reading
        with self._sub_lock:
            subscribers = list(self._subscribers)
        for cb in subscribers:
            try:
                cb(reading)
            except Exception:
                pass  # a bad consumer must not kill the worker

    @staticmethod
    def _to_reading(d: Dict) -> "BatteryReading":
        chg_mv = d.get("ChargerVoltage(mV)")
        input_v = (float(chg_mv) / 1000.0) if chg_mv is not None else None

        def volts(key):
            mv = d.get(key)
            return round(float(mv) / 1000.0, 2) if mv is not None else None

        cur_ma = d.get("Current(mA)")
        soc = d.get("SOC(%)")
        temp_k = d.get("CellTemp(degK)")
        cyc = d.get("CycleCount")
        tte_min = d.get("TimeToEmpty(min)")

        return BatteryReading(
            time.monotonic(),
            True,
            input_v is not None and input_v >= (5.0 / 2),
            int(soc) if soc is not None else None,
            volts("CellVoltage(mV)"),
            round(input_v, 2) if input_v is not None else None,
            round(float(cur_ma) / 1000.0, 2) if cur_ma is not None else None,
            round(float(temp_k) - 273.15, 2) if temp_k is not None else None,
            volts("USB OUT-1(mV)"),
            volts("USB OUT-2(mV)"),
            int(cyc) if cyc is not None else None,
            int(float(tte_min) * 60) if tte_min is not None else None,
        )

    def _warn(self, msg: str) -> None:
        if self._log is not None:
            try:
                self._log.warning(msg)
            except Exception:
                pass


def _format_info(frame: Dict) -> Dict:
    """Shape an info frame into ``{"version", "serial_number", "boot"}``.

    FirmwareVersion arrives as three digit characters (e.g. "201"); the pack's
    own driver maps those to a dotted version, so "201" -> "2.0.1".
    """
    fw = (str(frame.get("FirmwareVersion", "")) + "000")
    version = "%s.%s.%s" % (fw[0], fw[1], fw[2])
    boot = str(frame.get("BootData", ""))
    boot_info = {}  # type: Dict
    if len(boot) >= 9:
        yy, mm, dd = boot[3:5], boot[5:7], boot[7:9]
        boot_info = {
            "version": boot[0],
            "pcb_version": boot[1:3],
            "date": "%s/%s/%s" % (mm, dd, yy),
        }
    return {
        "version": version,
        "serial_number": frame.get("SerialNumber"),
        "boot": boot_info,
    }


# ---------------------------------------------------------------------------
# CLI: watch the pack, or test a software power-off
# ---------------------------------------------------------------------------
def main() -> None:
    """Run with ``sudo python3 -m pyhut.battery`` (needs access to /dev/ttyACM*).

    With no arguments it prints live readings. ``--shutdown [SECONDS]`` arms the
    pack's output cutoff, prints the acknowledgement, and exits WITHOUT halting
    the Jetson, so you can confirm the command lands before wiring it into a real
    shutdown path. ``--raw`` echoes the unparsed serial lines.
    """
    import argparse

    parser = argparse.ArgumentParser(description="Duckiebattery monitor / power-off")
    parser.add_argument("--port", default=None, help="force a /dev/ttyACM* node")
    parser.add_argument("--raw", action="store_true", help="echo raw serial frames")
    parser.add_argument("--shutdown", nargs="?", type=int, const=20, default=None,
                        metavar="SECONDS",
                        help="arm the output cutoff after SECONDS (default 20) and exit")
    parser.add_argument("--force-timed", action="store_true",
                        help="send the timed Q{nn} form even on 2.0.1+ firmware "
                             "(test whether the pack honours a chosen delay)")
    args = parser.parse_args()

    on_raw = (lambda s: print("raw:", s)) if args.raw else None
    with Duckiebattery(port=args.port, on_raw=on_raw) as bat:
        if args.shutdown is not None:
            print("Waiting for the pack...")
            bat.request_info(wait=True)
            print("Firmware:", bat.firmware_version, " Serial:", bat.serial_number)
            ok = bat.turn_off(timeout=args.shutdown, wait=True,
                              force_timed=args.force_timed)
            if ok:
                ttl = bat.last_ttl_sec
                if ttl is not None:
                    print("Pack acknowledged. Output cutoff armed for ~%ds." % ttl)
                else:
                    print("Pack acknowledged (bare QACK, no duration reported).")
                    print("On 2.0.1+ the delay is the firmware's own built-in value,")
                    print("not the %ds you asked for. Try --force-timed to test a" % args.shutdown)
                    print("chosen delay, and watch the Jetson power LED to time it.")
                print("Raw ack:", bat.last_ack)
                print("")
                print("NOTE: the countdown started NOW, at arm time. The rail will")
                print("drop when it expires whether or not the OS has halted, so run")
                print("'sudo poweroff' immediately to be down before the cut.")
            else:
                print("No acknowledgement.")
                if args.force_timed:
                    print("This firmware may not accept the timed Q%02d form."
                          % max(1, min(99, args.shutdown)))
                    print("Do NOT poweroff: the pack did not confirm a cutoff.")
                else:
                    print("Is the pack present and on the data cable?")
            return

        print("Reading battery (Ctrl-C to stop)...")
        try:
            while True:
                r = bat.latest()
                if r is None:
                    print("  (waiting for first frame)")
                else:
                    state = "charging" if r.charging else "on battery"
                    print("  %3s%%  %s  cell=%.2fV in=%.2fV I=%.2fA T=%.1fC"
                          % (r.percentage, state, r.cell_voltage or 0.0,
                             r.input_voltage or 0.0, r.current or 0.0,
                             r.temperature or 0.0))
                time.sleep(1.0)
        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    main()