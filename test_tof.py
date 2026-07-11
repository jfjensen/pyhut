#!/usr/bin/env python3
"""Manual on-device test for the extended VL53L0X driver.

Run as root on the Jetson Nano with the ToF wired and answering at 0x29:

    sudo python3 -m pyhut.test_tof

Point the sensor at a wall ~10-30 cm away for the "valid" checks, then at open
space (ceiling / out a window, beyond ~1.2 m) to trigger the invalid / out-of-
range path. Std lib only, Python 3.6 compatible.
"""

import time

from .duckiebot import VL53L0X


def main():
    # io_timeout_s must exceed the measurement timing budget, or reads will
    # time out. 0.3 s comfortably covers budgets up to ~200 ms.
    tof = VL53L0X(io_timeout_s=0.3)
    try:
        print("ID check passed -- sensor is on the bus.")
        print("timing budget: %d us" % tof.measurement_timing_budget)

        # 1. Single-shot still works, still returns a plain int.
        mm = tof.read_mm()
        print("\n[single-shot] read_mm() -> %r (%s)" % (mm, type(mm).__name__))
        assert isinstance(mm, int)

        # 2. Continuous mode: start once, read a burst, time it.
        print("\n[continuous] starting...")
        tof.start_continuous()
        assert tof.is_continuous_mode
        t0 = time.monotonic()
        n = 50
        valid = 0
        for _ in range(n):
            r = tof.read_continuous()          # int on good read, None on bad
            if r is not None:
                valid += 1
        dt = time.monotonic() - t0
        print("  %d reads in %.3f s -> %.1f Hz (%d valid, %d invalid)"
              % (n, dt, n / dt, valid, n - valid))
        tof.stop_continuous()
        assert not tof.is_continuous_mode

        # 3. Invalid-reading path. Aim at open space (beyond range) and watch
        #    for None. Ctrl-C when you've seen enough.
        print("\n[status] aim at open space to force out-of-range (Ctrl-C to skip)")
        tof.start_continuous()
        try:
            for _ in range(40):
                r = tof.read_continuous()
                print("  %s" % ("None (invalid/out-of-range)" if r is None
                                else "%d mm" % r))
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            tof.stop_continuous()

        # 4. Timing-budget lever: faster budget -> higher scan rate.
        print("\n[budget] 20 ms (fast) vs 200 ms (accurate)")
        for budget in (20000, 200000):
            tof.measurement_timing_budget = budget
            tof.start_continuous()
            t0 = time.monotonic()
            for _ in range(20):
                tof.read_continuous()
            dt = time.monotonic() - t0
            print("  budget %6d us -> %.1f Hz" % (budget, 20 / dt))
            tof.stop_continuous()

        print("\nAll paths exercised.")
    finally:
        tof.close()


if __name__ == "__main__":
    main()
