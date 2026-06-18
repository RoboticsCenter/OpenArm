#!/usr/bin/env python3
"""
Concurrent dual-channel diagnostic.

The dashboard now keeps BOTH CAN channels of the dm-usb2canfd-dual adapter open
at the same time (one Controller per channel) so the two arms can run together.
This script reproduces that exact situation -- it opens channel 0 AND channel 1
simultaneously and then asks each id on each channel for feedback, printing the
live position. Run it with the arms powered.

What to look for
----------------
* If channel 0 and channel 1 report DIFFERENT motors / positions, the two
  channels are genuinely independent buses and concurrent two-arm control works.
* If both channels report the SAME motors (e.g. id 1 answers on both, with the
  same position that changes together when you move one joint by hand), then the
  second controller is aliasing onto the first physical bus -- the two arms are
  NOT on independent channels, which explains "arm 2 calibrating but arm 1
  moves".

Usage:
    python diag_dual.py            # probe ids 1..8 on both channels
    python diag_dual.py 1 3 5      # probe specific ids
"""
from __future__ import annotations

import sys
import time

from motorbridge import Controller

DEVICE_TYPE = "usb2canfd-dual"
MODEL = "4310"


def open_both():
    ctrls = {}
    for ch in ("0", "1"):
        try:
            ctrls[ch] = Controller.from_dm_device(DEVICE_TYPE, ch)
            print(f"[ok] opened channel {ch}  (ptr={hex(ctrls[ch]._ptr)})")
        except Exception as e:  # noqa: BLE001
            print(f"[FAIL] could not open channel {ch}: {e}")
    return ctrls


def probe(ctrl, ch, ids):
    found = []
    for mid in ids:
        fid = mid + 0x10
        try:
            m = ctrl.add_damiao_motor(mid, fid, MODEL)
        except Exception as e:  # noqa: BLE001
            print(f"  ch{ch} id {mid}: add failed ({e})")
            continue
        hit = None
        try:
            m.request_feedback()
            for _ in range(15):
                ctrl.poll_feedback_once()
                st = m.get_state()
                if st is not None:
                    hit = st
                    break
                time.sleep(0.008)
        except Exception as e:  # noqa: BLE001
            print(f"  ch{ch} id {mid}: feedback error ({e})")
        finally:
            try:
                m.close()
            except Exception:
                pass
        if hit is not None:
            print(f"  ch{ch} id {mid:2d}: pos={hit.pos:+.4f} vel={hit.vel:+.4f} "
                  f"status={hit.status_code}")
            found.append(mid)
        else:
            print(f"  ch{ch} id {mid:2d}: no reply")
    return found


def main():
    ids = [int(a, 0) for a in sys.argv[1:]] or list(range(1, 9))
    print("Opening BOTH channels concurrently (as the dashboard does)...")
    ctrls = open_both()
    if not ctrls:
        print("no channels opened; is the adapter plugged in?")
        return
    try:
        summary = {}
        for ch, ctrl in ctrls.items():
            print(f"\n=== channel {ch} (while both open) ===")
            summary[ch] = probe(ctrl, ch, ids)
        print("\n=== summary ===")
        for ch, found in summary.items():
            print(f"  channel {ch}: ids {found}")
        if len(summary) == 2:
            a, b = summary.get("0", []), summary.get("1", [])
            if a and a == b:
                print("\n[!] Both channels report the SAME ids. The channels are "
                      "likely NOT independent buses while both are open.")
            else:
                print("\n[ok] Channels report different ids -> independent buses.")
    finally:
        for ctrl in ctrls.values():
            for fn in (ctrl.close_bus, ctrl.close):
                try:
                    fn()
                except Exception:
                    pass


if __name__ == "__main__":
    main()
