#!/usr/bin/env python3
"""
Two-arm discovery probe.

A single DM-USB2FDCAN (dual) adapter exposes two CAN channels (0 and 1). With
both arms wired to one adapter, the expectation is one arm per channel. This
script opens BOTH channels, reports whether each opens, and for ids 1..8 tries:
  - a robust register read (MST/ESC/PMAX/VMAX/TMAX), then
  - a bare request_feedback fallback
across a few feedback-id candidates, so motors with a non-standard MST_ID
(e.g. the J8009P that slips through the default scan) still show up.
"""
from __future__ import annotations

import sys
import time

from motorbridge import Controller

DEVICE_TYPE = "usb2canfd-dual"
MODEL = "4310"
TIMEOUT_MS = 60
RID_MST_ID, RID_ESC_ID = 7, 8
RID_PMAX, RID_VMAX, RID_TMAX = 21, 22, 23


def fid_candidates(sid: int):
    cands = [sid + 0x10, sid, 0x100 + sid, 0x200 + sid, 0x300 + sid]
    seen, out = set(), []
    for c in cands:
        if 0 <= c <= 0x7FF and c not in seen:
            seen.add(c)
            out.append(c)
    return out


def open_bus(ch: str, tries: int = 6):
    last = None
    for _ in range(tries):
        try:
            return Controller.from_dm_device(DEVICE_TYPE, ch)
        except Exception as e:
            last = e
            time.sleep(0.4)
    raise RuntimeError(last)


def reg_read(ctrl, sid, fid):
    m = ctrl.add_damiao_motor(sid, fid, MODEL)
    try:
        info = {"mst": m.get_register_u32(RID_MST_ID, TIMEOUT_MS),
                "esc": m.get_register_u32(RID_ESC_ID, TIMEOUT_MS)}
        try:
            info["pmax"] = round(m.get_register_f32(RID_PMAX, TIMEOUT_MS), 2)
            info["vmax"] = round(m.get_register_f32(RID_VMAX, TIMEOUT_MS), 2)
            info["tmax"] = round(m.get_register_f32(RID_TMAX, TIMEOUT_MS), 2)
        except Exception:
            pass
        return info
    except Exception:
        return None
    finally:
        try:
            m.close()
        except Exception:
            pass


def fb_read(ctrl, sid, fid):
    m = ctrl.add_damiao_motor(sid, fid, MODEL)
    try:
        m.request_feedback()
        for _ in range(12):
            ctrl.poll_feedback_once()
            st = m.get_state()
            if st is not None:
                return {"status": st.status_code, "pos": round(st.pos, 3)}
            time.sleep(0.008)
        return None
    except Exception:
        return None
    finally:
        try:
            m.close()
        except Exception:
            pass


def attempt(ch, sid, fid):
    """Fresh controller per attempt -- the SDK keys motors by motor_id, so a
    second add of the same id on a live controller fails ('already exists')."""
    ctrl = open_bus(ch)
    try:
        r = reg_read(ctrl, sid, fid)
        if r:
            r["method"] = "reg"
            return r
        r = fb_read(ctrl, sid, fid)
        if r:
            r["method"] = "fb"
            return r
        return None
    finally:
        for fn in (ctrl.close_bus, ctrl.close):
            try:
                fn()
            except Exception:
                pass


def main():
    ids = [int(a, 0) for a in sys.argv[1:]] or list(range(1, 9))
    for ch in ["0", "1"]:
        print(f"\n=== channel {ch} ===")
        try:
            open_bus(ch).close()
        except Exception as e:
            print(f"  [FAIL] could not open channel {ch}: {e}")
            continue
        for sid in ids:
            hit = None
            for fid in fid_candidates(sid):
                try:
                    hit = attempt(ch, sid, fid)
                except Exception as e:
                    print(f"  id {sid:2d} fid 0x{fid:X} open error: {e}")
                    continue
                if hit:
                    hit["fid"] = fid
                    break
            if hit and hit["method"] == "reg":
                print(f"  id {sid:2d} fid 0x{hit['fid']:X} [reg] "
                      f"MST=0x{hit['mst']:X} ESC=0x{hit['esc']:X} "
                      f"pmax={hit.get('pmax')} vmax={hit.get('vmax')} tmax={hit.get('tmax')}")
            elif hit:
                print(f"  id {sid:2d} fid 0x{hit['fid']:X} [fb]  "
                      f"status={hit['status']} pos={hit['pos']}")
            else:
                print(f"  id {sid:2d} -- no reply")


if __name__ == "__main__":
    main()
