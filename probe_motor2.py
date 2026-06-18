#!/usr/bin/env python3
"""
Targeted probe for a DaMiao J8009P at logical "motor 2".

The dashboard scan only finds a motor if it answers a bare request_feedback()
on feedback_id = id+0x10 while decoded as the default model. Big DM motors
(8009) are frequently configured with a different MST_ID (feedback id) and/or
ESC_ID (receive id), so they slip through. This script sweeps:

  - send/receive id (ESC_ID candidates) around the target
  - feedback id (MST_ID candidates)
  - two detection methods: register-read (robust) and request_feedback

and reports anything that answers.
"""
from __future__ import annotations

import sys
import time

from motorbridge import Controller

RID_MST_ID = 7   # feedback id
RID_ESC_ID = 8   # receive id
RID_PMAX, RID_VMAX, RID_TMAX = 21, 22, 23

DEVICE_TYPE = "usb2canfd-dual"
MODEL = "8009"            # decode as J8009 family
TIMEOUT_MS = 60

# Logical target. The 8009P *should* be id 2, but we also probe neighbours in
# case its ESC_ID was left at a different value.
SEND_IDS = [2, 1, 3, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F]
# Feedback-id candidates to try for each send id. The SDK keys a controller by
# motor_id, so each (send_id, fid) attempt uses a FRESH controller -- keep the
# candidate list focused so the total number of bus opens stays reasonable.
def fid_candidates(send_id: int) -> list[int]:
    cands = [send_id + 0x10, send_id, 0x100 + send_id, 0x200 + send_id,
             0x300 + send_id, 0x00, 0x01, 0x10, 0x11]
    seen, out = set(), []
    for c in cands:
        if 0 <= c <= 0x7FF and c not in seen:
            seen.add(c); out.append(c)
    return out


def open_bus(ch: str, tries: int = 8):
    last = None
    for _ in range(tries):
        try:
            return Controller.from_dm_device(DEVICE_TYPE, ch)
        except Exception as e:  # bus may be briefly held by the dashboard
            last = e
            time.sleep(0.4)
    raise RuntimeError(f"could not open dm-device ch{ch}: {last}")


def try_register_read(ctrl, send_id: int, fid: int):
    """Return dict if the motor answered a register read, else None."""
    motor = ctrl.add_damiao_motor(send_id, fid, MODEL)
    try:
        mst = motor.get_register_u32(RID_MST_ID, TIMEOUT_MS)
        esc = motor.get_register_u32(RID_ESC_ID, TIMEOUT_MS)
        info = {"method": "register", "send_id": send_id, "fid": fid,
                "mst_id": mst, "esc_id": esc}
        try:
            info["pmax"] = round(motor.get_register_f32(RID_PMAX, TIMEOUT_MS), 3)
            info["vmax"] = round(motor.get_register_f32(RID_VMAX, TIMEOUT_MS), 3)
            info["tmax"] = round(motor.get_register_f32(RID_TMAX, TIMEOUT_MS), 3)
        except Exception:
            pass
        return info
    except Exception:
        return None
    finally:
        try:
            motor.close()
        except Exception:
            pass


def try_feedback(ctrl, send_id: int, fid: int):
    motor = ctrl.add_damiao_motor(send_id, fid, MODEL)
    try:
        motor.request_feedback()
        for _ in range(12):
            ctrl.poll_feedback_once()
            st = motor.get_state()
            if st is not None:
                return {"method": "feedback", "send_id": send_id, "fid": fid,
                        "status": st.status_code, "pos": round(st.pos, 4),
                        "vel": round(st.vel, 4), "torq": round(st.torq, 4)}
            time.sleep(0.008)
        return None
    except Exception:
        return None
    finally:
        try:
            motor.close()
        except Exception:
            pass


def attempt(ch: str, sid: int, fid: int):
    """Fresh controller per attempt (SDK keys motors by motor_id)."""
    ctrl = open_bus(ch)
    try:
        r = try_register_read(ctrl, sid, fid)
        if r is None:
            r = try_feedback(ctrl, sid, fid)
        return r
    finally:
        try:
            ctrl.close_bus()
        except Exception:
            pass
        try:
            ctrl.close()
        except Exception:
            pass


def main():
    targets = [int(a, 0) for a in sys.argv[1:]] or SEND_IDS
    hits = []
    for ch in ["0", "1"]:
        print(f"[ch{ch}] probing send-ids {targets}")
        for sid in targets:
            got = False
            for fid in fid_candidates(sid):
                try:
                    r = attempt(ch, sid, fid)
                except Exception as e:
                    print(f"  [ch{ch}] open/attempt error at send=0x{sid:X} fid=0x{fid:X}: {e}")
                    continue
                if r:
                    r["channel"] = ch
                    if r["method"] == "register":
                        print(f"  [HIT/reg] ch{ch} send=0x{sid:X} fid=0x{fid:X} "
                              f"-> MST=0x{r['mst_id']:X} ESC=0x{r['esc_id']:X} "
                              f"pmax={r.get('pmax')} vmax={r.get('vmax')} tmax={r.get('tmax')}")
                    else:
                        print(f"  [HIT/fb]  ch{ch} send=0x{sid:X} fid=0x{fid:X} "
                              f"-> status={r['status']} pos={r['pos']} vel={r['vel']} torq={r['torq']}")
                    hits.append(r); got = True
                    break
            if not got:
                print(f"  [ .. ]    ch{ch} send=0x{sid:X} no reply")

    print("\n=== summary ===")
    if not hits:
        print("no motors answered on the probed ids")
    for h in hits:
        print(h)


if __name__ == "__main__":
    main()
