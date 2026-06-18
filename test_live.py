#!/usr/bin/env python3
"""Live hardware check: scan + connect + read telemetry via MotorService."""
import time
from backend import MotorService

svc = MotorService()
print("scanning bus 1..16 ...")
found = svc.scan(1, 16)
print("found:", found)

if found:
    mid = found[0]["motor_id"]
    print(f"connecting to motor id {mid} ...")
    st = svc.connect(motor_id=mid, autodetect_model=True)
    print("connected. model=", st["model"], "limits=", st["limits"])
    for _ in range(10):
        s = svc.status()["state"]
        print(f"  pos={s['pos']:+.3f} rad  vel={s['vel']:+.3f}  torq={s['torq']:+.3f}  "
              f"Tmos={s['t_mos']} Trot={s['t_rotor']} status={s['status_code']} online={s['online']}")
        time.sleep(0.2)
    svc.disable()
print("done (no crash).")
svc.shutdown()
