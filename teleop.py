#!/usr/bin/env python3
"""Real-time CAN-to-CAN teleoperation for OpenArm / DaMiao DM arms.

This drives one or more LEADER -> FOLLOWER bus pairs at once. On each pair the
LEADER arm is left limp (torque off) so you can backdrive it by hand, while the
FOLLOWER arm is energized in MIT mode and commanded every tick to the leader's
live joint positions -- so it mirrors your motion.

Bimanual setup on this machine
------------------------------
There are two OpenArm units, each a left+right pair, each unit wired to its own
gs_usb CAN-FD adapter (two channels per adapter) -> four SocketCAN interfaces:

    unit 1 (leader):   can0 = left arm,   can1 = right arm   (adapter A)
    unit 2 (follower): can2 = left arm,   can3 = right arm   (adapter B)

So the default mapping mirrors like-for-like sides:

    can0 (lead left)  -> can2 (follow left)
    can1 (lead right) -> can3 (follow right)

Move either of unit 1's arms by hand and the matching arm on unit 2 follows.
Use ``--swap`` to make unit 2 the leader instead. Use ``--pairs`` to describe a
different wiring entirely.

Transport
---------
``canX`` specs use SocketCAN-FD (gs_usb / candleLight). A bare ``0``/``1`` or
``dm:0`` spec uses a DaMiao DM-USB2FDCAN adapter, and ``/dev/ttyACM0`` a DaMiao
serial bridge -- so this keeps working if the hardware is swapped back.

A SocketCAN interface must be UP before use. Bring all four up once per boot:

    for i in 0 1 2 3; do
      sudo ip link set can$i type can bitrate 1000000 dbitrate 5000000 fd on
      sudo ip link set can$i up
    done

Always run inside the project venv: ``./venv/bin/python teleop.py ...``.

Recommended first run
---------------------
ALWAYS dry-run first. It opens every bus, leaves BOTH arms limp, and just prints
each leader joint next to its follower joint so you can confirm the mapping and
that the two arms agree on zero -- with nothing energized and nothing moving:

    ./venv/bin/python teleop.py --dry-run

When the numbers line up (move a leader joint, watch the matching pair change),
run for real:

    ./venv/bin/python teleop.py                 # unit 1 leads, unit 2 follows
    ./venv/bin/python teleop.py --swap           # unit 2 leads, unit 1 follows
    ./venv/bin/python teleop.py --pairs can0:can2 # left side only
    ./venv/bin/python teleop.py --kp 8 --ramp 2.0 # gentler, slower ramp-in

Safety (READ before energizing)
-------------------------------
* Keep a hand on the e-stop / Ctrl-C. On Ctrl-C or any exit the script ALWAYS
  disables every motor on every bus and closes them.
* On start each follower EASES from its current pose to the leader pose over
  ``--ramp`` seconds -- it moves on its own during that window. Stand clear and
  start each follower arm near its matching leader's pose.
* Follower targets are clamped to conservative soft limits (the overlap of the
  saved calibrations) and the per-tick change is slew-limited, so a feedback
  glitch can't slam an arm.
* Start with low gains; raise ``--kp`` only once tracking looks safe. Joint 1 is
  the heavy 8009 base motor -- be especially careful there.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional

# --------------------------------------------------------------------------- #
# Environment / driver setup (mirrors move.py / run.sh)
# --------------------------------------------------------------------------- #


def _ensure_driver_lib() -> None:
    """Point the DM_Device SDK at the cached runtime, like run.sh / move.py do.

    Only relevant for the DaMiao USB/serial transports; harmless for SocketCAN.
    """
    if os.environ.get("MOTOR_DM_DEVICE_LIB"):
        return
    lib = Path.home() / ".cache/motorbridge/dm_device/v1.1.0/linux/x86_64/libdm_device.so"
    if lib.is_file():
        os.environ["MOTOR_DM_DEVICE_LIB"] = str(lib)


CALIBRATION_STORE_PATH = Path(os.environ.get(
    "DM_CALIBRATION_STORE", str(Path(__file__).with_name("calibrations.json"))))

# PMAX (rad), VMAX (rad/s), TMAX (Nm) per model -- soft-limit fallback when a
# joint has no saved calibration. Copied from backend.DAMIAO_MODEL_LIMITS.
DAMIAO_MODEL_LIMITS: Dict[str, tuple] = {
    "3507": (12.566, 50.0, 5.0),
    "4310": (12.5, 30.0, 10.0),
    "4310P": (12.5, 50.0, 10.0),
    "4340": (12.5, 10.0, 28.0),
    "4340P": (12.5, 10.0, 28.0),
    "4340_v20": (12.5, 20.0, 28.0),
    "6006": (12.5, 45.0, 20.0),
    "8006": (12.5, 45.0, 40.0),
    "8009": (12.5, 45.0, 54.0),
    "10010L": (12.5, 25.0, 200.0),
    "10010": (12.5, 20.0, 200.0),
    "H3510": (12.5, 280.0, 1.0),
    "G6215": (12.5, 45.0, 10.0),
    "H6220": (12.5, 45.0, 10.0),
    "JH11": (12.5, 10.0, 12.0),
    "6248P": (12.566, 20.0, 120.0),
}

# DaMiao feedback status codes (low nibble of the MIT feedback frame).
DM_STATUS_TEXT = {
    0: "disabled", 1: "enabled", 8: "overvoltage", 9: "undervoltage",
    10: "overcurrent", 11: "MOSFET over-temp", 12: "rotor over-temp",
    13: "comm loss", 14: "overload",
}
# Codes >= 8 are latched hardware faults: never auto-re-enable into them.
DM_FAULT_MIN = 8

# Gentle fallback gains when a joint has no saved calibration gains.
DEFAULT_KP = 12.0
DEFAULT_KD = 0.5

# Default bimanual wiring: leader unit (adapter A) -> follower unit (adapter B),
# matching sides (left->left, right->right).
DEFAULT_PAIRS = "can0:can2,can1:can3"
DEFAULT_IDS = "1,3,4,5,6,7,8"


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


# --------------------------------------------------------------------------- #
# Transport classification (mirrors dk1_dashboard.py)
# --------------------------------------------------------------------------- #


def _classify_bus(spec: str) -> str:
    s = (spec or "").strip()
    if s.startswith("can"):
        return "socketcan"
    if s.startswith("/dev/") or s.startswith("tty"):
        return "dm_serial"
    return "dm_device"  # "dm:0", "dm:1", or bare "0"/"1"


def _can_is_up(channel: str) -> Optional[bool]:
    """True/False if the SocketCAN interface exists, None if it doesn't."""
    try:
        with open(f"/sys/class/net/{channel}/flags") as f:
            return bool(int(f.read().strip(), 16) & 0x1)  # IFF_UP
    except FileNotFoundError:
        return None
    except Exception:  # noqa: BLE001
        return None


def _is_enobufs(exc: BaseException) -> bool:
    msg = str(exc).lower()
    return "no buffer space" in msg or "os error 105" in msg


def _retry_enobufs(fn, *args, attempts: int = 6, delay: float = 0.0015):
    """Call ``fn(*args)``, retrying briefly on the transient SocketCAN ENOBUFS.

    gs_usb CAN-FD adapters have a shallow tx queue; a burst of frames can briefly
    fill it and the non-blocking write returns ENOBUFS. A genuine transient
    drains in well under a millisecond, so a short bounded retry smooths it.
    """
    last: Optional[BaseException] = None
    for _ in range(attempts):
        try:
            return fn(*args)
        except Exception as e:  # noqa: BLE001
            if not _is_enobufs(e):
                raise
            last = e
            time.sleep(delay)
    if last is not None:
        raise last


def open_controller(spec: str, device_type: str):
    """Open one motorbridge Controller for the given bus spec."""
    from motorbridge import Controller

    kind = _classify_bus(spec)
    if kind == "socketcan":
        up = _can_is_up(spec)
        if up is None:
            raise RuntimeError(
                f"CAN interface '{spec}' not found. Plug in the adapter, or pass "
                f"a 'dm:0'/'dm:1' DM-USB2FDCAN channel.")
        if up is False:
            raise RuntimeError(
                f"CAN interface '{spec}' is DOWN. Bring it up:\n"
                f"  sudo ip link set {spec} type can bitrate 1000000 "
                f"dbitrate 5000000 fd on\n"
                f"  sudo ip link set {spec} up")
        return Controller.from_socketcanfd(spec)
    if kind == "dm_serial":
        path = spec if spec.startswith("/dev/") else f"/dev/{spec}"
        if not os.path.exists(path):
            raise RuntimeError(f"serial bridge '{path}' not found.")
        baud = int(os.environ.get("DM_SERIAL_BAUD", "921600"))
        return Controller.from_dm_serial(path, baud)
    # dm_device: "dm:0" / "dm:1" / bare "0"/"1"
    ch = spec.split(":", 1)[1] if ":" in spec else spec
    return Controller.from_dm_device(device_type, ch or "0")


# --------------------------------------------------------------------------- #
# Calibration
# --------------------------------------------------------------------------- #


def load_calibrations() -> dict:
    try:
        with open(CALIBRATION_STORE_PATH) as f:
            return json.load(f)
    except Exception:  # noqa: BLE001
        return {}


def joint_calibration(cals: dict, motor_id: int) -> dict:
    """Aggregate everything we know about one joint id across ALL saved arms.

    There is no per-can calibration yet (the saved data is keyed by the old
    adapter channels "0"/"1"), and every arm is the same OpenArm design, so we
    derive CONSERVATIVE soft limits: the OVERLAP (intersection) of every saved
    range for this joint id. That way a follower target can never exceed the
    known travel of any arm. Gains come from the matching records too.
    """
    recs = [v for k, v in cals.items()
            if isinstance(v, dict) and v.get("motor_id") == motor_id]
    model = None
    kp = kd = None
    lo = -math.inf
    hi = math.inf
    for r in recs:
        model = model or r.get("model")
        try:
            lo = max(lo, float(r["pos_min"]))
            hi = min(hi, float(r["pos_max"]))
        except (KeyError, TypeError, ValueError):
            pass
        res = r.get("result") or {}
        if kp is None and res.get("kp") is not None:
            kp = float(res["kp"])
        if kd is None and res.get("kd") is not None:
            kd = float(res["kd"])
    return {"model": model, "kp": kp, "kd": kd,
            "pos_min": (None if lo == -math.inf else lo),
            "pos_max": (None if hi == math.inf else hi)}


# --------------------------------------------------------------------------- #
# Per-joint config
# --------------------------------------------------------------------------- #


class Joint:
    """Everything teleop needs to track one leader->follower joint pair."""

    def __init__(self, motor_id: int, model: str, kp: float, kd: float,
                 pos_min: float, pos_max: float, sign: float, offset: float):
        self.motor_id = motor_id
        self.feedback_id = motor_id + 0x10
        self.model = model
        self.kp = kp
        self.kd = kd
        self.pos_min = pos_min
        self.pos_max = pos_max
        self.sign = sign        # +1 or -1 (mirror flip)
        self.offset = offset    # rad added after the sign flip

        # Live handles / state, filled in at connect time.
        self.leader_motor = None
        self.follower_motor = None
        self.leader_pos: float = float("nan")
        self.follower_pos: float = float("nan")
        self.follower_status: int = -1
        self.target: float = 0.0          # last commanded follower target
        self.faulted: bool = False
        self.fault_text: str = ""
        self._recover_count: int = 0
        self._last_recover_ts: float = 0.0

    def map_leader_to_target(self, leader_pos: float) -> float:
        """Leader raw position -> clamped follower setpoint."""
        t = self.sign * leader_pos + self.offset
        return _clamp(t, self.pos_min, self.pos_max)


def build_joints(ids: List[int], cals: dict, model_override: Optional[str],
                 kp_override: Optional[float], kd_override: Optional[float],
                 inverts: set, offsets: Dict[int, float]) -> List[Joint]:
    joints: List[Joint] = []
    for mid in ids:
        c = joint_calibration(cals, mid)
        model = model_override or c["model"] or "4310"
        pmax = DAMIAO_MODEL_LIMITS.get(model, (12.5, 30.0, 10.0))[0]

        pos_min = c["pos_min"] if c["pos_min"] is not None else -pmax
        pos_max = c["pos_max"] if c["pos_max"] is not None else pmax
        if pos_max <= pos_min:  # guard against a degenerate record
            pos_min, pos_max = -pmax, pmax

        kp = kp_override if kp_override is not None else (c["kp"] or DEFAULT_KP)
        kd = kd_override if kd_override is not None else (c["kd"] or DEFAULT_KD)

        sign = -1.0 if mid in inverts else 1.0
        offset = float(offsets.get(mid, 0.0))
        joints.append(Joint(mid, model, kp, kd, pos_min, pos_max, sign, offset))
    return joints


# --------------------------------------------------------------------------- #
# One leader->follower bus pair ("side")
# --------------------------------------------------------------------------- #


class Side:
    def __init__(self, leader_bus: str, follower_bus: str):
        self.leader_bus = leader_bus
        self.follower_bus = follower_bus
        self.leader_sc = _classify_bus(leader_bus) == "socketcan"
        self.follower_sc = _classify_bus(follower_bus) == "socketcan"
        self.leader_ctrl = None
        self.follower_ctrl = None
        self.joints: List[Joint] = []

    @property
    def name(self) -> str:
        return f"{self.leader_bus}->{self.follower_bus}"


# --------------------------------------------------------------------------- #
# CAN helpers
# --------------------------------------------------------------------------- #


def read_state(ctrl, motor, socketcan: bool):
    """Poll one motor for a fresh state; returns the state object or None."""
    retries = 8 if socketcan else 15
    for _ in range(retries):
        try:
            motor.request_feedback()
        except Exception:  # noqa: BLE001
            pass
        try:
            ctrl.poll_feedback_once()
        except Exception:  # noqa: BLE001
            pass
        st = motor.get_state()
        if st is not None:
            return st
        time.sleep(0.002)
    return None


def refresh_positions(ctrl, joints: List[Joint], which: str) -> None:
    """Batch-refresh leader or follower positions for one side in one pass.

    Requests feedback from every joint, drains the bus, then reads cached state
    -- much cheaper than a per-joint retry loop when running many buses at once.
    """
    motors = []
    for j in joints:
        m = j.leader_motor if which == "leader" else j.follower_motor
        if m is None:
            continue
        motors.append((j, m))
        try:
            m.request_feedback()
        except Exception:  # noqa: BLE001
            pass
    for _ in range(len(motors) + 2):
        try:
            ctrl.poll_feedback_once()
        except Exception:  # noqa: BLE001
            break
    for j, m in motors:
        try:
            st = m.get_state()
        except Exception:  # noqa: BLE001
            st = None
        if st is None:
            continue
        if which == "leader":
            j.leader_pos = float(st.pos)
        else:
            j.follower_pos = float(st.pos)
            j.follower_status = st.status_code


def clean_enable(motor, socketcan: bool) -> None:
    """Drop torque, assert MIT mode, then energize (mirrors move.py)."""
    send = (lambda fn, *a: _retry_enobufs(fn, *a)) if socketcan else (lambda fn, *a: fn(*a))
    try:
        motor.disable()
    except Exception:  # noqa: BLE001
        pass
    time.sleep(0.03)
    try:
        motor.clear_error()
    except Exception:  # noqa: BLE001
        pass
    try:
        from motorbridge import Mode
        send(motor.ensure_mode, Mode.MIT)
    except Exception:  # noqa: BLE001
        pass  # MIT frames are native on DM motors; the ack is just flaky.
    try:
        send(motor.enable)
    except Exception:  # noqa: BLE001
        pass


# --------------------------------------------------------------------------- #
# Main teleop session
# --------------------------------------------------------------------------- #


class Teleop:
    def __init__(self, args):
        self.args = args
        self.sides: List[Side] = []
        self._ctrls: Dict[str, object] = {}   # bus spec -> Controller (shared)
        self._stop = False
        self._status_h = 0   # number of status lines last printed (in-place redraw)

    # -- setup ----------------------------------------------------------- #
    def _open_bus(self, spec: str):
        if spec not in self._ctrls:
            self._ctrls[spec] = open_controller(spec, self.args.device_type)
        return self._ctrls[spec]

    def connect(self) -> None:
        a = self.args
        cals = load_calibrations()
        ids = _parse_id_list(a.ids, ordered=True)
        inverts = _parse_id_list(a.invert)
        offsets = _parse_offsets(a.offset)

        pairs = _parse_pairs(a.pairs, swap=a.swap)
        if not pairs:
            raise RuntimeError("no leader->follower pairs configured (--pairs)")
        self.sides = [Side(lb, fb) for lb, fb in pairs]

        # Open every distinct bus exactly once.
        for s in self.sides:
            print(f"opening LEADER bus   '{s.leader_bus}' ...")
            s.leader_ctrl = self._open_bus(s.leader_bus)
            print(f"opening FOLLOWER bus '{s.follower_bus}' ...")
            s.follower_ctrl = self._open_bus(s.follower_bus)

        for s in self.sides:
            s.joints = build_joints(ids, cals, a.model, a.kp, a.kd, inverts, offsets)
            for j in s.joints:
                j.leader_motor = s.leader_ctrl.add_damiao_motor(
                    j.motor_id, j.feedback_id, j.model)
                j.follower_motor = s.follower_ctrl.add_damiao_motor(
                    j.motor_id, j.feedback_id, j.model)
                if s.leader_sc:
                    try:
                        j.leader_motor.set_can_timeout_ms(50)
                    except Exception:  # noqa: BLE001
                        pass
                if s.follower_sc:
                    try:
                        j.follower_motor.set_can_timeout_ms(50)
                    except Exception:  # noqa: BLE001
                        pass

        # Leaders (and, in dry-run, followers too) stay limp.
        for s in self.sides:
            for j in s.joints:
                try:
                    j.leader_motor.disable()
                except Exception:  # noqa: BLE001
                    pass
                if a.dry_run:
                    try:
                        j.follower_motor.disable()
                    except Exception:  # noqa: BLE001
                        pass

    # -- run ------------------------------------------------------------- #
    def run(self) -> None:
        if self.args.dry_run:
            self._run_dry()
        else:
            self._run_live()

    def _run_dry(self) -> None:
        self._banner()
        print("\nDRY RUN: both arms limp, nothing energized. Move a LEADER joint "
              "by hand\nand watch its FOLLOWER partner track the same number.  "
              "Ctrl-C to stop.\n")
        last_print = 0.0
        try:
            while not self._stop:
                t0 = time.time()
                for s in self.sides:
                    refresh_positions(s.leader_ctrl, s.joints, "leader")
                    refresh_positions(s.follower_ctrl, s.joints, "follower")
                now = time.time()
                if now - last_print >= 0.2:
                    last_print = now
                    self._emit_status("DRY")
                time.sleep(max(0.0, 0.02 - (time.time() - t0)))
        except KeyboardInterrupt:
            print("\ninterrupted")
        finally:
            self.shutdown()

    def _run_live(self) -> None:
        a = self.args
        self._banner()

        print("reading initial poses ...")
        for s in self.sides:
            refresh_positions(s.leader_ctrl, s.joints, "leader")
            refresh_positions(s.follower_ctrl, s.joints, "follower")

        print("enabling follower joints (MIT mode) ...")
        for s in self.sides:
            for j in s.joints:
                clean_enable(j.follower_motor, s.follower_sc)
                j.target = j.follower_pos if not math.isnan(j.follower_pos) else 0.0

        period = 1.0 / max(1.0, a.rate)
        ramp_ticks = max(1, int(a.ramp * a.rate))
        ramp_start = {(s.name, j.motor_id): j.target
                      for s in self.sides for j in s.joints}

        print(f"\nteleop live @ {a.rate:.0f} Hz  (ramp-in {a.ramp:.1f}s)  "
              f"-- Ctrl-C to stop\n")
        tick = 0
        last_print = 0.0
        try:
            while not self._stop:
                t0 = time.time()
                alpha = 1.0 if tick >= ramp_ticks else (tick + 1) / ramp_ticks

                for s in self.sides:
                    refresh_positions(s.leader_ctrl, s.joints, "leader")
                    self._drive_side(s, alpha, ramp_start, a)

                now = time.time()
                if now - last_print >= 0.25:
                    last_print = now
                    self._emit_status("RAMP" if alpha < 1.0 else "LIVE")

                tick += 1
                time.sleep(max(0.0, period - (time.time() - t0)))
        except KeyboardInterrupt:
            print("\ninterrupted")
        finally:
            self.shutdown()

    def _drive_side(self, s: Side, alpha: float, ramp_start: dict, a) -> None:
        for j in s.joints:
            if j.faulted:
                continue
            if math.isnan(j.leader_pos):
                desired = j.target  # hold last good target
            else:
                live = j.map_leader_to_target(j.leader_pos)
                if alpha < 1.0:
                    start = ramp_start.get((s.name, j.motor_id), live)
                    desired = start + alpha * (live - start)
                else:
                    desired = live
            if a.max_step > 0:  # slew-rate limit (anti-glitch)
                desired = _clamp(desired, j.target - a.max_step, j.target + a.max_step)
            j.target = _clamp(desired, j.pos_min, j.pos_max)

            args5 = (j.target, 0.0, j.kp, j.kd, a.tau)
            try:
                if s.follower_sc:
                    _retry_enobufs(j.follower_motor.send_mit, *args5)
                else:
                    j.follower_motor.send_mit(*args5)
            except Exception as e:  # noqa: BLE001
                j.fault_text = f"command error: {e}"

        # Drain follower feedback and supervise status.
        for _ in range(len(s.joints) + 2):
            try:
                s.follower_ctrl.poll_feedback_once()
            except Exception:  # noqa: BLE001
                break
        for j in s.joints:
            try:
                st = j.follower_motor.get_state()
            except Exception:  # noqa: BLE001
                st = None
            if st is not None:
                j.follower_pos = float(st.pos)
                j.follower_status = st.status_code
                self._supervise(s, j, st.status_code)

    def _supervise(self, s: Side, j: Joint, sc: int) -> None:
        """React to follower status: latch faults off, re-enable clean drops."""
        if sc >= DM_FAULT_MIN:
            if not j.faulted:
                j.faulted = True
                j.fault_text = f"hardware fault: {DM_STATUS_TEXT.get(sc, f'code {sc}')}"
                try:
                    j.follower_motor.disable()
                except Exception:  # noqa: BLE001
                    pass
                print(f"\n[FAULT] {s.name} joint {j.motor_id}: {j.fault_text} "
                      f"-- latched OFF, will not re-energize.")
            return
        if sc == 1:
            j._recover_count = 0
            return
        if sc == 0 and not j.faulted:
            now = time.time()
            if now - j._last_recover_ts < 0.25:
                return
            j._last_recover_ts = now
            j._recover_count += 1
            if j._recover_count > 12:
                j.faulted = True
                j.fault_text = "keeps dropping out -- check power/CAN wiring"
                print(f"\n[FAULT] {s.name} joint {j.motor_id}: {j.fault_text} "
                      f"-- latched OFF.")
                return
            clean_enable(j.follower_motor, s.follower_sc)

    # -- output ---------------------------------------------------------- #
    def _banner(self) -> None:
        a = self.args
        mode = "DRY RUN (no motion)" if a.dry_run else "LIVE"
        print("=" * 74)
        print(f"  OpenArm / DaMiao CAN-to-CAN teleoperation  [{mode}]")
        print("=" * 74)
        for s in self.sides:
            print(f"  {s.leader_bus} (lead, limp)  ->  {s.follower_bus} (follow"
                  f"{', limp' if a.dry_run else ', driven'})")
        if not a.dry_run:
            print(f"  rate {a.rate:.0f} Hz   ramp-in {a.ramp:.1f}s   "
                  f"max-step {a.max_step:.3f} rad/tick   tau {a.tau:.2f}")
        j0 = self.sides[0].joints if self.sides else []
        print("  joints (shared across sides):")
        for j in j0:
            inv = "  INVERTED" if j.sign < 0 else ""
            off = f"  offset={j.offset:+.3f}" if abs(j.offset) > 1e-9 else ""
            print(f"    id {j.motor_id:2d}  model {j.model:<9s} "
                  f"kp={j.kp:5.1f} kd={j.kd:4.2f}  "
                  f"limits [{j.pos_min:+.3f}, {j.pos_max:+.3f}]{inv}{off}")
        print("=" * 74)

    @staticmethod
    def _fmt(pos: float) -> str:
        return " nan " if math.isnan(pos) else f"{pos:+.2f}"

    def _emit_status(self, phase: str) -> None:
        """Redraw a stable block of one line per side (no scrolling/wrapping).

        Uses an ANSI cursor-up jump to overwrite the previous block in place, so
        a long multi-bus readout stays pinned instead of flooding the terminal.
        """
        lines = []
        for s in self.sides:
            parts = []
            for j in s.joints:
                if j.faulted:
                    tag = "!"
                elif j.follower_status not in (1, -1):
                    tag = f"s{j.follower_status}"
                else:
                    tag = ""
                parts.append(f"j{j.motor_id}:{self._fmt(j.leader_pos)}>"
                             f"{self._fmt(j.follower_pos)}{tag}")
            lines.append(f"[{phase} {s.name}] " + " ".join(parts))
        if self._status_h:
            sys.stdout.write(f"\033[{self._status_h}A")  # up N lines
        for ln in lines:
            sys.stdout.write("\r\033[K" + ln + "\n")       # clear line, print
        sys.stdout.flush()
        self._status_h = len(lines)

    # -- teardown -------------------------------------------------------- #
    def shutdown(self) -> None:
        print("\nshutting down: disabling all motors, closing buses ...")
        for s in self.sides:
            for j in s.joints:
                for m in (j.follower_motor, j.leader_motor):
                    if m is None:
                        continue
                    try:
                        m.disable()
                    except Exception:  # noqa: BLE001
                        pass
                    try:
                        m.close()
                    except Exception:  # noqa: BLE001
                        pass
        for ctrl in self._ctrls.values():
            for fn in ("close_bus", "shutdown", "close"):
                try:
                    getattr(ctrl, fn)()
                except Exception:  # noqa: BLE001
                    pass
        self._ctrls.clear()
        print("disabled and closed.")


# --------------------------------------------------------------------------- #
# CLI parsing helpers
# --------------------------------------------------------------------------- #


def _parse_id_list(spec: str, ordered: bool = False):
    out: List[int] = []
    for tok in (spec or "").replace(" ", "").split(","):
        if not tok:
            continue
        out.append(int(tok, 0))
    if not ordered:
        return set(out)
    seen, uniq = set(), []
    for x in out:
        if x not in seen:
            seen.add(x)
            uniq.append(x)
    return uniq


def _parse_offsets(spec: str) -> Dict[int, float]:
    out: Dict[int, float] = {}
    for tok in (spec or "").replace(" ", "").split(","):
        if not tok:
            continue
        k, _, v = tok.partition("=")
        out[int(k, 0)] = float(v)
    return out


def _parse_pairs(spec: str, swap: bool = False) -> List[tuple]:
    """Parse 'lead:follow,lead:follow' into [(lead, follow), ...]."""
    pairs: List[tuple] = []
    for tok in (spec or "").replace(" ", "").split(","):
        if not tok:
            continue
        lead, sep, follow = tok.partition(":")
        if not sep or not lead or not follow:
            raise ValueError(
                f"bad --pairs entry {tok!r}; expected 'leaderbus:followerbus'")
        if swap:
            lead, follow = follow, lead
        pairs.append((lead, follow))
    return pairs


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Real-time CAN-to-CAN teleop: hand-move each leader arm, the "
                    "paired follower arm mirrors it.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Examples:\n"
               "  ./venv/bin/python teleop.py --dry-run\n"
               "  ./venv/bin/python teleop.py\n"
               "  ./venv/bin/python teleop.py --swap\n"
               "  ./venv/bin/python teleop.py --pairs can0:can2\n"
               "  ./venv/bin/python teleop.py --kp 8 --ramp 2.0\n")
    p.add_argument("--pairs", default=DEFAULT_PAIRS,
                   help="comma list of leader:follower bus pairs "
                        f"(default {DEFAULT_PAIRS}). Buses: 'canX' (SocketCAN), "
                        "'dm:0' (DM adapter), '/dev/ttyACM0' (serial).")
    p.add_argument("--swap", action="store_true",
                   help="swap leader/follower on every pair (the other unit leads)")
    p.add_argument("--dry-run", dest="dry_run", action="store_true",
                   help="open all buses, keep everything LIMP, just print "
                        "leader vs follower positions (no motion). Run this first.")
    p.add_argument("--device-type", dest="device_type",
                   default=os.environ.get("DM_DEVICE_TYPE", "usb2canfd-dual"),
                   help="motorbridge dm device type for dm:* buses (default usb2canfd-dual)")
    p.add_argument("--ids", default=DEFAULT_IDS,
                   help=f"comma list of joint motor ids on every arm (default {DEFAULT_IDS})")
    p.add_argument("--model", default=None,
                   help="force one DaMiao model for all joints "
                        "(default: per-joint from calibrations.json)")
    p.add_argument("--rate", type=float, default=120.0,
                   help="control loop rate in Hz (default 120)")
    p.add_argument("--kp", type=float, default=None,
                   help="MIT position gain for ALL follower joints "
                        "(default: per-joint calibration gain, ~10-12)")
    p.add_argument("--kd", type=float, default=None,
                   help="MIT damping gain for ALL follower joints "
                        "(default: per-joint calibration gain, ~0.45-0.5)")
    p.add_argument("--tau", type=float, default=0.0,
                   help="feed-forward torque on the follower (Nm, default 0)")
    p.add_argument("--ramp", type=float, default=1.5,
                   help="seconds to ease each follower onto its leader pose at "
                        "start (default 1.5)")
    p.add_argument("--max-step", dest="max_step", type=float, default=0.15,
                   help="max follower target change per tick (rad); anti-glitch "
                        "slew limit, 0 disables (default 0.15)")
    p.add_argument("--invert", default="",
                   help="comma list of joint ids to sign-flip on every side "
                        "(for mirror-image mounting), e.g. --invert 3,5")
    p.add_argument("--offset", default="",
                   help="per-joint zero offset rad added after the sign flip, "
                        "e.g. --offset 4=0.05,6=-0.1")
    return p


def main() -> int:
    args = build_parser().parse_args()
    _ensure_driver_lib()
    try:
        import motorbridge  # noqa: F401
    except ImportError as e:
        print(f"could not import motorbridge: {e}\n"
              f"run inside the project venv, e.g. ./venv/bin/python teleop.py ...",
              file=sys.stderr)
        return 2

    teleop = Teleop(args)

    def _sig(_signum, _frame):
        teleop._stop = True
    signal.signal(signal.SIGTERM, _sig)

    try:
        teleop.connect()
    except Exception as e:  # noqa: BLE001
        print(f"\nsetup failed: {e}", file=sys.stderr)
        teleop.shutdown()
        return 1
    teleop.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
