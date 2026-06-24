#!/usr/bin/env python3
"""OpenArm bimanual CAN-to-CAN teleoperation engine.

Hardware on this machine
------------------------
Two OpenArm units, each a left+right pair of arms. Each unit's two arm buses
feed into ONE dual-channel gs_usb CAN-FD adapter, so there are four SocketCAN
interfaces total (two per USB adapter):

    unit A (adapter 1):  can0 = right arm,  can1 = left arm
    unit B (adapter 2):  can2 = right arm,  can3 = left arm

Teleoperation pairs like sides together:

    right side:  can0 <-> can2
    left  side:  can1 <-> can3

One unit is the LEADER (left limp, you back-drive it by hand); the other is the
FOLLOWER (energized, mirrors the leader). A swap flips which unit leads.

Control scheme (unilateral, relative / offset-locked)
-----------------------------------------------------
On start we record ``offset = follower_pos - leader_pos`` for every joint that is
present on BOTH the leader and follower arm, then each tick command
``target = leader_pos + offset`` to the follower in MIT mode (Kp/Kd, tau=0),
clamped to conservative soft limits and slew-limited per tick. The follower
therefore begins exactly where it already is (no jump) and tracks the leader's
*motion* from there.

Redundancy
----------
Each arm has 8 joints (CAN ids 1-8) but some motors may be unplugged. At connect
time every id is probed; a joint only teleoperates if it answers on BOTH the
leader and follower side of its pair. Missing joints are reported and skipped so
the connected joints keep working.

Every CAN access happens on ONE worker thread; HTTP handlers submit commands to
it or read plain shared state under a lock.
"""

from __future__ import annotations

import json
import math
import os
import queue
import subprocess
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional


def _ensure_driver_lib() -> None:
    """Point the DM_Device SDK at the cached runtime (harmless for SocketCAN)."""
    if os.environ.get("MOTOR_DM_DEVICE_LIB"):
        return
    lib = Path.home() / ".cache/motorbridge/dm_device/v1.1.0/linux/x86_64/libdm_device.so"
    if lib.is_file():
        os.environ["MOTOR_DM_DEVICE_LIB"] = str(lib)


# PMAX (rad), VMAX (rad/s), TMAX (Nm) per DaMiao model.
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
}

# OpenArm default per-joint motor models (CAN ids 1-8). Used when a joint has no
# saved calibration to tell us its real model.
OPENARM_MODELS = {1: "8009", 2: "8009", 3: "4340", 4: "4340",
                  5: "4310", 6: "4310", 7: "4310", 8: "4310"}

# Per-model default MIT gains for following. Heavier proximal joints need more.
MODEL_GAINS = {
    "8009": (24.0, 1.0),
    "8006": (24.0, 1.0),
    "4340": (18.0, 0.7),
    "4340P": (18.0, 0.7),
    "4340_v20": (18.0, 0.7),
    "4310": (12.0, 0.45),
    "4310P": (12.0, 0.45),
}
DEFAULT_KP = 12.0
DEFAULT_KD = 0.45

DM_STATUS_TEXT = {
    0: "disabled", 1: "enabled", 8: "overvoltage", 9: "undervoltage",
    10: "overcurrent", 11: "MOSFET over-temp", 12: "rotor over-temp",
    13: "comm loss", 14: "overload",
}
DM_FAULT_MIN = 8  # codes >= 8 are latched hardware faults

JOINT_IDS = list(range(1, 9))

CALIBRATION_STORE_PATH = Path(os.environ.get(
    "DM_CALIBRATION_STORE", str(Path(__file__).with_name("calibrations.json"))))

# CAN-FD bring-up parameters (OpenArm standard).
CAN_BITRATE = int(os.environ.get("OA_CAN_BITRATE", "1000000"))
CAN_DBITRATE = int(os.environ.get("OA_CAN_DBITRATE", "5000000"))

# Default bus topology: each side pairs unit-A bus with unit-B bus.
# (side name, unit-A bus, unit-B bus)
DEFAULT_TOPOLOGY = [
    ("right", "can0", "can2"),
    ("left", "can1", "can3"),
]


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def _is_enobufs(exc: BaseException) -> bool:
    msg = str(exc).lower()
    return "no buffer space" in msg or "os error 105" in msg


def _retry_enobufs(fn, *args, attempts: int = 6, delay: float = 0.0015):
    """Call fn(*args), retrying briefly on the transient SocketCAN ENOBUFS."""
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


# --------------------------------------------------------------------------- #
# SocketCAN bus management (sudo ip link)
# --------------------------------------------------------------------------- #


def can_exists(iface: str) -> bool:
    return Path(f"/sys/class/net/{iface}").exists()


def can_is_up(iface: str) -> Optional[bool]:
    """True/False if the interface exists, None if it doesn't."""
    try:
        with open(f"/sys/class/net/{iface}/flags") as f:
            return bool(int(f.read().strip(), 16) & 0x1)  # IFF_UP
    except FileNotFoundError:
        return None
    except Exception:  # noqa: BLE001
        return None


def bring_up_can(iface: str) -> dict:
    """Bring one SocketCAN-FD interface up via sudo. Returns {ok, output}."""
    if not can_exists(iface):
        return {"ok": False, "output": f"{iface} does not exist (adapter unplugged?)"}
    cmds = [
        ["sudo", "-n", "ip", "link", "set", iface, "down"],
        ["sudo", "-n", "ip", "link", "set", iface, "type", "can",
         "bitrate", str(CAN_BITRATE), "dbitrate", str(CAN_DBITRATE), "fd", "on"],
        ["sudo", "-n", "ip", "link", "set", iface, "txqueuelen", "1000"],
        ["sudo", "-n", "ip", "link", "set", iface, "up"],
    ]
    out_lines: List[str] = []
    for cmd in cmds:
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        except Exception as e:  # noqa: BLE001
            return {"ok": False, "output": f"{' '.join(cmd)}: {e}"}
        msg = (r.stdout + r.stderr).strip()
        if msg:
            out_lines.append(msg)
        # "down" / txqueuelen may fail harmlessly; only the final "up" is fatal.
        if r.returncode != 0 and cmd[-1] in ("up", "on"):
            hint = ""
            if "password" in msg.lower() or "sudo" in msg.lower():
                hint = (" -- passwordless sudo is not configured. Either launch "
                        "with sudo, or add a sudoers rule for 'ip link'.")
            return {"ok": False, "output": (msg or f"failed: {' '.join(cmd)}") + hint}
    return {"ok": can_is_up(iface) is True, "output": "; ".join(out_lines) or "up"}


def bring_down_can(iface: str) -> dict:
    if not can_exists(iface):
        return {"ok": True, "output": f"{iface} absent"}
    try:
        r = subprocess.run(["sudo", "-n", "ip", "link", "set", iface, "down"],
                           capture_output=True, text=True, timeout=10)
    except Exception as e:  # noqa: BLE001
        return {"ok": False, "output": str(e)}
    return {"ok": r.returncode == 0, "output": (r.stdout + r.stderr).strip() or "down"}


# --------------------------------------------------------------------------- #
# Calibration (saved soft limits per joint id)
# --------------------------------------------------------------------------- #


def load_calibrations() -> dict:
    try:
        with open(CALIBRATION_STORE_PATH) as f:
            data = json.load(f)
        return data if isinstance(data, dict) else {}
    except Exception:  # noqa: BLE001
        return {}


def joint_calibration(cals: dict, motor_id: int) -> dict:
    """Aggregate saved info for one joint id across ALL saved arms.

    Soft limits are the CONSERVATIVE overlap (intersection) of every saved range
    for this id, so a follower target can never exceed any arm's known travel.
    """
    recs = [v for k, v in cals.items()
            if isinstance(v, dict) and v.get("motor_id") == motor_id]
    model = None
    lo, hi = -math.inf, math.inf
    for r in recs:
        model = model or r.get("model")
        try:
            lo = max(lo, float(r["pos_min"]))
            hi = min(hi, float(r["pos_max"]))
        except (KeyError, TypeError, ValueError):
            pass
    return {"model": model,
            "pos_min": (None if lo == -math.inf else lo),
            "pos_max": (None if hi == math.inf else hi)}


# --------------------------------------------------------------------------- #
# Per-joint teleop state
# --------------------------------------------------------------------------- #


class Joint:
    def __init__(self, motor_id: int, model: str, kp: float, kd: float,
                 pos_min: float, pos_max: float, pmax: float):
        self.motor_id = motor_id
        self.feedback_id = motor_id + 0x10
        self.model = model
        self.kp = kp
        self.kd = kd
        self.pos_min = pos_min      # saved-calibration soft limits (opt-in)
        self.pos_max = pos_max
        self.pmax = pmax            # full physical range (default clamp)

        self.leader_present = False
        self.follower_present = False
        self.leader_pos = float("nan")
        self.follower_pos = float("nan")
        self.follower_status = -1
        self.offset = 0.0
        self.target = 0.0
        self.faulted = False
        self.fault_text = ""
        self._recover_count = 0
        self._last_recover_ts = 0.0

        # SDK handles (worker thread only).
        self.leader_motor = None
        self.follower_motor = None

    @property
    def active(self) -> bool:
        return self.leader_present and self.follower_present and not self.faulted

    def limits(self, use_cal: bool):
        """Clamp range: saved calibration limits, or the full physical range."""
        if use_cal:
            return self.pos_min, self.pos_max
        return -self.pmax, self.pmax

    def reset_runtime(self):
        self.faulted = False
        self.fault_text = ""
        self._recover_count = 0
        self.offset = 0.0
        self.target = 0.0

    def status(self) -> dict:
        def fmt(x):
            return None if (x != x) else round(x, 4)  # NaN -> None
        return {
            "id": self.motor_id,
            "model": self.model,
            "kp": round(self.kp, 2),
            "kd": round(self.kd, 3),
            "leader_present": self.leader_present,
            "follower_present": self.follower_present,
            "active": self.active,
            "leader_pos": fmt(self.leader_pos),
            "follower_pos": fmt(self.follower_pos),
            "follower_status": self.follower_status,
            "follower_status_text": DM_STATUS_TEXT.get(self.follower_status, ""),
            "faulted": self.faulted,
            "fault_text": self.fault_text,
            "pos_min": round(self.pos_min, 4),
            "pos_max": round(self.pos_max, 4),
        }


class Side:
    """One left/right teleop pairing of two physical arm buses."""

    def __init__(self, name: str, bus_a: str, bus_b: str):
        self.name = name
        self.bus_a = bus_a  # unit A arm bus
        self.bus_b = bus_b  # unit B arm bus
        self.joints: List[Joint] = []

    def leader_bus(self, swapped: bool) -> str:
        return self.bus_b if swapped else self.bus_a

    def follower_bus(self, swapped: bool) -> str:
        return self.bus_a if swapped else self.bus_b


# --------------------------------------------------------------------------- #
# CAN feedback helpers (batched, one pass per bus)
# --------------------------------------------------------------------------- #


def _refresh_bus(ctrl, motors, which: str) -> None:
    """Refresh leader or follower positions for all joints on one bus at once.

    ``motors`` is a list of ``(joint, motor_handle)`` tuples on the same bus.
    """
    pairs = []
    for j in motors:
        joint, m = j
        try:
            m.request_feedback()
        except Exception:  # noqa: BLE001
            pass
        pairs.append((joint, m))
    for _ in range(len(pairs) + 2):
        try:
            ctrl.poll_feedback_once()
        except Exception:  # noqa: BLE001
            break
    for joint, m in pairs:
        try:
            st = m.get_state()
        except Exception:  # noqa: BLE001
            st = None
        if st is None:
            continue
        if which == "leader":
            joint.leader_pos = float(st.pos)
        else:
            joint.follower_pos = float(st.pos)
            joint.follower_status = st.status_code


# --------------------------------------------------------------------------- #
# Command unit for the worker thread
# --------------------------------------------------------------------------- #


class _Cmd:
    __slots__ = ("fn", "event", "result", "exc")

    def __init__(self, fn):
        self.fn = fn
        self.event = threading.Event()
        self.result = None
        self.exc: Optional[BaseException] = None


class OpenArmTeleop:
    """Four-bus OpenArm teleop service driven by a single worker thread.

    Teleop phases:
      idle      - buses may be open, nothing energized, no streaming
      monitor   - both arms LIMP; read & display leader vs follower (dry run)
      following - followers energized in MIT, mirror their leaders
    E-stop is a latch that forces every motor disabled until cleared.
    """

    def __init__(self, rate_hz: float = 120.0):
        _ensure_driver_lib()
        self.rate_hz = rate_hz
        self.lock = threading.RLock()

        cals = load_calibrations()
        self.sides: List[Side] = []
        for name, bus_a, bus_b in DEFAULT_TOPOLOGY:
            s = Side(name, bus_a, bus_b)
            s.joints = self._build_joints(cals)
            self.sides.append(s)

        self.swapped = False
        self.crossed = False          # swap which follower bus pairs each side
        self.use_cal_limits = False   # clamp to saved calibration vs full range
        self.phase = "idle"           # idle | monitor | following
        self.estopped = False
        self.connected = False
        self.message = "not connected"
        self.fault: Optional[str] = None

        # Tunables (worker reads under lock).
        self.gain_scale = 1.0         # multiplies every joint's kp/kd
        self.max_step = 0.15          # rad per tick slew clamp (0 = off)
        self.tau = 0.0                # feed-forward torque
        self.ramp_s = 1.5             # ease-in seconds on follow start

        # Worker-thread-owned device handles.
        self._ctrls: Dict[str, object] = {}   # bus -> Controller
        self._ramp_tick = 0
        self._ramp_ticks = 1

        self._q: "queue.Queue[_Cmd]" = queue.Queue()
        self._stop = False
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------ #
    # Construction helpers
    # ------------------------------------------------------------------ #
    def _build_joints(self, cals: dict) -> List[Joint]:
        joints = []
        for mid in JOINT_IDS:
            c = joint_calibration(cals, mid)
            model = c["model"] or OPENARM_MODELS.get(mid, "4310")
            pmax = DAMIAO_MODEL_LIMITS.get(model, (12.5, 30.0, 10.0))[0]
            pos_min = c["pos_min"] if c["pos_min"] is not None else -pmax
            pos_max = c["pos_max"] if c["pos_max"] is not None else pmax
            if pos_max <= pos_min:
                pos_min, pos_max = -pmax, pmax
            kp, kd = MODEL_GAINS.get(model, (DEFAULT_KP, DEFAULT_KD))
            joints.append(Joint(mid, model, kp, kd, pos_min, pos_max, pmax))
        return joints

    def _all_buses(self) -> List[str]:
        out = []
        for s in self.sides:
            for b in (s.bus_a, s.bus_b):
                if b not in out:
                    out.append(b)
        return out

    # ------------------------------------------------------------------ #
    # Command submission (HTTP threads)
    # ------------------------------------------------------------------ #
    def _submit(self, fn, timeout: float = 30.0):
        cmd = _Cmd(fn)
        self._q.put(cmd)
        if not cmd.event.wait(timeout):
            raise TimeoutError("device command timed out")
        if cmd.exc is not None:
            raise cmd.exc
        return cmd.result

    # ------------------------------------------------------------------ #
    # Worker thread
    # ------------------------------------------------------------------ #
    def _worker(self):
        period = 1.0 / self.rate_hz
        while not self._stop:
            t0 = time.time()
            while True:
                try:
                    cmd = self._q.get_nowait()
                except queue.Empty:
                    break
                try:
                    cmd.result = cmd.fn()
                except BaseException as e:  # noqa: BLE001
                    cmd.exc = e
                finally:
                    cmd.event.set()
            try:
                self._tick()
            except Exception as e:  # noqa: BLE001
                with self.lock:
                    self.fault = str(e)
            dt = time.time() - t0
            time.sleep(max(0.0, period - dt))
        self._teardown()

    def _tick(self):
        with self.lock:
            phase = self.phase
        if phase == "idle" or not self._ctrls:
            return
        if phase == "monitor":
            self._tick_monitor()
        elif phase == "following":
            self._tick_following()

    def _tick_monitor(self):
        for s in self.sides:
            lb = s.leader_bus(self.swapped)
            fb = s.follower_bus(self.swapped)
            lead = [(j, j.leader_motor) for j in s.joints
                    if j.leader_present and j.leader_motor is not None]
            foll = [(j, j.follower_motor) for j in s.joints
                    if j.follower_present and j.follower_motor is not None]
            if lb in self._ctrls and lead:
                _refresh_bus(self._ctrls[lb], lead, "leader")
            if fb in self._ctrls and foll:
                _refresh_bus(self._ctrls[fb], foll, "follower")

    def _tick_following(self):
        with self.lock:
            scale = self.gain_scale
            max_step = self.max_step
            tau = self.tau
            use_cal = self.use_cal_limits
        alpha = 1.0 if self._ramp_tick >= self._ramp_ticks \
            else (self._ramp_tick + 1) / self._ramp_ticks
        for s in self.sides:
            lb = s.leader_bus(self.swapped)
            fb = s.follower_bus(self.swapped)
            lead = [(j, j.leader_motor) for j in s.joints
                    if j.active and j.leader_motor is not None]
            if lb in self._ctrls and lead:
                _refresh_bus(self._ctrls[lb], lead, "leader")
            fctrl = self._ctrls.get(fb)
            if fctrl is None:
                continue
            socketcan = True  # all OpenArm buses are SocketCAN
            for j in s.joints:
                if not j.active or j.follower_motor is None:
                    continue
                lo, hi = j.limits(use_cal)
                if math.isnan(j.leader_pos):
                    desired = j.target
                else:
                    live = _clamp(j.leader_pos + j.offset, lo, hi)
                    if alpha < 1.0:
                        desired = j.target + alpha * (live - j.target)
                    else:
                        desired = live
                if max_step > 0:
                    desired = _clamp(desired, j.target - max_step, j.target + max_step)
                j.target = _clamp(desired, lo, hi)
                kp = j.kp * scale
                kd = j.kd * scale
                try:
                    if socketcan:
                        _retry_enobufs(j.follower_motor.send_mit,
                                       j.target, 0.0, kp, kd, tau)
                    else:
                        j.follower_motor.send_mit(j.target, 0.0, kp, kd, tau)
                except Exception as e:  # noqa: BLE001
                    j.fault_text = f"command error: {e}"
            # Drain follower feedback + supervise.
            foll = [(j, j.follower_motor) for j in s.joints
                    if j.follower_present and j.follower_motor is not None]
            for _ in range(len(foll) + 2):
                try:
                    fctrl.poll_feedback_once()
                except Exception:  # noqa: BLE001
                    break
            for j, m in foll:
                try:
                    st = m.get_state()
                except Exception:  # noqa: BLE001
                    st = None
                if st is not None:
                    j.follower_pos = float(st.pos)
                    j.follower_status = st.status_code
                    self._supervise(j, st.status_code)
        self._ramp_tick += 1

    def _supervise(self, j: Joint, sc: int) -> None:
        if sc == 13 and not j.faulted:
            # Comm loss is recoverable (a missed control frame), NOT a hardware
            # fault -- clear and re-energize instead of latching the joint off.
            now = time.time()
            if now - j._last_recover_ts < 0.25:
                return
            j._last_recover_ts = now
            j._recover_count += 1
            if j._recover_count > 40:
                j.faulted = True
                j.fault_text = "persistent comm loss -- check CAN wiring/power"
                return
            self._clean_enable(j.follower_motor)
            return
        if sc >= DM_FAULT_MIN:
            if not j.faulted:
                j.faulted = True
                j.fault_text = f"hardware fault: {DM_STATUS_TEXT.get(sc, f'code {sc}')}"
                try:
                    j.follower_motor.disable()
                except Exception:  # noqa: BLE001
                    pass
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
                return
            self._clean_enable(j.follower_motor)

    # ------------------------------------------------------------------ #
    # Device operations (worker thread)
    # ------------------------------------------------------------------ #
    def _open_bus(self, bus: str):
        from motorbridge import Controller
        if bus in self._ctrls:
            return self._ctrls[bus]
        up = can_is_up(bus)
        if up is None:
            raise RuntimeError(f"{bus} not found (adapter unplugged?)")
        if up is False:
            raise RuntimeError(f"{bus} is DOWN -- bring it up first")
        ctrl = Controller.from_socketcanfd(bus)
        self._ctrls[bus] = ctrl
        return ctrl

    @staticmethod
    def _probe(ctrl, motor) -> bool:
        """Return True if a motor answers on the bus."""
        try:
            motor.request_feedback()
        except Exception:  # noqa: BLE001
            pass
        for _ in range(10):
            try:
                ctrl.poll_feedback_once()
            except Exception:  # noqa: BLE001
                break
            try:
                if motor.get_state() is not None:
                    return True
            except Exception:  # noqa: BLE001
                pass
            time.sleep(0.003)
        return False

    @staticmethod
    def _clean_enable(motor) -> None:
        try:
            motor.disable()
        except Exception:  # noqa: BLE001
            pass
        time.sleep(0.02)
        try:
            motor.clear_error()
        except Exception:  # noqa: BLE001
            pass
        try:
            from motorbridge import Mode
            motor.ensure_mode(Mode.MIT)
        except Exception:  # noqa: BLE001
            pass
        try:
            _retry_enobufs(motor.enable)
        except Exception:  # noqa: BLE001
            pass

    def _do_connect(self) -> dict:
        """Open every up bus, add all 8 motors per arm, probe presence."""
        # Start clean.
        self._do_stop()
        self._teardown()
        from motorbridge import Controller  # noqa: F401  (import check)

        opened, errors = [], []
        for bus in self._all_buses():
            try:
                self._open_bus(bus)
                opened.append(bus)
            except Exception as e:  # noqa: BLE001
                errors.append(f"{bus}: {e}")

        # Add motors on both buses of each side and probe which answer.
        for s in self.sides:
            for j in s.joints:
                j.leader_present = False
                j.follower_present = False
                j.leader_motor = None
                j.follower_motor = None
                j.reset_runtime()
            for bus, role in ((s.bus_a, "a"), (s.bus_b, "b")):
                ctrl = self._ctrls.get(bus)
                if ctrl is None:
                    continue
                for j in s.joints:
                    try:
                        m = ctrl.add_damiao_motor(j.motor_id, j.feedback_id, j.model)
                    except Exception:  # noqa: BLE001
                        continue
                    try:
                        # Watchdog OFF: DaMiao motors trip to "comm loss"
                        # (status 13) if they don't get a control frame within
                        # the timeout. We send nothing while limp/monitoring, so
                        # any non-zero watchdog spuriously faults idle motors.
                        m.set_can_timeout_ms(0)
                    except Exception:  # noqa: BLE001
                        pass
                    present = self._probe(ctrl, m)
                    if role == "a":
                        j._motor_a = m
                    else:
                        j._motor_b = m
                    if present:
                        setattr(j, f"_present_{role}", True)
        # Resolve leader/follower handles from a/b per the swap state.
        self._assign_roles()
        with self.lock:
            self.connected = bool(opened)
            present_total = sum(1 for s in self.sides for j in s.joints
                                if j.leader_present or j.follower_present)
            if not opened:
                self.message = "no buses available: " + ("; ".join(errors) or "none up")
            else:
                self.message = (f"connected on {', '.join(opened)}; "
                                f"{present_total} motors detected")
                if errors:
                    self.message += " (" + "; ".join(errors) + ")"
            self.fault = None
        return self.status()

    def _assign_roles(self):
        """Map each joint's a/b motor handles to leader/follower per swap."""
        for s in self.sides:
            for j in s.joints:
                ma = getattr(j, "_motor_a", None)
                mb = getattr(j, "_motor_b", None)
                pa = getattr(j, "_present_a", False)
                pb = getattr(j, "_present_b", False)
                if self.swapped:
                    j.leader_motor, j.follower_motor = mb, ma
                    j.leader_present, j.follower_present = pb, pa
                else:
                    j.leader_motor, j.follower_motor = ma, mb
                    j.leader_present, j.follower_present = pa, pb

    def _do_monitor(self) -> dict:
        if not self._ctrls:
            raise RuntimeError("connect first")
        # Everything limp.
        self._disable_all_followers()
        with self.lock:
            self.phase = "monitor"
            self.message = "monitor (limp): move a leader joint, watch its follower"
        return self.status()

    def _do_follow(self) -> dict:
        with self.lock:
            if self.estopped:
                raise RuntimeError("E-STOP latched -- clear it first")
        if not self._ctrls:
            raise RuntimeError("connect first")
        # Fresh read of both sides to lock offsets at current poses.
        for s in self.sides:
            lb, fb = s.leader_bus(self.swapped), s.follower_bus(self.swapped)
            lead = [(j, j.leader_motor) for j in s.joints
                    if j.leader_present and j.leader_motor is not None]
            foll = [(j, j.follower_motor) for j in s.joints
                    if j.follower_present and j.follower_motor is not None]
            if lb in self._ctrls and lead:
                _refresh_bus(self._ctrls[lb], lead, "leader")
            if fb in self._ctrls and foll:
                _refresh_bus(self._ctrls[fb], foll, "follower")

        active_total = 0
        for s in self.sides:
            for j in s.joints:
                j.reset_runtime()
                if not (j.leader_present and j.follower_present):
                    continue
                if math.isnan(j.leader_pos) or math.isnan(j.follower_pos):
                    j.faulted = True
                    j.fault_text = "no position read at start"
                    continue
                # Offset-lock: follower starts where it is, mirrors leader motion.
                j.offset = j.follower_pos - j.leader_pos
                j.target = j.follower_pos
                active_total += 1
        if active_total == 0:
            raise RuntimeError(
                "no joint is present on BOTH leader and follower -- check wiring")
        # Energize followers.
        for s in self.sides:
            for j in s.joints:
                if j.active and j.follower_motor is not None:
                    self._clean_enable(j.follower_motor)
        self._ramp_tick = 0
        self._ramp_ticks = max(1, int(self.ramp_s * self.rate_hz))
        with self.lock:
            self.phase = "following"
            self.message = f"following @ {self.rate_hz:.0f} Hz; {active_total} joints active"
        return self.status()

    def _disable_all_followers(self):
        for s in self.sides:
            for j in s.joints:
                if j.follower_motor is not None:
                    try:
                        j.follower_motor.disable()
                    except Exception:  # noqa: BLE001
                        pass
                if j.leader_motor is not None:
                    try:
                        j.leader_motor.disable()  # leaders stay limp anyway
                    except Exception:  # noqa: BLE001
                        pass

    def _do_stop(self) -> dict:
        self._disable_all_followers()
        with self.lock:
            if self.phase != "idle":
                self.message = "stopped (idle, all limp)"
            self.phase = "idle"
        return self.status()

    def _do_estop(self) -> dict:
        with self.lock:
            self.estopped = True
            self.phase = "idle"
            self.message = "E-STOP latched -- all motors disabled"
        self._disable_all_followers()
        return self.status()

    def _do_clear_estop(self) -> dict:
        with self.lock:
            self.estopped = False
            self.message = "E-STOP cleared (idle)"
        return self.status()

    def _do_swap(self) -> dict:
        # Must be idle/limp to swap roles safely.
        self._do_stop()
        with self.lock:
            self.swapped = not self.swapped
        self._assign_roles()
        with self.lock:
            self.message = f"swapped: leaders now on {'unit B' if self.swapped else 'unit A'}"
        return self.status()

    def _do_cross(self) -> dict:
        """Swap which follower bus pairs with each side (fixes L/R crossed).

        The two physical units' arms can enumerate so that the leader's left
        arm ends up paired with the follower's right arm. Swapping the two
        sides' unit-B buses re-pairs them like-for-like. Rebuilds the bus if we
        were connected.
        """
        self._do_stop()
        was = self.connected
        if len(self.sides) >= 2:
            self.sides[0].bus_b, self.sides[1].bus_b = \
                self.sides[1].bus_b, self.sides[0].bus_b
        self.crossed = not self.crossed
        if was:
            self._do_connect()
        with self.lock:
            self.message = "L/R sides swapped" + (" (reconnected)" if was else "")
        return self.status()

    def _do_identify(self, bus: str) -> dict:
        """Briefly wiggle the lowest connected motor on a bus to find it."""
        ctrl = self._ctrls.get(bus)
        if ctrl is None:
            raise RuntimeError(f"{bus} not open")
        target_joint = None
        for s in self.sides:
            for j in s.joints:
                m = (getattr(j, "_motor_a", None) if bus == s.bus_a
                     else getattr(j, "_motor_b", None) if bus == s.bus_b else None)
                present = (getattr(j, "_present_a", False) if bus == s.bus_a
                           else getattr(j, "_present_b", False))
                if m is not None and present:
                    target_joint = (j, m)
                    break
            if target_joint:
                break
        if not target_joint:
            raise RuntimeError(f"no connected motor on {bus}")
        j, m = target_joint
        st = m.get_state()
        base = float(st.pos) if st is not None else 0.0
        self._clean_enable(m)
        try:
            for tgt in (base + 0.12, base - 0.12, base + 0.12, base):
                t_end = time.time() + 0.35
                while time.time() < t_end:
                    _retry_enobufs(m.send_mit, tgt, 0.0, 5.0, 0.6, 0.0)
                    time.sleep(0.02)
        finally:
            try:
                m.disable()
            except Exception:  # noqa: BLE001
                pass
        return {"bus": bus, "motor_id": j.motor_id}

    def _teardown(self):
        for s in self.sides:
            for j in s.joints:
                for m in (getattr(j, "_motor_a", None), getattr(j, "_motor_b", None)):
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
                j._motor_a = None
                j._motor_b = None
                j.leader_motor = None
                j.follower_motor = None
        for ctrl in self._ctrls.values():
            for fn in ("close_bus", "shutdown", "close"):
                try:
                    getattr(ctrl, fn)()
                except Exception:  # noqa: BLE001
                    pass
        self._ctrls.clear()

    # ------------------------------------------------------------------ #
    # Public API (HTTP threads)
    # ------------------------------------------------------------------ #
    def connect(self):
        return self._submit(self._do_connect, timeout=60.0)

    def disconnect(self):
        def run():
            self._do_stop()
            self._teardown()
            with self.lock:
                self.connected = False
                self.message = "disconnected"
            return self.status()
        return self._submit(run, timeout=30.0)

    def monitor(self):
        return self._submit(self._do_monitor)

    def follow(self):
        return self._submit(self._do_follow, timeout=30.0)

    def stop(self):
        return self._submit(self._do_stop)

    def estop(self):
        return self._submit(self._do_estop, timeout=10.0)

    def clear_estop(self):
        return self._submit(self._do_clear_estop)

    def swap(self):
        return self._submit(self._do_swap, timeout=30.0)

    def cross(self):
        return self._submit(self._do_cross, timeout=60.0)

    def identify(self, bus: str):
        return self._submit(lambda: self._do_identify(bus), timeout=20.0)

    def set_params(self, gain_scale=None, max_step=None, tau=None, ramp_s=None,
                   use_cal_limits=None):
        with self.lock:
            if gain_scale is not None:
                self.gain_scale = _clamp(float(gain_scale), 0.1, 3.0)
            if max_step is not None:
                self.max_step = _clamp(float(max_step), 0.0, 1.0)
            if tau is not None:
                self.tau = _clamp(float(tau), -5.0, 5.0)
            if ramp_s is not None:
                self.ramp_s = _clamp(float(ramp_s), 0.0, 10.0)
            if use_cal_limits is not None:
                self.use_cal_limits = bool(use_cal_limits)
        return self.status()

    # ---- bus management (HTTP threads; sudo runs off the worker) ------ #
    def bus_status(self) -> List[dict]:
        out = []
        for s in self.sides:
            for bus, unit in ((s.bus_a, "A"), (s.bus_b, "B")):
                if any(b["name"] == bus for b in out):
                    continue
                up = can_is_up(bus)
                if self.swapped:
                    role = "follower" if unit == "A" else "leader"
                else:
                    role = "leader" if unit == "A" else "follower"
                out.append({
                    "name": bus,
                    "unit": unit,
                    "side": s.name,
                    "role": role,
                    "exists": can_exists(bus),
                    "up": (up is True),
                    "open": bus in self._ctrls,
                })
        return out

    def bring_up_all(self) -> dict:
        results = {b: bring_up_can(b) for b in self._all_buses()}
        ok = all(r["ok"] for r in results.values())
        with self.lock:
            ups = [b for b, r in results.items() if r["ok"]]
            self.message = (f"buses up: {', '.join(ups)}" if ups
                            else "no buses came up")
        return {"ok": ok, "results": results}

    def bring_up_one(self, bus: str) -> dict:
        return bring_up_can(bus)

    def bring_down_one(self, bus: str) -> dict:
        return bring_down_can(bus)

    # ------------------------------------------------------------------ #
    def status(self) -> dict:
        with self.lock:
            sides = []
            for s in self.sides:
                joints = [j.status() for j in s.joints]
                active = [j["id"] for j in joints if j["active"]]
                lead_only = [j["id"] for j in joints
                             if j["leader_present"] and not j["follower_present"]]
                foll_only = [j["id"] for j in joints
                             if j["follower_present"] and not j["leader_present"]]
                missing = [j["id"] for j in joints
                           if not j["leader_present"] and not j["follower_present"]]
                faulted = [j["id"] for j in joints if j["faulted"]]
                sides.append({
                    "name": s.name,
                    "leader_bus": s.leader_bus(self.swapped),
                    "follower_bus": s.follower_bus(self.swapped),
                    "joints": joints,
                    "active": active,
                    "leader_only": lead_only,
                    "follower_only": foll_only,
                    "missing": missing,
                    "faulted": faulted,
                })
            return {
                "ok": True,
                "phase": self.phase,
                "connected": self.connected,
                "estopped": self.estopped,
                "swapped": self.swapped,
                "crossed": self.crossed,
                "rate_hz": self.rate_hz,
                "message": self.message,
                "fault": self.fault,
                "params": {
                    "gain_scale": round(self.gain_scale, 2),
                    "max_step": round(self.max_step, 3),
                    "tau": round(self.tau, 2),
                    "ramp_s": round(self.ramp_s, 2),
                    "use_cal_limits": self.use_cal_limits,
                },
                "buses": self.bus_status(),
                "sides": sides,
            }

    def shutdown(self):
        self._stop = True
