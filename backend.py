"""
Backend service for one or more DaMiao DM-series motors sharing a single
DM-USB2FDCAN adapter (USB mode), using the motorbridge DM_Device transport.

Threading model (important):
  The DaMiao DM_Device SDK is backed by libusb and is *thread-affine* -- the USB
  handle must only ever be touched by the thread that created it. So ALL SDK
  calls (open, scan, enable, control, feedback, close) run on ONE dedicated
  worker thread. HTTP handlers never call the SDK directly; they either:
    - submit a command to the worker and wait for the result (connect, scan,
      enable, disable, set_zero, estop), or
    - update plain shared state under a lock (set_targets, set_mode, status),
      which the worker reads on its next control tick.

  The same worker thread also runs the periodic control loop: for every enabled
  motor it continuously re-sends that motor's active setpoint (DM motors fault
  out if commands stop) and pulls fresh feedback into per-motor snapshots.

Multi-motor model:
  All motors live on ONE shared Controller (one CAN channel / one bus). Each
  motor has its own setpoints, mode, enable flag, limits and telemetry, so they
  are controlled fully independently. Because the motorbridge model is fixed at
  add-time and re-adding a live id fails, changing the motor roster (add /
  remove / model change from autodetect) rebuilds the controller and re-adds
  every known motor.
"""

from __future__ import annotations

import json
import queue
import os
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional

from motorbridge import Controller, Mode


# PMAX (rad), VMAX (rad/s), TMAX (Nm) per model -- used for auto-detection.
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

MODE_MAP = {
    "mit": Mode.MIT,
    "pos_vel": Mode.POS_VEL,
    "vel": Mode.VEL,
    "force_pos": Mode.FORCE_POS,
}

# DaMiao feedback status/error codes (low nibble of the MIT feedback frame).
DM_STATUS_TEXT = {
    0: "disabled",
    1: "enabled",
    8: "overvoltage",
    9: "undervoltage",
    10: "overcurrent",
    11: "MOSFET over-temperature",
    12: "rotor over-temperature",
    13: "communication loss",
    14: "overload",
}
_UNSET = object()
# Codes >= 8 are hardware faults: the motor latched itself off and must NOT be
# auto-re-enabled (re-energizing into an over-current/temp condition is unsafe).
DM_FAULT_MIN = 8

# Conservative defaults for automatic hardstop calibration. The routine sweeps
# one motor at a time, watches for a sustained low-velocity stall at each end,
# moves to the measured midpoint, then writes that midpoint as the motor zero.
def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


CAL_SPEED_RAD_S = _env_float("CAL_SPEED_RAD_S", 1.25)
CAL_MAX_SPEED_RAD_S = _env_float("CAL_MAX_SPEED_RAD_S", 2.00)
CAL_STALL_VEL_RAD_S = 0.04
CAL_STALL_DWELL_S = 0.45
CAL_MIN_SWEEP_S = 0.50
CAL_MAX_SWEEP_S = 90.0
CAL_ABSOLUTE_SWEEP_S = _env_float("CAL_ABSOLUTE_SWEEP_S", 180.0)
CAL_MIN_SPAN_RAD = _env_float("CAL_MIN_SPAN_RAD", 0.75)
CAL_CENTER_TOL_RAD = 0.06
CAL_MIT_KP = _env_float("CAL_MIT_KP", 10.0)
CAL_MIT_KD = _env_float("CAL_MIT_KD", 0.45)
CAL_MIT_TARGET_LEAD_RAD = _env_float("CAL_MIT_TARGET_LEAD_RAD", 0.18)
CAL_MIT_MAX_KP = _env_float("CAL_MIT_MAX_KP", 22.0)
CAL_MIT_MAX_TARGET_LEAD_RAD = _env_float("CAL_MIT_MAX_TARGET_LEAD_RAD", 0.40)
CAL_MIT_MAX_TAU_FF = _env_float("CAL_MIT_MAX_TAU_FF", 0.35)
CAL_MIT_MAX_EFFORT_FRAC = _env_float("CAL_MIT_MAX_EFFORT_FRAC", 0.55)
CAL_J4_CENTER_LEAD_RAD = _env_float("CAL_J4_CENTER_LEAD_RAD", 1.00)
CAL_J4_CENTER_TAU_FF = _env_float("CAL_J4_CENTER_TAU_FF", 1.20)
CAL_J4_CENTER_KP = _env_float("CAL_J4_CENTER_KP", CAL_MIT_MAX_KP)
# J4 only auto-finds its BACK hardstop; the forward extent is taken from the
# saved calibration span (or this default if nothing has been saved yet), so we
# never drive J4 into the front stop or require a manual mark.
CAL_J4_DEFAULT_SPAN_RAD = _env_float("CAL_J4_DEFAULT_SPAN_RAD", 1.93904)
CAL_PROGRESS_EPS_RAD = 0.01
CAL_EFFORT_STAGE_SCALES = (1.0, 1.35, 1.75, 2.20)
CAL_JOINT_PROFILES = {
    # J4 can need more breakaway effort to reach the real mechanical stops.
    4: {
        "speed": _env_float("CAL_J4_SPEED_RAD_S", 1.50),
        "kp": _env_float("CAL_J4_MIT_KP", 12.0),
        "kd": _env_float("CAL_J4_MIT_KD", 0.50),
        "lead": _env_float("CAL_J4_MIT_TARGET_LEAD_RAD", 0.25),
    },
}
CALIBRATION_STORE_PATH = Path(os.environ.get(
    "DM_CALIBRATION_STORE",
    str(Path(__file__).with_name("calibrations.json"))))


def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def _best_model(pmax: float, vmax: float, tmax: float) -> Optional[str]:
    best, best_err = None, 1e18
    for model, (mp, mv, mt) in DAMIAO_MODEL_LIMITS.items():
        err = (mp - pmax) ** 2 + (mv - vmax) ** 2 + (mt - tmax) ** 2
        if err < best_err:
            best, best_err = model, err
    return best


def _blank_state() -> dict:
    return {"pos": 0.0, "vel": 0.0, "torq": 0.0,
            "t_mos": 0.0, "t_rotor": 0.0, "status_code": 0,
            "status_text": "—", "online": False, "ts": 0.0}


def _blank_sp() -> dict:
    return {"pos": 0.0, "vel": 0.0, "kp": 0.0, "kd": 0.0,
            "tau": 0.0, "vlim": 2.0, "ratio": 1.0}


def _blank_calibration() -> dict:
    return {"active": False, "phase": "idle", "message": "",
            "error": None, "result": None, "saved_at": None}


class _Cmd:
    """A unit of work to run on the device worker thread."""
    __slots__ = ("fn", "event", "result", "exc")

    def __init__(self, fn):
        self.fn = fn
        self.event = threading.Event()
        self.result = None
        self.exc: Optional[BaseException] = None


class _MotorHandle:
    """Per-motor configuration, setpoints and telemetry.

    The SDK ``_motor`` handle is ONLY touched by the worker thread; every other
    field is plain shared state guarded by the manager's lock.
    """

    def __init__(self, motor_id: int, feedback_id: int, model: str,
                 channel: str = "0"):
        self.motor_id = motor_id
        self.feedback_id = feedback_id
        self.model = model
        self.channel = str(channel)
        self.limits = DAMIAO_MODEL_LIMITS.get(model, (12.5, 30.0, 10.0))
        self.enabled = False
        self.mode = "mit"
        self._mode_applied: Optional[str] = None
        # Last mode actually written to the motor's mode register (worker-thread
        # only). Used to re-assert MIT when a prior non-MIT write (e.g. pos_vel
        # calibration centering) left the register in another mode.
        self._mode_register: Optional[str] = None
        self.max_power = False
        # Latching emergency stop: once set, the worker holds the motor disabled
        # every tick and refuses to send any motion command until the user
        # explicitly re-enables (which clears the latch).
        self.estopped = False
        self.fault: Optional[str] = None
        self.sp = _blank_sp()
        self.state = _blank_state()
        self.calibration = _blank_calibration()
        self.pos_min: Optional[float] = None
        self.pos_max: Optional[float] = None
        self._motor = None  # SDK handle (worker-thread only)
        # Auto-recovery bookkeeping (worker-thread only).
        self._recover_count = 0
        self._last_recover_ts = 0.0

    def status(self) -> dict:
        online = self.state["online"] and (time.time() - self.state["ts"] < 0.5)
        pmax, vmax, tmax = self.limits
        pos_min = -pmax if self.pos_min is None else self.pos_min
        pos_max = pmax if self.pos_max is None else self.pos_max
        return {
            "motor_id": self.motor_id,
            "feedback_id": self.feedback_id,
            "channel": self.channel,
            "model": self.model,
            "enabled": self.enabled,
            "estopped": self.estopped,
            "mode": self.mode,
            "max_power": self.max_power,
            "fault": self.fault,
            "limits": {
                "pmax": pmax, "vmax": vmax, "tmax": tmax,
                "pos_min": round(pos_min, 5),
                "pos_max": round(pos_max, 5),
                "calibrated": self.pos_min is not None and self.pos_max is not None,
            },
            "setpoints": dict(self.sp),
            "state": dict(self.state, online=online),
            "calibration": dict(self.calibration),
        }


class MotorService:
    """Manages a set of DM motors on one shared CAN bus / worker thread."""

    def __init__(self, device_type: str = "usb2canfd-dual", channel: str = "0",
                 model: str = "4310", rate_hz: float = 50.0):
        self.device_type = device_type
        self.channel = str(channel)
        self.default_model = model
        self.rate_hz = rate_hz

        # Lock protects only the plain shared state below (NOT the SDK handle).
        self.lock = threading.RLock()
        self.fault: Optional[str] = None
        # Motors are keyed by (channel, motor_id) so the two arms on a dual
        # adapter can share overlapping CAN ids (e.g. joint 5 on both arms).
        self.motors: "Dict[tuple, _MotorHandle]" = {}
        self._calibration_store = self._load_calibration_store()

        # One device handle per CAN channel -- ONLY touched by the worker thread.
        self._ctrls: "Dict[str, Controller]" = {}

        self._q: "queue.Queue[_Cmd]" = queue.Queue()
        self._stop = False
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------ #
    # Command submission (called from HTTP threads)
    # ------------------------------------------------------------------ #
    def _submit(self, fn, timeout: float = 20.0):
        cmd = _Cmd(fn)
        self._q.put(cmd)
        if not cmd.event.wait(timeout):
            raise TimeoutError("device command timed out")
        if cmd.exc is not None:
            raise cmd.exc
        return cmd.result

    @staticmethod
    def _key(channel, motor_id) -> tuple:
        return (str(channel), int(motor_id))

    @staticmethod
    def _cal_key(channel, motor_id) -> str:
        return f"{channel}:{motor_id}"

    def _ctrl_for(self, mh: "_MotorHandle") -> Optional[Controller]:
        return self._ctrls.get(str(mh.channel))

    def _require(self, channel, motor_id) -> _MotorHandle:
        mh = self.motors.get(self._key(channel, motor_id))
        if mh is None:
            raise RuntimeError(
                f"motor {motor_id} on channel {channel} not connected")
        return mh

    def _load_calibration_store(self) -> dict:
        try:
            with CALIBRATION_STORE_PATH.open("r", encoding="utf-8") as f:
                data = json.load(f)
            return data if isinstance(data, dict) else {}
        except FileNotFoundError:
            return {}
        except Exception as e:  # noqa: BLE001
            self.fault = f"could not read calibration store: {e}"
            return {}

    def _write_calibration_store(self):
        CALIBRATION_STORE_PATH.parent.mkdir(parents=True, exist_ok=True)
        tmp = CALIBRATION_STORE_PATH.with_suffix(CALIBRATION_STORE_PATH.suffix + ".tmp")
        with tmp.open("w", encoding="utf-8") as f:
            json.dump(self._calibration_store, f, indent=2, sort_keys=True)
            f.write("\n")
        tmp.replace(CALIBRATION_STORE_PATH)

    def _apply_saved_calibration(self, mh: _MotorHandle):
        saved = self._calibration_store.get(self._cal_key(mh.channel, mh.motor_id))
        if not isinstance(saved, dict):
            # Fall back to a legacy single-arm key (plain motor id).
            saved = self._calibration_store.get(str(mh.motor_id))
        if not isinstance(saved, dict):
            return
        try:
            pos_min = float(saved["pos_min"])
            pos_max = float(saved["pos_max"])
        except (KeyError, TypeError, ValueError):
            return
        if pos_max <= pos_min:
            return
        with self.lock:
            mh.pos_min = pos_min
            mh.pos_max = pos_max
            mh.calibration = dict(
                mh.calibration,
                phase="saved",
                message="loaded saved midpoint calibration",
                result=saved.get("result"),
                saved_at=saved.get("saved_at"),
            )

    def _save_calibration(self, mh: _MotorHandle, result: dict, method: str):
        record = {
            "motor_id": mh.motor_id,
            "feedback_id": mh.feedback_id,
            "channel": mh.channel,
            "model": mh.model,
            "method": method,
            "pos_min": round(float(mh.pos_min), 5),
            "pos_max": round(float(mh.pos_max), 5),
            "saved_at": time.time(),
            "result": result,
        }
        self._calibration_store[self._cal_key(mh.channel, mh.motor_id)] = record
        self._write_calibration_store()

    def _forget_calibration(self, mh: _MotorHandle):
        changed = self._calibration_store.pop(
            self._cal_key(mh.channel, mh.motor_id), None) is not None
        # Also drop any legacy single-arm key.
        changed = (self._calibration_store.pop(str(mh.motor_id), None)
                   is not None) or changed
        if changed:
            self._write_calibration_store()

    def _saved_span(self, mh: _MotorHandle) -> Optional[float]:
        """Return the saved hardstop-to-hardstop span (rad) for a motor, or None."""
        rec = self._calibration_store.get(self._cal_key(mh.channel, mh.motor_id))
        if not isinstance(rec, dict):
            rec = self._calibration_store.get(str(mh.motor_id))
        if not isinstance(rec, dict):
            return None
        res = rec.get("result") or {}
        try:
            span = float(res.get("span"))
            if span > 0:
                return span
        except (TypeError, ValueError):
            pass
        try:
            span = abs(float(rec["pos_max"]) - float(rec["pos_min"]))
            if span > 0:
                return span
        except (KeyError, TypeError, ValueError):
            pass
        return None

    # ------------------------------------------------------------------ #
    # Worker thread: owns the device, runs commands + control loop
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
                except BaseException as e:  # noqa: BLE001 - report back to caller
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
        if not self._ctrls or not self.motors:
            return
        # Snapshot per-motor intent under the lock.
        with self.lock:
            plan = [(mh, mh.enabled, mh.estopped, mh.mode, dict(mh.sp), mh._mode_applied)
                    for mh in self.motors.values()]

        for mh, enabled, estopped, mode, sp, mode_applied in plan:
            if mh._motor is None:
                continue
            if estopped:
                # Latched e-stop: keep torque cut every tick, never command
                # motion. Still poll telemetry so the UI shows live position.
                mh._motor.disable()
                try:
                    mh._motor.request_feedback()
                except Exception:
                    pass
                continue
            # A transient SDK error on one motor must not abort the whole tick
            # (which would starve every other motor of commands).
            try:
                if enabled:
                    if mode_applied != mode:
                        mh._motor.disable()
                        time.sleep(0.02)
                        mode_warning = self._ensure_mode_for_commands(mh, mode)
                        mh._motor.enable()
                        with self.lock:
                            mh._mode_applied = mode
                            if mode_warning is None:
                                mh.fault = None
                    if mode == "mit":
                        mh._motor.send_mit(sp["pos"], sp["vel"], sp["kp"], sp["kd"], sp["tau"])
                    elif mode == "pos_vel":
                        mh._motor.send_pos_vel(sp["pos"], sp["vlim"])
                    elif mode == "vel":
                        mh._motor.send_vel(sp["vel"])
                    elif mode == "force_pos":
                        mh._motor.send_force_pos(sp["pos"], sp["vlim"], sp["ratio"])
                else:
                    mh._motor.request_feedback()
            except Exception as e:  # noqa: BLE001
                with self.lock:
                    mh.fault = f"command error: {e}"

        # Drain feedback frames on every channel, then read each motor's state.
        counts: Dict[str, int] = {}
        for mh in self.motors.values():
            counts[str(mh.channel)] = counts.get(str(mh.channel), 0) + 1
        for ch, ctrl in self._ctrls.items():
            for _ in range(counts.get(ch, 0) + 2):
                try:
                    ctrl.poll_feedback_once()
                except Exception:
                    break
        for mh in list(self.motors.values()):
            if mh._motor is None:
                continue
            try:
                st = mh._motor.get_state()
            except Exception:
                st = None
            if st is not None:
                sc = st.status_code
                with self.lock:
                    mh.state = {
                        "pos": round(st.pos, 5),
                        "vel": round(st.vel, 5),
                        "torq": round(st.torq, 5),
                        "t_mos": round(st.t_mos, 1),
                        "t_rotor": round(st.t_rotor, 1),
                        "status_code": sc,
                        "status_text": DM_STATUS_TEXT.get(sc, f"code {sc}"),
                        "online": True,
                        "ts": time.time(),
                    }
                self._supervise(mh, sc)

    def _supervise(self, mh: _MotorHandle, sc: int):
        """Watch a commanded motor's reported status and react:
          - hardware fault (code >= 8): latch off and surface the reason; do NOT
            re-energize into an over-current / over-temperature condition.
          - clean unexpected disable (code 0 while we think it's enabled): the
            motor dropped out on its own -> auto re-enable so it keeps running
            without the user having to toggle Disable/Enable.
        """
        with self.lock:
            commanding = mh.enabled and not mh.estopped
        if not commanding:
            return
        if sc >= DM_FAULT_MIN:
            with self.lock:
                mh.enabled = False
                mh.estopped = True  # latch: require explicit re-enable
                mh.fault = f"hardware fault: {DM_STATUS_TEXT.get(sc, f'code {sc}')}"
            return
        if sc == 1:  # healthy/enabled
            mh._recover_count = 0
            return
        if sc == 0:  # motor reports disabled while we are commanding it
            now = time.time()
            if now - mh._last_recover_ts < 0.25:
                return
            mh._last_recover_ts = now
            mh._recover_count += 1
            if mh._recover_count > 12:
                with self.lock:
                    mh.enabled = False
                    mh.fault = ("motor keeps dropping out of enabled state -- "
                                "check power supply / CAN wiring")
                return
            # Force the mode-apply path to re-run (disable -> ensure_mode ->
            # enable) on the next tick, which re-energizes the motor.
            with self.lock:
                mh._mode_applied = None
                mh.fault = None

    # ------------------------------------------------------------------ #
    # Device operations (executed on the worker thread)
    # ------------------------------------------------------------------ #
    def _teardown(self):
        for mh in self.motors.values():
            if mh._motor is not None:
                try:
                    mh._motor.disable()
                except Exception:
                    pass
                try:
                    mh._motor.close()
                except Exception:
                    pass
                mh._motor = None
        for ctrl in self._ctrls.values():
            try:
                ctrl.close_bus()
            except Exception:
                pass
            try:
                ctrl.close()
            except Exception:
                pass
        self._ctrls = {}

    def _rebuild(self):
        """(Re)open the shared controller and re-add every known motor.

        The motorbridge model is fixed at add-time and re-adding a live id on a
        running controller fails, so any roster/model change goes through a full
        clean rebuild of the bus. One controller is opened per CAN channel in
        use so both arms of a dual adapter can run at once.
        """
        self._teardown()
        if not self.motors:
            return
        for mh in self.motors.values():
            ch = str(mh.channel)
            ctrl = self._ctrls.get(ch)
            if ctrl is None:
                ctrl = Controller.from_dm_device(self.device_type, ch)
                self._ctrls[ch] = ctrl
            mh._motor = ctrl.add_damiao_motor(
                mh.motor_id, mh.feedback_id, mh.model)
            mh._mode_applied = None
            # The physical mode register state is unknown after a rebuild, so
            # force the next MIT enable to re-assert it.
            mh._mode_register = None

    def _autodetect(self, mh: _MotorHandle) -> bool:
        """Read PMAX/VMAX/TMAX registers off a motor and pick the best model.
        Returns True if the stored model changed (caller must rebuild)."""
        try:
            pmax = mh._motor.get_register_f32(21, 500)
            vmax = mh._motor.get_register_f32(22, 500)
            tmax = mh._motor.get_register_f32(23, 500)
        except Exception:
            return False
        with self.lock:
            mh.limits = (round(pmax, 4), round(vmax, 4), round(tmax, 4))
        best = _best_model(pmax, vmax, tmax)
        if best and best != mh.model:
            mh.model = best
            return True
        return False

    def _add(self, motor_id, feedback_id, model, autodetect, channel):
        motor_id = int(motor_id)
        ch = str(channel) if channel is not None else self.channel
        model = model or self.default_model
        fid = int(feedback_id) if feedback_id is not None else motor_id + 0x10
        key = self._key(ch, motor_id)

        with self.lock:
            mh = self.motors.get(key)
            if mh is None:
                mh = _MotorHandle(motor_id, fid, model, ch)
                self.motors[key] = mh
            else:
                mh.feedback_id = fid
                mh.model = model
                mh.limits = DAMIAO_MODEL_LIMITS.get(model, (12.5, 30.0, 10.0))
                mh.enabled = False
                mh.estopped = False  # reconnect = clean slate (still disabled)
                mh.pos_min = None
                mh.pos_max = None

        self._rebuild()
        if autodetect:
            if self._autodetect(mh):
                self._rebuild()
        self._apply_saved_calibration(mh)
        with self.lock:
            self.fault = None
        return self.status()

    def _remove(self, channel, motor_id):
        with self.lock:
            self.motors.pop(self._key(channel, motor_id), None)
        self._rebuild()
        return self.status()

    def _probe_channel(self, ctrl, ch, start_id, end_id, polls, dwell):
        """Probe every id in [start_id, end_id] on an already-open controller."""
        found: List[dict] = []
        for mid in range(start_id, end_id + 1):
            fid = mid + 0x10
            try:
                motor = ctrl.add_damiao_motor(mid, fid, self.default_model)
            except Exception:
                continue
            try:
                motor.request_feedback()
                hit = None
                for _ in range(polls):
                    ctrl.poll_feedback_once()
                    st = motor.get_state()
                    if st is not None:
                        hit = st
                        break
                    time.sleep(dwell)
                if hit is not None:
                    found.append({
                        "channel": ch,
                        "motor_id": mid, "feedback_id": fid,
                        "status_code": hit.status_code,
                        "pos": round(hit.pos, 4),
                        "vel": round(hit.vel, 4),
                        "torq": round(hit.torq, 4),
                    })
            except Exception:
                pass
            finally:
                try:
                    motor.close()
                except Exception:
                    pass
                time.sleep(0.002)
        return found

    def _scan(self, start_id, end_id):
        # Scanning needs exclusive use of the bus, so drop the live controller
        # first, then restore the roster afterwards.
        self._teardown()
        channels = ["0", "1"] if "dual" in self.device_type else [self.channel]
        # Wide sweeps need a small per-id budget to stay responsive; present
        # motors answer on the first poll or two, so this rarely misses.
        span = max(1, end_id - start_id + 1)
        polls, dwell = (8, 0.006) if span > 32 else (15, 0.008)

        found: List[dict] = []
        opened = 0
        last_open_err: Optional[str] = None
        for ch in channels:
            try:
                ctrl = Controller.from_dm_device(self.device_type, ch)
            except Exception as e:  # noqa: BLE001
                last_open_err = str(e)
                continue
            opened += 1
            try:
                found.extend(self._probe_channel(ctrl, ch, start_id, end_id, polls, dwell))
            except Exception:
                pass
            finally:
                try:
                    ctrl.close_bus()
                except Exception:
                    pass
                try:
                    ctrl.close()
                except Exception:
                    pass

        # Restore any previously-connected motors.
        if self.motors:
            try:
                self._rebuild()
            except Exception:
                pass

        if opened == 0:
            raise RuntimeError(
                "CAN adapter not detected -- check the USB-CAN adapter is "
                "plugged in and powered" +
                (f" ({last_open_err})" if last_open_err else ""))
        return found

    def _ensure_mode(self, mh: _MotorHandle, mode_str, attempts: int = 6):
        """ensure_mode with retries -- the DM mode-register write ack is flaky."""
        last = None
        for _ in range(attempts):
            try:
                mh._motor.ensure_mode(MODE_MAP[mode_str], 1000)
                mh._mode_register = mode_str
                return
            except Exception as e:  # noqa: BLE001
                last = e
                time.sleep(0.06)
        raise last

    def _ensure_mode_for_commands(self, mh: _MotorHandle, mode_str: str) -> Optional[str]:
        """Apply a mode when the mode actually needs a register write.

        MIT commands are native CAN frames on DaMiao motors, so we normally skip
        the flaky mode-register write. BUT if the register was previously set to
        a non-MIT mode (e.g. pos_vel during calibration centering), those native
        MIT frames will not drive the motor until the register is put back to
        MIT, so we re-assert it (best-effort, since the ack can be flaky).
        Non-MIT modes always require a confirmed mode write.
        """
        if mode_str == "mit":
            if mh._mode_register != "mit":
                try:
                    self._ensure_mode(mh, "mit")
                except Exception:
                    pass
            return None
        self._ensure_mode(mh, mode_str)
        return None

    def _do_enable(self, channel, motor_id):
        mh = self._require(channel, motor_id)
        if mh._motor is None:
            raise RuntimeError("not connected")
        mh._motor.disable()
        time.sleep(0.03)
        mode_warning = self._ensure_mode_for_commands(mh, mh.mode)
        mh._motor.enable()
        with self.lock:
            mh._mode_applied = mh.mode
            mh.enabled = True
            mh.estopped = False  # explicit enable clears the e-stop latch
            mh.fault = None
        mh._recover_count = 0

    def _do_disable(self, channel, motor_id):
        mh = self._require(channel, motor_id)
        with self.lock:
            mh.enabled = False
            mh.max_power = False
        if mh._motor is not None:
            mh._motor.disable()

    def _do_zero(self, channel, motor_id):
        """Mechanical home calibration: cut torque (shaft free to be positioned
        by hand / against a stop), define the current position as zero, then
        read it back to verify. Leaves the motor disabled."""
        mh = self._require(channel, motor_id)
        ctrl = self._ctrl_for(mh)
        if mh._motor is None:
            raise RuntimeError("not connected")
        with self.lock:
            mh.enabled = False
            mh.max_power = False
            mh._mode_applied = None
            mh.pos_min = None
            mh.pos_max = None
        self._forget_calibration(mh)
        mh._motor.disable()
        time.sleep(0.05)
        mh._motor.set_zero_position()
        time.sleep(0.05)
        residual = None
        for _ in range(5):
            mh._motor.request_feedback()
            if ctrl is not None:
                ctrl.poll_feedback_once()
            st = mh._motor.get_state()
            if st is not None:
                residual = round(st.pos, 5)
                with self.lock:
                    mh.state["pos"] = residual
            time.sleep(0.02)
        ok = residual is not None and abs(residual) < 0.05
        return {"residual_pos": residual, "ok": ok}

    def _do_manual_calibrate(self, channel, motor_id, low_stop, high_stop,
                             center_tol=CAL_CENTER_TOL_RAD):
        """Manual hardstop calibration.

        The user moves the disabled joint by hand to both hardstops, then moves
        it to the computed midpoint before calling this. We only write zero if
        the current position is close to that midpoint.
        """
        mh = self._require(channel, motor_id)
        if mh._motor is None:
            raise RuntimeError("not connected")
        low = float(low_stop)
        high = float(high_stop)
        span = abs(high - low)
        if span < CAL_MIN_SPAN_RAD:
            raise RuntimeError(
                f"manual hardstop span too small ({span:.3f} rad); "
                "move to the actual low and high stops")
        center = (low + high) / 2.0
        with self.lock:
            mh.enabled = False
            mh.max_power = False
            mh._mode_applied = None
        mh._motor.disable()
        time.sleep(0.05)
        st = self._feedback_now(mh, polls=5, dwell=0.02)
        if st is None:
            raise RuntimeError("could not read current position for manual calibration")
        current = float(st.pos)
        center_err = current - center
        if abs(center_err) > float(center_tol):
            raise RuntimeError(
                f"move joint to midpoint {center:.3f} rad before saving "
                f"(current {current:.3f}, error {center_err:.3f})")

        mh._motor.set_zero_position()
        time.sleep(0.08)
        residual = None
        for _ in range(5):
            st = self._feedback_now(mh, polls=1, dwell=0.02)
            if st is not None:
                residual = round(float(st.pos), 5)
            time.sleep(0.02)
        pos_min = min(low, high) - center
        pos_max = max(low, high) - center
        result = {
            "low_stop": round(low, 5),
            "high_stop": round(high, 5),
            "span": round(span, 5),
            "center": round(center, 5),
            "center_error": round(center_err, 5),
            "pos_min": round(pos_min, 5),
            "pos_max": round(pos_max, 5),
            "residual_pos": residual,
            "ok": residual is not None and abs(residual) < 0.05,
            "manual": True,
        }
        with self.lock:
            mh.pos_min = pos_min
            mh.pos_max = pos_max
            mh.sp.update(pos=0.0, vel=0.0, tau=0.0)
            mh.mode = "mit"
            mh._mode_applied = None
            mh.fault = None
        self._save_calibration(mh, result, "manual")
        self._set_calibration(
            mh, active=False, phase="complete",
            message="manual midpoint zero calibrated", result=result)
        return result

    def _do_manual_calibrate_center(self, channel, motor_id, low_stop, high_stop,
                                    speed=CAL_SPEED_RAD_S):
        """Manual calibration with automatic centering.

        The user drives the joint to each hardstop and marks the two positions;
        we then drive to the midpoint on our own and write zero there. This is
        the flow used for arms whose hardstops are set by hand (e.g. Aloha).
        """
        mh = self._require(channel, motor_id)
        if mh._motor is None:
            raise RuntimeError("not connected")
        low = float(low_stop)
        high = float(high_stop)
        span = abs(high - low)
        if span < CAL_MIN_SPAN_RAD:
            raise RuntimeError(
                f"marked span too small ({span:.3f} rad); drive to and mark the "
                "actual front and rear hardstops")
        center = (low + high) / 2.0
        pos_min = min(low, high) - center
        pos_max = max(low, high) - center
        _, vmax, _ = mh.limits
        speed = _clamp(abs(float(speed or CAL_SPEED_RAD_S)), 0.05,
                       min(CAL_MAX_SPEED_RAD_S, vmax))
        kp = max(0.0, CAL_MIT_KP)
        kd = max(0.0, CAL_MIT_KD)
        lead = _clamp(CAL_MIT_TARGET_LEAD_RAD, 0.05, 0.35)

        # Take exclusive use of the bus: only the selected joint should move.
        with self.lock:
            for other in self.motors.values():
                other.enabled = False
                other.max_power = False
                other.sp.update(vel=0.0, tau=0.0)
        for other in self.motors.values():
            if other._motor is not None:
                try:
                    other._motor.disable()
                except Exception:
                    pass

        self._set_calibration(
            mh, active=True, phase="centering",
            message=f"moving to midpoint {center:.3f} rad", error=None,
            result=None)
        try:
            move_timeout = max(30.0, span / speed + 15.0)
            try:
                self._move_to_center_pos_vel(mh, center, speed, move_timeout)
            except Exception as pos_vel_error:  # noqa: BLE001
                self._set_calibration(
                    mh, active=True, phase="centering",
                    message=f"position midpoint move failed; trying MIT ({pos_vel_error})")
                self._enable_for_calibration(mh, "mit")
                self._move_to_center(mh, center, speed, move_timeout, kp, kd, lead)

            self._raise_if_calibration_stopped(mh)
            self._disable_after_calibration(mh)
            time.sleep(0.08)
            mh._motor.set_zero_position()
            time.sleep(0.08)
            residual = None
            for _ in range(5):
                st = self._feedback_now(mh)
                if st is not None:
                    residual = round(float(st.pos), 5)
                time.sleep(0.02)
            result = {
                "low_stop": round(low, 5),
                "high_stop": round(high, 5),
                "span": round(span, 5),
                "center": round(center, 5),
                "pos_min": round(pos_min, 5),
                "pos_max": round(pos_max, 5),
                "speed": round(speed, 3),
                "residual_pos": residual,
                "ok": residual is not None and abs(residual) < 0.05,
                "manual": True,
            }
            with self.lock:
                mh.pos_min = pos_min
                mh.pos_max = pos_max
                mh.sp.update(pos=0.0, vel=0.0, tau=0.0)
                mh.mode = "mit"
                mh._mode_applied = None
                mh.fault = None
            self._save_calibration(mh, result, "manual")
            self._set_calibration(
                mh, active=False, phase="complete",
                message="manual midpoint zero calibrated", result=result,
                saved_at=self._calibration_store.get(self._cal_key(mh.channel, mh.motor_id), {}).get("saved_at"))
            return result
        except Exception as e:  # noqa: BLE001
            self._disable_after_calibration(mh)
            with self.lock:
                mh.fault = f"calibration failed: {e}"
            self._set_calibration(
                mh, active=False, phase="failed", message=str(e), error=str(e))
            raise

    def _set_calibration(self, mh: _MotorHandle, *, active=_UNSET,
                         phase=_UNSET, message=_UNSET, error=_UNSET,
                         result=_UNSET, saved_at=_UNSET):
        with self.lock:
            cal = dict(mh.calibration)
            if active is not _UNSET:
                cal["active"] = bool(active)
            if phase is not _UNSET:
                cal["phase"] = phase
            if message is not _UNSET:
                cal["message"] = message
            if error is not _UNSET:
                cal["error"] = error
            if result is not _UNSET:
                cal["result"] = result
            if saved_at is not _UNSET:
                cal["saved_at"] = saved_at
            mh.calibration = cal

    def _raise_if_calibration_stopped(self, mh: _MotorHandle):
        with self.lock:
            stopped = self._stop or mh.estopped
        if stopped:
            raise RuntimeError("calibration stopped")

    def _feedback_now(self, mh: _MotorHandle, polls: int = 2,
                      dwell: float = 0.01):
        ctrl = self._ctrl_for(mh)
        if mh._motor is None or ctrl is None:
            return None
        st = None
        try:
            mh._motor.request_feedback()
        except Exception:
            pass
        for _ in range(polls):
            try:
                ctrl.poll_feedback_once()
            except Exception:
                pass
            try:
                cur = mh._motor.get_state()
            except Exception:
                cur = None
            if cur is not None:
                st = cur
                sc = cur.status_code
                with self.lock:
                    mh.state = {
                        "pos": round(cur.pos, 5),
                        "vel": round(cur.vel, 5),
                        "torq": round(cur.torq, 5),
                        "t_mos": round(cur.t_mos, 1),
                        "t_rotor": round(cur.t_rotor, 1),
                        "status_code": sc,
                        "status_text": DM_STATUS_TEXT.get(sc, f"code {sc}"),
                        "online": True,
                        "ts": time.time(),
                    }
                break
            time.sleep(dwell)
        return st

    def _enable_for_calibration(self, mh: _MotorHandle, mode: str):
        mh._motor.disable()
        time.sleep(0.03)
        mode_warning = self._ensure_mode_for_commands(mh, mode)
        mh._motor.enable()
        with self.lock:
            mh.mode = mode
            mh.enabled = True
            mh.estopped = False
            mh.max_power = False
            mh.fault = None
            mh._mode_applied = mode
        if mode_warning:
            self._set_calibration(mh, message="MIT mode ack missed; trying calibration")

    def _disable_after_calibration(self, mh: _MotorHandle):
        with self.lock:
            mh.enabled = False
            mh.max_power = False
            # Preserve the user's Kp/Kd so normal MIT slider control still works
            # after a failed or completed calibration.
            mh.sp.update(vel=0.0, tau=0.0)
            mh._mode_applied = None
        if mh._motor is not None:
            try:
                mh._motor.disable()
            except Exception:
                pass

    def _sweep_to_hardstop(self, mh: _MotorHandle, direction: int,
                           speed: float, max_s: float, phase: str,
                           kp: float, kd: float, lead: float) -> float:
        direction = 1 if direction >= 0 else -1
        _, _, tmax = mh.limits
        tmax = max(0.0, float(tmax))
        effort_budget = max(0.5, tmax * CAL_MIT_MAX_EFFORT_FRAC)
        max_tau_ff = min(CAL_MIT_MAX_TAU_FF, tmax * 0.08)
        effort_stage = 0
        self._set_calibration(
            mh, active=True, phase=phase,
            message=f"sweep {'forward' if direction > 0 else 'reverse'} "
                    f"at {speed:.2f} rad/s")
        start = time.time()
        deadline = start + max_s
        absolute_deadline = start + max(CAL_ABSOLUTE_SWEEP_S, max_s)
        last_progress_ts = start
        last_progress_pos = None
        last_pos = None
        target = None
        last_cmd_ts = start
        while time.time() < deadline and time.time() < absolute_deadline:
            self._raise_if_calibration_stopped(mh)
            now = time.time()
            if last_pos is None:
                st = self._feedback_now(mh)
                if st is not None:
                    last_pos = float(st.pos)
                    last_progress_pos = last_pos
                    target = last_pos
                time.sleep(0.03)
                continue

            # Advance the virtual target and keep only a bounded lead ahead of
            # the shaft. At the hardstop this caps effort near Kp * lead.
            dt = max(0.0, now - last_cmd_ts)
            last_cmd_ts = now
            effort_scale = CAL_EFFORT_STAGE_SCALES[effort_stage]
            stage_kp = min(CAL_MIT_MAX_KP, kp * effort_scale)
            stage_tau = 0.0
            if len(CAL_EFFORT_STAGE_SCALES) > 1:
                stage_tau = max_tau_ff * effort_stage / (len(CAL_EFFORT_STAGE_SCALES) - 1)
            # Keep estimated MIT effort below the motor's trip-prone region.
            # Commanded effort is roughly Kp * position lead + feed-forward tau.
            lead_budget = max(0.05, (effort_budget - stage_tau) / max(stage_kp, 0.1))
            stage_lead = min(CAL_MIT_MAX_TARGET_LEAD_RAD, lead * effort_scale, lead_budget)
            target = (target if target is not None else last_pos) + direction * speed * dt
            if direction > 0:
                target = min(target, last_pos + stage_lead)
            else:
                target = max(target, last_pos - stage_lead)
            mh._motor.send_mit(target, 0.0, stage_kp, kd, direction * stage_tau)
            st = self._feedback_now(mh)
            if st is not None:
                sc = st.status_code
                if sc >= DM_FAULT_MIN:
                    raise RuntimeError(
                        f"hardware fault while sweeping: "
                        f"{DM_STATUS_TEXT.get(sc, f'code {sc}')} "
                        f"(reduce CAL_MIT_MAX_EFFORT_FRAC or assist the joint)")
                pos = float(st.pos)
                vel = abs(float(st.vel))
                if last_progress_pos is None or abs(pos - last_progress_pos) >= CAL_PROGRESS_EPS_RAD:
                    last_progress_ts = now
                    last_progress_pos = pos
                    deadline = min(absolute_deadline, now + max_s)
                last_pos = pos
                past_startup = (now - start) >= CAL_MIN_SWEEP_S
                no_progress = (now - last_progress_ts) >= CAL_STALL_DWELL_S
                low_velocity = vel <= CAL_STALL_VEL_RAD_S
                possible_stall = past_startup and no_progress and (low_velocity or sc == 0)
                if possible_stall:
                    if effort_stage < len(CAL_EFFORT_STAGE_SCALES) - 1:
                        effort_stage += 1
                        last_progress_ts = now
                        last_progress_pos = last_pos
                        target = last_pos
                        next_scale = CAL_EFFORT_STAGE_SCALES[effort_stage]
                        self._set_calibration(
                            mh, active=True, phase=phase,
                            message="load stall; increasing calibration effort "
                                    f"(stage {effort_stage + 1}, "
                                    f"Kp {min(CAL_MIT_MAX_KP, kp * next_scale):.1f}, "
                                    f"budget {effort_budget:.1f} Nm)")
                        continue
                    return last_pos
            time.sleep(0.03)
        raise RuntimeError(
            f"no hardstop detected after {time.time() - start:.0f}s "
            f"({phase}, last position {last_pos})")

    def _move_to_center(self, mh: _MotorHandle, target: float, speed: float,
                        timeout_s: float, kp: float, kd: float,
                        lead_limit: float, max_lead: Optional[float] = None,
                        max_tau_ff: Optional[float] = None):
        self._set_calibration(
            mh, active=True, phase="centering",
            message=f"moving to midpoint {target:.3f} rad")
        start = time.time()
        last_err = None
        last_progress_ts = start
        last_progress_pos = None
        last_reenable_ts = 0.0
        last_cmd_ts = start
        command_target = None
        effort_stage = 0
        _, _, tmax = mh.limits
        tmax = max(0.0, float(tmax))
        max_lead = CAL_MIT_MAX_TARGET_LEAD_RAD if max_lead is None else float(max_lead)
        max_tau_ff = min(CAL_MIT_MAX_TAU_FF, tmax * 0.08) if max_tau_ff is None else float(max_tau_ff)
        while time.time() - start < timeout_s:
            self._raise_if_calibration_stopped(mh)
            now = time.time()
            st = self._feedback_now(mh)
            if st is not None:
                if st.status_code >= DM_FAULT_MIN:
                    raise RuntimeError(
                        f"hardware fault while centering: "
                        f"{DM_STATUS_TEXT.get(st.status_code, f'code {st.status_code}')}")
                if st.status_code == 0:
                    if now - last_reenable_ts > 0.5:
                        last_reenable_ts = now
                        self._set_calibration(
                            mh, active=True, phase="centering",
                            message="motor dropped disabled while centering; re-enabling")
                        self._enable_for_calibration(mh, "mit")
                    time.sleep(0.05)
                    continue
                pos = float(st.pos)
                if command_target is None:
                    command_target = pos
                last_err = target - pos
                if abs(last_err) <= CAL_CENTER_TOL_RAD and abs(float(st.vel)) <= speed:
                    return
                if last_progress_pos is None or abs(pos - last_progress_pos) >= CAL_PROGRESS_EPS_RAD:
                    last_progress_ts = time.time()
                    last_progress_pos = pos
                elif time.time() - last_progress_ts >= 1.0 and effort_stage < len(CAL_EFFORT_STAGE_SCALES) - 1:
                    effort_stage += 1
                    last_progress_ts = time.time()
                    self._set_calibration(
                        mh, active=True, phase="centering",
                        message=f"centering stalled; increasing effort stage {effort_stage + 1}")
                effort_scale = CAL_EFFORT_STAGE_SCALES[effort_stage]
                stage_kp = min(CAL_MIT_MAX_KP, max(kp, kp * effort_scale))
                stage_lead = min(max_lead, lead_limit * effort_scale)
                stage_tau = 0.0
                if len(CAL_EFFORT_STAGE_SCALES) > 1:
                    stage_tau = max_tau_ff * effort_stage / (len(CAL_EFFORT_STAGE_SCALES) - 1)
                direction = 1 if last_err >= 0 else -1
                dt = max(0.0, now - last_cmd_ts)
                last_cmd_ts = now
                command_target += direction * speed * dt
                if direction > 0:
                    command_target = min(command_target, target, pos + stage_lead)
                else:
                    command_target = max(command_target, target, pos - stage_lead)
                mh._motor.send_mit(command_target, 0.0, stage_kp, kd, direction * stage_tau)
            time.sleep(0.03)
        raise RuntimeError(
            f"could not reach midpoint within {timeout_s:.0f}s "
            f"(last error {last_err})")

    def _move_to_center_pos_vel(self, mh: _MotorHandle, target: float,
                                speed: float, timeout_s: float):
        self._set_calibration(
            mh, active=True, phase="centering",
            message=f"position move to midpoint {target:.3f} rad")
        speed = max(0.05, abs(float(speed)))
        start = time.time()
        last_err = None
        last_reenable_ts = 0.0
        self._enable_for_calibration(mh, "pos_vel")
        while time.time() - start < timeout_s:
            self._raise_if_calibration_stopped(mh)
            mh._motor.send_pos_vel(target, speed)
            st = self._feedback_now(mh)
            if st is not None:
                if st.status_code >= DM_FAULT_MIN:
                    raise RuntimeError(
                        f"hardware fault while position-centering: "
                        f"{DM_STATUS_TEXT.get(st.status_code, f'code {st.status_code}')}")
                if st.status_code == 0:
                    now = time.time()
                    if now - last_reenable_ts > 0.5:
                        last_reenable_ts = now
                        self._set_calibration(
                            mh, active=True, phase="centering",
                            message="motor dropped disabled during position move; re-enabling")
                        self._enable_for_calibration(mh, "pos_vel")
                    time.sleep(0.05)
                    continue
                last_err = target - float(st.pos)
                if abs(last_err) <= CAL_CENTER_TOL_RAD and abs(float(st.vel)) <= speed:
                    return
            time.sleep(0.03)
        raise RuntimeError(
            f"position move could not reach midpoint within {timeout_s:.0f}s "
            f"(last error {last_err})")

    def _do_auto_calibrate_one(self, channel, motor_id, speed=CAL_SPEED_RAD_S,
                               max_s=CAL_MAX_SWEEP_S):
        mh = self._require(channel, motor_id)
        if mh._motor is None:
            raise RuntimeError("not connected")
        profile = CAL_JOINT_PROFILES.get(int(motor_id), {})
        if (speed is None or float(speed or 0.0) == CAL_SPEED_RAD_S) and "speed" in profile:
            speed = profile["speed"]
        speed = abs(float(speed or CAL_SPEED_RAD_S))
        _, vmax, _ = mh.limits
        speed = _clamp(speed, 0.05, min(CAL_MAX_SPEED_RAD_S, vmax))
        max_s = _clamp(float(max_s or CAL_MAX_SWEEP_S), 10.0, CAL_ABSOLUTE_SWEEP_S)
        kp = max(0.0, float(profile.get("kp", CAL_MIT_KP)))
        kd = max(0.0, float(profile.get("kd", CAL_MIT_KD)))
        lead = _clamp(float(profile.get("lead", CAL_MIT_TARGET_LEAD_RAD)), 0.05, 0.35)

        # Calibration owns the bus for this command; leave all motors disabled
        # so only the selected joint can move.
        with self.lock:
            for other in self.motors.values():
                other.enabled = False
                other.max_power = False
                other.sp.update(vel=0.0, tau=0.0)
        for other in self.motors.values():
            if other._motor is not None:
                try:
                    other._motor.disable()
                except Exception:
                    pass

        self._set_calibration(
            mh, active=True, phase="starting",
            message=f"auto calibration at {speed:.2f} rad/s "
                    f"(Kp {kp:.1f}, lead {lead:.2f})", error=None,
            result=None)
        is_j4 = int(motor_id) == 4
        method = "auto"
        try:
            self._enable_for_calibration(mh, "mit")
            low = self._sweep_to_hardstop(
                mh, -1, speed, max_s, "finding low stop", kp, kd, lead)
            self._set_calibration(
                mh, active=True, phase="found low stop",
                message=f"low hardstop at {low:.3f} rad")
            time.sleep(0.25)
            self._raise_if_calibration_stopped(mh)
            if is_j4:
                # J4 must not drive into its front stop, so reuse the saved span
                # (or the configured default) to place the forward extent.
                method = "j4_back_saved_front"
                span = self._saved_span(mh) or CAL_J4_DEFAULT_SPAN_RAD
                high = low + span
                self._set_calibration(
                    mh, active=True, phase="using saved front",
                    message=f"J4 back stop at {low:.3f} rad; using saved span "
                            f"{span:.3f} rad for the front extent")
            else:
                high = self._sweep_to_hardstop(
                    mh, 1, speed, max_s, "finding high stop", kp, kd, lead)
            span = abs(high - low)
            if span < CAL_MIN_SPAN_RAD:
                raise RuntimeError(
                    f"hardstop span too small ({span:.3f} rad); check motion path")
            center = (low + high) / 2.0
            pos_min = min(low, high) - center
            pos_max = max(low, high) - center
            move_timeout = max(30.0, span / speed + 15.0)
            self._raise_if_calibration_stopped(mh)
            if is_j4:
                # Position mode reaches the midpoint reliably from the back stop;
                # fall back to stronger MIT centering only if pos_vel setup fails.
                try:
                    self._move_to_center_pos_vel(mh, center, speed, move_timeout)
                except Exception as pos_vel_error:
                    self._set_calibration(
                        mh, active=True, phase="centering",
                        message=f"position midpoint move failed; trying MIT ({pos_vel_error})")
                    self._enable_for_calibration(mh, "mit")
                    self._move_to_center(
                        mh, center, speed, move_timeout,
                        max(kp, CAL_J4_CENTER_KP), kd,
                        max(lead, CAL_J4_CENTER_LEAD_RAD),
                        max_lead=CAL_J4_CENTER_LEAD_RAD,
                        max_tau_ff=CAL_J4_CENTER_TAU_FF)
            else:
                self._move_to_center(mh, center, speed, move_timeout, kp, kd, lead)

            self._raise_if_calibration_stopped(mh)
            self._disable_after_calibration(mh)
            time.sleep(0.08)
            mh._motor.set_zero_position()
            time.sleep(0.08)
            residual = None
            for _ in range(5):
                st = self._feedback_now(mh)
                if st is not None:
                    residual = round(float(st.pos), 5)
                time.sleep(0.02)
            result = {
                "low_stop": round(low, 5),
                "high_stop": round(high, 5),
                "span": round(span, 5),
                "center": round(center, 5),
                "pos_min": round(pos_min, 5),
                "pos_max": round(pos_max, 5),
                "speed": round(speed, 3),
                "kp": round(kp, 3),
                "kd": round(kd, 3),
                "lead": round(lead, 3),
                "residual_pos": residual,
                "ok": residual is not None and abs(residual) < 0.05,
            }
            with self.lock:
                mh.pos_min = pos_min
                mh.pos_max = pos_max
                mh.sp.update(pos=0.0, vel=0.0, tau=0.0)
                mh.mode = "mit"
                mh._mode_applied = None
            self._save_calibration(mh, result, method)
            self._set_calibration(
                mh, active=False, phase="complete",
                message="midpoint zero calibrated", result=result,
                saved_at=self._calibration_store.get(self._cal_key(mh.channel, mh.motor_id), {}).get("saved_at"))
            return result
        except Exception as e:
            self._disable_after_calibration(mh)
            with self.lock:
                mh.fault = f"calibration failed: {e}"
            self._set_calibration(
                mh, active=False, phase="failed", message=str(e), error=str(e))
            raise

    def _do_return_home(self, channel=None, motor_id=None):
        single = motor_id is not None
        if single:
            targets = [self._require(channel, motor_id)]
        elif channel is not None:
            targets = [self.motors[k] for k in sorted(self.motors)
                       if k[0] == str(channel)]
        else:
            targets = [self.motors[k] for k in sorted(self.motors)]
        homed = []
        for mh in targets:
            if mh._motor is None:
                continue
            pmax, _, _ = mh.limits
            calibrated = mh.pos_min is not None and mh.pos_max is not None
            if not calibrated and not single:
                continue
            with self.lock:
                mh.mode = "mit"
                mh.max_power = False
                mh.estopped = False
                mh.sp.update(
                    pos=_clamp(0.0, -pmax if mh.pos_min is None else mh.pos_min,
                               pmax if mh.pos_max is None else mh.pos_max),
                    vel=0.0,
                    kp=max(float(mh.sp.get("kp", 0.0)), CAL_MIT_KP),
                    kd=max(float(mh.sp.get("kd", 0.0)), CAL_MIT_KD),
                    tau=0.0,
                )
            self._do_enable(mh.channel, mh.motor_id)
            homed.append({"channel": mh.channel, "motor_id": mh.motor_id})
        return {"homed": homed}

    # ------------------------------------------------------------------ #
    # Public API (called from HTTP threads)
    # ------------------------------------------------------------------ #
    def connect(self, motor_id, model=None, feedback_id=None,
                autodetect_model=True, channel=None):
        return self._submit(lambda: self._add(
            motor_id, feedback_id, model, autodetect_model, channel))

    def connect_many(self, items):
        """items: list of dicts with motor_id and optional feedback_id/channel."""
        def run():
            last = None
            for it in items:
                last = self._add(
                    it["motor_id"], it.get("feedback_id"),
                    it.get("model"), it.get("autodetect", True),
                    it.get("channel"))
            return last or self.status()
        return self._submit(run, timeout=40.0)

    def disconnect(self, channel, motor_id):
        return self._submit(lambda: self._remove(channel, motor_id))

    def scan(self, start_id=1, end_id=16):
        return self._submit(lambda: self._scan(start_id, end_id), timeout=40.0)

    def enable(self, channel, motor_id):
        return self._submit(lambda: self._do_enable(channel, motor_id))

    def disable(self, channel, motor_id):
        return self._submit(lambda: self._do_disable(channel, motor_id))

    def set_zero(self, channel, motor_id):
        return self._submit(lambda: self._do_zero(channel, motor_id))

    def manual_calibrate(self, channel, motor_id, low_stop, high_stop,
                         center_tol=CAL_CENTER_TOL_RAD):
        return self._submit(
            lambda: self._do_manual_calibrate(
                channel, motor_id, low_stop, high_stop, center_tol),
            timeout=20.0)

    def manual_calibrate_center(self, channel, motor_id, low_stop, high_stop,
                                speed=CAL_SPEED_RAD_S):
        return self._submit(
            lambda: self._do_manual_calibrate_center(
                channel, motor_id, low_stop, high_stop, speed),
            timeout=150.0)

    def clear_calibration(self, channel, motor_id):
        """Forget a motor's soft limits / saved calibration so it can be
        (re)calibrated across its full mechanical range."""
        mh = self._require(channel, motor_id)
        with self.lock:
            mh.pos_min = None
            mh.pos_max = None
            mh.calibration = _blank_calibration()
        self._forget_calibration(mh)
        return {"ok": True}

    def auto_calibrate(self, channel, motor_id, speed=CAL_SPEED_RAD_S,
                       max_s=CAL_MAX_SWEEP_S):
        timeout = float(max_s) * 2.0 + 90.0
        return self._submit(
            lambda: self._do_auto_calibrate_one(channel, motor_id, speed, max_s),
            timeout=timeout)

    def auto_calibrate_all(self, speed=CAL_SPEED_RAD_S,
                           max_s=CAL_MAX_SWEEP_S):
        def run():
            results = []
            for key in sorted(self.motors):
                ch, mid = key
                results.append({
                    "channel": ch,
                    "motor_id": mid,
                    **self._do_auto_calibrate_one(ch, mid, speed, max_s),
                })
            return results
        timeout = (float(max_s) * 2.0 + 90.0) * max(1, len(self.motors))
        return self._submit(run, timeout=timeout)

    def return_home(self, channel=None, motor_id=None):
        return self._submit(
            lambda: self._do_return_home(channel, motor_id), timeout=60.0)

    def estop(self, channel=None, motor_id=None):
        # Latch e-stop: flip flags so the worker stops commanding AND holds the
        # motor(s) disabled every tick, then fire an immediate disable. The latch
        # stays until the user explicitly re-enables. With no target, every motor
        # on every arm is stopped.
        with self.lock:
            if motor_id is not None and channel is not None:
                key = self._key(channel, motor_id)
                targets = [self.motors[key]] if key in self.motors else []
            elif channel is not None:
                targets = [mh for k, mh in self.motors.items()
                           if k[0] == str(channel)]
            else:
                targets = list(self.motors.values())
            keys = [self._key(mh.channel, mh.motor_id) for mh in targets]
            for mh in targets:
                mh.enabled = False
                mh.max_power = False
                mh.estopped = True
                mh.sp.update(pos=mh.state.get("pos", 0.0), vel=0.0,
                             kp=0.0, kd=0.0, tau=0.0)

        def _disable_now():
            for key in keys:
                mh = self.motors.get(key)
                if mh is not None and mh._motor is not None:
                    try:
                        mh._motor.disable()
                    except Exception:
                        pass
        try:
            self._submit(_disable_now, timeout=5.0)
        except Exception:
            pass

    def set_max_power(self, channel, motor_id, on: bool, direction: int = 1):
        """Full-send toggle for one motor: command maximum torque (TMAX) in MIT
        mode in the given direction, and enable."""
        direction = 1 if direction >= 0 else -1
        mh = self._require(channel, motor_id)
        with self.lock:
            mh.max_power = bool(on)
            tmax = mh.limits[2]
            if on:
                mh.mode = "mit"
                mh.sp.update(pos=0.0, vel=0.0, kp=0.0, kd=0.0,
                             tau=direction * tmax)
            else:
                mh.sp.update(kp=0.0, kd=0.0, tau=0.0)
        if on:
            self.enable(channel, motor_id)

    def set_mode(self, channel, motor_id, mode: str):
        if mode not in MODE_MAP:
            raise ValueError(f"unknown mode: {mode}")
        mh = self._require(channel, motor_id)
        with self.lock:
            mh.mode = mode
            mh._mode_applied = None  # worker re-applies on next tick
            mh.max_power = False

    def set_targets(self, channel, motor_id, **kw):
        mh = self._require(channel, motor_id)
        with self.lock:
            pmax, vmax, tmax = mh.limits
            pos_min = -pmax if mh.pos_min is None else mh.pos_min
            pos_max = pmax if mh.pos_max is None else mh.pos_max
            if "pos" in kw:
                mh.sp["pos"] = _clamp(float(kw["pos"]), pos_min, pos_max)
            if "vel" in kw:
                mh.sp["vel"] = _clamp(float(kw["vel"]), -vmax, vmax)
            if "tau" in kw:
                mh.sp["tau"] = _clamp(float(kw["tau"]), -tmax, tmax)
            if "kp" in kw:
                mh.sp["kp"] = _clamp(float(kw["kp"]), 0.0, 500.0)
            if "kd" in kw:
                mh.sp["kd"] = _clamp(float(kw["kd"]), 0.0, 5.0)
            if "vlim" in kw:
                mh.sp["vlim"] = _clamp(float(kw["vlim"]), 0.0, vmax)
            if "ratio" in kw:
                mh.sp["ratio"] = _clamp(float(kw["ratio"]), 0.0, 1.0)

    def status(self) -> dict:
        with self.lock:
            motors = [self.motors[k].status() for k in sorted(self.motors)]
            return {
                "device_type": self.device_type,
                "channel": self.channel,
                "fault": self.fault,
                "connected": bool(self.motors),
                "motors": motors,
            }

    def shutdown(self):
        self._stop = True
