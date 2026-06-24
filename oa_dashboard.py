#!/usr/bin/env python3
"""Flask dashboard for OpenArm bimanual CAN-to-CAN teleoperation.

Serves a small web UI that lets you bring up the CAN buses, see which motors are
connected on each of the four arms, and run / stop / e-stop / swap a unilateral
leader -> follower teleoperation. The control loop itself lives in
``oa_engine.OpenArmTeleop`` (one worker thread owns all CAN access).

    HOST=0.0.0.0 PORT=5002 ./venv/bin/python oa_dashboard.py
"""

from __future__ import annotations

import json
import os
import time

from flask import Flask, Response, jsonify, render_template, request

import oa_engine

app = Flask(__name__)

RATE_HZ = float(os.environ.get("OA_RATE_HZ", "120"))
service = oa_engine.OpenArmTeleop(rate_hz=RATE_HZ)


def _ok(**kw):
    return jsonify({"ok": True, **kw})


def _err(msg, code=400):
    return jsonify({"ok": False, "error": str(msg)}), code


@app.route("/")
def index():
    return render_template("openarm.html")


@app.route("/api/oa/status")
def api_status():
    return jsonify(service.status())


@app.route("/api/oa/stream")
def api_stream():
    def gen():
        while True:
            yield f"data: {json.dumps(service.status())}\n\n"
            time.sleep(0.1)
    return Response(gen(), mimetype="text/event-stream",
                    headers={"Cache-Control": "no-cache",
                             "X-Accel-Buffering": "no"})


# ----------------------------- bus management ----------------------------- #
@app.route("/api/oa/bus/up_all", methods=["POST"])
def api_bus_up_all():
    try:
        res = service.bring_up_all()
    except Exception as e:  # noqa: BLE001
        return _err(e, 500)
    return _ok(**res, status=service.status())


@app.route("/api/oa/bus/up", methods=["POST"])
def api_bus_up():
    body = request.get_json(silent=True) or {}
    bus = body.get("bus")
    if not bus:
        return _err("bus is required")
    try:
        res = service.bring_up_one(str(bus))
    except Exception as e:  # noqa: BLE001
        return _err(e, 500)
    return _ok(result=res, status=service.status())


@app.route("/api/oa/bus/down", methods=["POST"])
def api_bus_down():
    body = request.get_json(silent=True) or {}
    bus = body.get("bus")
    if not bus:
        return _err("bus is required")
    try:
        res = service.bring_down_one(str(bus))
    except Exception as e:  # noqa: BLE001
        return _err(e, 500)
    return _ok(result=res, status=service.status())


# ----------------------------- connection -------------------------------- #
@app.route("/api/oa/connect", methods=["POST"])
def api_connect():
    try:
        st = service.connect()
    except Exception as e:  # noqa: BLE001
        return _err(e, 500)
    return _ok(status=st)


@app.route("/api/oa/disconnect", methods=["POST"])
def api_disconnect():
    try:
        st = service.disconnect()
    except Exception as e:  # noqa: BLE001
        return _err(e, 500)
    return _ok(status=st)


@app.route("/api/oa/identify", methods=["POST"])
def api_identify():
    body = request.get_json(silent=True) or {}
    bus = body.get("bus")
    if not bus:
        return _err("bus is required")
    try:
        res = service.identify(str(bus))
    except Exception as e:  # noqa: BLE001
        return _err(e, 500)
    return _ok(**res)


# ----------------------------- teleop control ---------------------------- #
@app.route("/api/oa/teleop", methods=["POST"])
def api_teleop():
    body = request.get_json(silent=True) or {}
    action = body.get("action")
    try:
        if action == "monitor":
            st = service.monitor()
        elif action == "follow":
            st = service.follow()
        elif action == "stop":
            st = service.stop()
        elif action == "estop":
            st = service.estop()
        elif action == "clear_estop":
            st = service.clear_estop()
        elif action == "swap":
            st = service.swap()
        elif action == "cross":
            st = service.cross()
        else:
            return _err(f"unknown action: {action!r}")
    except Exception as e:  # noqa: BLE001
        return _err(e, 500)
    return _ok(status=st)


@app.route("/api/oa/params", methods=["POST"])
def api_params():
    body = request.get_json(silent=True) or {}
    try:
        st = service.set_params(
            gain_scale=body.get("gain_scale"),
            max_step=body.get("max_step"),
            tau=body.get("tau"),
            ramp_s=body.get("ramp_s"),
            use_cal_limits=body.get("use_cal_limits"),
        )
    except Exception as e:  # noqa: BLE001
        return _err(e, 500)
    return _ok(status=st)


if __name__ == "__main__":
    host = os.environ.get("HOST", "127.0.0.1")
    port = int(os.environ.get("PORT", "5002"))
    print(f"OpenArm teleop dashboard on http://{host}:{port}")
    app.run(host=host, port=port, threaded=True, debug=False)
