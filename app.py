"""Flask web dashboard for controlling a DaMiao DM-series motor."""

from __future__ import annotations

import json
import os
import time

from flask import Flask, Response, jsonify, render_template, request

import backend

app = Flask(__name__)

DEVICE_TYPE = os.environ.get("DM_DEVICE_TYPE", "usb2canfd-dual")
DEVICE_CHANNEL = os.environ.get("DM_CHANNEL", "0")
DEFAULT_MODEL = os.environ.get("DM_MODEL", "4310")

service = backend.MotorService(
    device_type=DEVICE_TYPE, channel=DEVICE_CHANNEL, model=DEFAULT_MODEL)


def _ok(**kw):
    return jsonify({"ok": True, **kw})


def _err(msg, code=400):
    return jsonify({"ok": False, "error": str(msg)}), code


@app.route("/")
def index():
    return render_template("index.html", models=sorted(backend.DAMIAO_MODEL_LIMITS))


@app.route("/api/status")
def api_status():
    return jsonify(service.status())


@app.route("/api/scan", methods=["POST"])
def api_scan():
    body = request.get_json(silent=True) or {}
    start = int(body.get("start_id", 1))
    end = int(body.get("end_id", 16))
    try:
        found = service.scan(start, end)
    except Exception as e:
        return _err(e, 500)
    return _ok(found=found)


def _mid(body):
    if "motor_id" not in body or body["motor_id"] is None:
        raise ValueError("motor_id is required")
    return int(body["motor_id"])


def _chan(body):
    """Channel (arm) for a request. Defaults to the service's primary channel
    so single-arm setups keep working without sending a channel."""
    ch = body.get("channel")
    return str(ch) if ch is not None else service.channel


@app.route("/api/connect", methods=["POST"])
def api_connect():
    body = request.get_json(silent=True) or {}
    try:
        # Accept either a single motor or a batch ("motors": [...]).
        if isinstance(body.get("motors"), list):
            items = []
            for it in body["motors"]:
                items.append({
                    "motor_id": int(it["motor_id"]),
                    "feedback_id": int(it["feedback_id"]) if it.get("feedback_id") is not None else None,
                    "model": it.get("model"),
                    "channel": str(it["channel"]) if it.get("channel") is not None else None,
                    "autodetect": bool(it.get("autodetect", True)),
                })
            status = service.connect_many(items)
        else:
            feedback_id = body.get("feedback_id")
            channel = body.get("channel")
            status = service.connect(
                motor_id=_mid(body),
                model=body.get("model"),
                feedback_id=int(feedback_id) if feedback_id is not None else None,
                autodetect_model=bool(body.get("autodetect", True)),
                channel=str(channel) if channel is not None else None,
            )
    except Exception as e:
        return _err(e, 500)
    return _ok(status=status)


@app.route("/api/disconnect", methods=["POST"])
def api_disconnect():
    body = request.get_json(silent=True) or {}
    try:
        status = service.disconnect(_chan(body), _mid(body))
    except Exception as e:
        return _err(e, 500)
    return _ok(status=status)


@app.route("/api/enable", methods=["POST"])
def api_enable():
    body = request.get_json(silent=True) or {}
    try:
        service.enable(_chan(body), _mid(body))
    except Exception as e:
        return _err(e, 500)
    return _ok()


@app.route("/api/disable", methods=["POST"])
def api_disable():
    body = request.get_json(silent=True) or {}
    try:
        service.disable(_chan(body), _mid(body))
    except Exception as e:
        return _err(e, 500)
    return _ok()


@app.route("/api/activate", methods=["POST"])
def api_activate():
    # Make one arm/channel the live bus (the dual adapter drives one at a time).
    body = request.get_json(silent=True) or {}
    try:
        st = service.activate(_chan(body))
    except Exception as e:
        return _err(e, 500)
    return _ok(status=st)


@app.route("/api/arm_side", methods=["POST"])
def api_arm_side():
    # Mark an arm (channel) as "left" or "right" so reverse-only calibration
    # sweeps away from the body in the correct direction.
    body = request.get_json(silent=True) or {}
    try:
        st = service.set_arm_side(_chan(body), body.get("side"))
    except Exception as e:
        return _err(e, 400)
    return _ok(status=st)


@app.route("/api/identify_arm", methods=["POST"])
def api_identify_arm():
    # Briefly wiggle a motor on this arm so the user can tell which is which.
    body = request.get_json(silent=True) or {}
    try:
        res = service.identify_arm(_chan(body)) or {}
    except Exception as e:
        return _err(e, 500)
    return _ok(**res)


@app.route("/api/estop", methods=["POST"])
def api_estop():
    body = request.get_json(silent=True) or {}
    # No motor_id => emergency-stop every motor (optionally limited to a channel).
    channel = body.get("channel")
    motor_id = body.get("motor_id")
    service.estop(str(channel) if channel is not None else None,
                  int(motor_id) if motor_id is not None else None)
    return _ok()


@app.route("/api/max_power", methods=["POST"])
def api_max_power():
    body = request.get_json(silent=True) or {}
    try:
        service.set_max_power(_chan(body), _mid(body), bool(body.get("on", False)),
                              int(body.get("direction", 1)))
    except Exception as e:
        return _err(e, 500)
    return _ok()


@app.route("/api/mode", methods=["POST"])
def api_mode():
    body = request.get_json(silent=True) or {}
    try:
        service.set_mode(_chan(body), _mid(body), body["mode"])
    except Exception as e:
        return _err(e)
    return _ok()


@app.route("/api/reset_params", methods=["POST"])
def api_reset_params():
    body = request.get_json(silent=True) or {}
    kp = body.get("kp")
    kd = body.get("kd")
    try:
        service.reset_params(
            _chan(body), _mid(body),
            kp=float(kp) if kp is not None else None,
            kd=float(kd) if kd is not None else None)
    except Exception as e:
        return _err(e, 500)
    return _ok()


@app.route("/api/target", methods=["POST"])
def api_target():
    body = request.get_json(silent=True) or {}
    try:
        service.set_targets(_chan(body), _mid(body), **{k: body[k] for k in
                            ("pos", "vel", "kp", "kd", "tau", "vlim", "ratio")
                            if k in body})
    except Exception as e:
        return _err(e)
    return _ok()


@app.route("/api/zero", methods=["POST"])
def api_zero():
    body = request.get_json(silent=True) or {}
    try:
        res = service.set_zero(_chan(body), _mid(body)) or {}
    except Exception as e:
        return _err(e, 500)
    return _ok(residual_pos=res.get("residual_pos"), verified=res.get("ok"))


@app.route("/api/manual_calibrate", methods=["POST"])
def api_manual_calibrate():
    body = request.get_json(silent=True) or {}
    try:
        res = service.manual_calibrate(
            _chan(body), _mid(body), body["low_stop"], body["high_stop"],
            float(body.get("center_tol", backend.CAL_CENTER_TOL_RAD))) or {}
    except Exception as e:
        return _err(e, 500)
    return _ok(result=res, verified=res.get("ok"))


@app.route("/api/clear_calibration", methods=["POST"])
def api_clear_calibration():
    body = request.get_json(silent=True) or {}
    try:
        service.clear_calibration(_chan(body), _mid(body))
    except Exception as e:
        return _err(e, 500)
    return _ok()


@app.route("/api/manual_calibrate_center", methods=["POST"])
def api_manual_calibrate_center():
    body = request.get_json(silent=True) or {}
    try:
        res = service.manual_calibrate_center(
            _chan(body), _mid(body), body["low_stop"], body["high_stop"],
            float(body.get("speed", backend.CAL_SPEED_RAD_S))) or {}
    except Exception as e:
        return _err(e, 500)
    return _ok(result=res, verified=res.get("ok"))


@app.route("/api/auto_calibrate", methods=["POST"])
def api_auto_calibrate():
    body = request.get_json(silent=True) or {}
    try:
        speed = float(body.get("speed", backend.CAL_SPEED_RAD_S))
        max_s = float(body.get("max_s", backend.CAL_MAX_SWEEP_S))
        if body.get("all"):
            results = service.auto_calibrate_all(speed=speed, max_s=max_s)
            return _ok(results=results)
        if body.get("all_arms"):
            # Calibrate this joint on every arm at the same time.
            results = service.auto_calibrate_joint(
                _mid(body), speed=speed, max_s=max_s)
            ok = all(r.get("ok") for r in results) if results else False
            return _ok(results=results, verified=ok)
        res = service.auto_calibrate(
            _chan(body), _mid(body), speed=speed, max_s=max_s) or {}
    except Exception as e:
        return _err(e, 500)
    return _ok(result=res, verified=res.get("ok"))


@app.route("/api/home", methods=["POST"])
def api_home():
    body = request.get_json(silent=True) or {}
    channel = body.get("channel")
    motor_id = body.get("motor_id")
    try:
        res = service.return_home(
            str(channel) if channel is not None else None,
            int(motor_id) if motor_id is not None else None) or {}
    except Exception as e:
        return _err(e, 500)
    return _ok(**res)


@app.route("/api/gesture", methods=["POST"])
def api_gesture():
    body = request.get_json(silent=True) or {}
    name = body.get("name")
    try:
        if body.get("stop"):
            service.stop_gesture()
            return _ok(stopped=True)
        speed = body.get("speed")
        info = service.play_gesture(
            name, speed=float(speed) if speed is not None else None)
    except Exception as e:
        return _err(e, 400)
    return _ok(**info)


@app.route("/api/stream")
def api_stream():
    def gen():
        while True:
            payload = json.dumps(service.status())
            yield f"data: {payload}\n\n"
            time.sleep(0.05)
    return Response(gen(), mimetype="text/event-stream",
                    headers={"Cache-Control": "no-cache",
                             "X-Accel-Buffering": "no"})


def _ssl_context():
    """Return an (certfile, keyfile) tuple for Flask if a TLS cert exists.

    Looks at MOTOR_CERT/MOTOR_KEY (override) or the default certs/ generated by
    gen_cert.sh. Returns None to fall back to plain HTTP.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    cert = os.environ.get("MOTOR_CERT", os.path.join(here, "certs", "server.crt"))
    key = os.environ.get("MOTOR_KEY", os.path.join(here, "certs", "server.key"))
    if os.path.isfile(cert) and os.path.isfile(key):
        return (cert, key)
    return None


if __name__ == "__main__":
    host = os.environ.get("HOST", "127.0.0.1")
    port = int(os.environ.get("PORT", "5000"))
    ssl_context = _ssl_context()
    scheme = "https" if ssl_context else "http"
    print(f"Serving motor dashboard over {scheme} on {host}:{port}")
    app.run(host=host, port=port, threaded=True, debug=False,
            ssl_context=ssl_context)
