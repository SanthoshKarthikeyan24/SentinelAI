"""Local web dashboard (Flask). Open http://127.0.0.1:5000 in a browser.

Routes
  GET  /             dashboard page
  GET  /video_feed   MJPEG stream of the annotated video
  GET  /api/status   JSON: camera, FPS, people, postures, fall states, alerts
  GET  /api/events   JSON: recent logged events
  POST /api/ack      acknowledge alerts   body: {"alert_id": "..."} (omit = all)
  POST /api/reset    reset fall state      body: {"track_id": 3}     (omit = all)
  POST /api/robot/<action>   robot operator actions (JSON body for parameters):
        connect | disconnect | stop | estop | mute | clear_emergency {"operator_confirmed": true} |
        manual {"direction": "forward|backward|left|right"} (repeat every ~150 ms while held) |
        led_on | led_off | led_blink | buzzer_test |
        follow_select {"track_id": 3} | follow_resume | follow_stop | ack_comm_fault

The dashboard only listens on 127.0.0.1 by default (dashboard.host); do not expose it
or the robot port to the internet.
"""
from __future__ import annotations

import logging
import time
from pathlib import Path

from flask import Flask, Response, jsonify, request, send_file

from ..app_runner import AppRunner

PAGE = Path(__file__).with_name("index.html")


def create_app(runner: AppRunner) -> Flask:
    app = Flask(__name__)
    logging.getLogger("werkzeug").setLevel(logging.WARNING)

    @app.get("/")
    def index():
        return send_file(PAGE)

    @app.get("/video_feed")
    def video_feed():
        def gen():
            last = None
            idle_since = time.time()
            while True:
                jpg = runner.latest_jpeg()
                if jpg is not None and jpg is not last:
                    last = jpg
                    idle_since = time.time()
                    yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
                elif runner.finished and time.time() - idle_since > 2:
                    return
                time.sleep(0.03)
        return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.get("/api/status")
    def status():
        return jsonify(runner.status())

    @app.get("/api/events")
    def events():
        n = min(int(request.args.get("n", runner.cfg.dashboard.max_recent_events)), 500)
        return jsonify(runner.recent_events(n))

    @app.post("/api/ack")
    def ack():
        body = request.get_json(silent=True) or {}
        n = runner.acknowledge(body.get("alert_id"))
        return jsonify({"acknowledged": n})

    @app.post("/api/reset")
    def reset():
        body = request.get_json(silent=True) or {}
        tid = body.get("track_id")
        if tid is not None and not isinstance(tid, int):
            return jsonify({"error": "track_id must be an integer"}), 400
        runner.reset(tid)
        return jsonify({"reset": "all" if tid is None else tid})

    @app.post("/api/robot/<action>")
    def robot_action(action):
        if action not in AppRunner.ROBOT_ACTIONS:
            return jsonify({"ok": False, "error": "unknown action"}), 404
        if request.content_length and request.content_length > 1024:
            return jsonify({"ok": False, "error": "request too large"}), 413
        body = request.get_json(silent=True) or {}
        if not isinstance(body, dict):
            return jsonify({"ok": False, "error": "body must be a JSON object"}), 400
        result = runner.robot_command(action, body)
        return jsonify(result), (200 if result.get("ok") else 409)

    return app
