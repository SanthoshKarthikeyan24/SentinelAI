"""Runs the camera + AI pipeline on a background thread and exposes its latest state.

The dashboard (or an OpenCV window, or nothing in headless mode) only READS
from this object, so the user interface never blocks inference and inference
never blocks the user interface.
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Any

import cv2

from .alerts.alert_manager import AlertManager
from .alerts.clip_buffer import ClipBuffer
from .analysis.assessment import TRACKING_LOST, assess
from .camera.stream import CameraStream, describe_source
from .config import Config, parse_source, resolve_path
from .logging_system.event_logger import EventLogger
from .pipeline import FrameResult, SentinelPipeline
from .utils.drawing import draw_result

log = logging.getLogger("sentinel.runner")


class AppRunner:
    def __init__(self, cfg: Config, max_frames: int = 0):
        self.cfg = cfg
        self.max_frames = int(max_frames)
        self.source = parse_source(cfg.camera.source)
        self.logger = EventLogger(resolve_path(cfg.logging.directory), cfg.logging.events_jsonl,
                                  cfg.logging.events_csv)
        self.alerts = AlertManager(cfg.alerts.require_acknowledgement, cfg.alerts.beep_on_alert)
        cb = dict(cfg.logging.get("clip_buffer") or {})
        self.clips = ClipBuffer(resolve_path(cfg.logging.directory) / "clips", bool(cb.get("enabled", False)),
                                cb.get("pre_s", 5.0), cb.get("post_s", 5.0), cb.get("fps", 10),
                                cb.get("max_clips", 20))
        self.alerts.add_listener(lambda a: self.clips.trigger(f"{a.event_type}_track{a.track_id}"))
        self.pipeline = SentinelPipeline(cfg, self.logger, self.alerts)
        self.robot = None
        if cfg.robot.enabled:
            from robot.robot_logic import create_robot_link
            self.robot = create_robot_link(cfg.robot, self.logger)
            # confirmed falls (emergency alerts) are forwarded to the robot with top priority
            self.alerts.add_listener(self.robot.on_alert)
        self.follow = None
        if self.robot is not None and cfg.robot.follow.get("enabled", True):
            from robot.follow import FollowManager, FollowParams
            self.follow = FollowManager(self.robot, FollowParams.from_config(cfg.robot.follow),
                                        self.pipeline, self.logger)
        c = cfg.camera
        self.camera = CameraStream(self.source, c.width, c.height, c.stall_timeout_s, c.reconnect_delay_s,
                                   c.reconnect_max_delay_s, c.max_reconnect_attempts, c.file_realtime)
        self._lock = threading.Lock()
        self._jpeg: bytes | None = None
        self._annotated = None
        self._result: FrameResult | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.frames_processed = 0
        self.started_at = time.time()
        self.finished = False
        self.error: str | None = None

    # -------------------------------------------------------------- lifecycle
    def start(self) -> None:
        log.info("Opening %s", describe_source(self.source))
        self.logger.log("SYSTEM_START", reason=f"source={describe_source(self.source)}; "
                                               f"posture={self.pipeline.posture.mode}; fall={self.pipeline.fall.mode}")
        if self.robot is not None:
            from robot.discovery import is_auto
            if is_auto(self.cfg.robot.host):
                log.info("Robot link enabled: searching the Wi-Fi for the ESP32 (port %d) ...", self.cfg.robot.port)
            else:
                log.info("Robot link enabled: %s:%d", self.cfg.robot.host, self.cfg.robot.port)
            self.robot.start()
        opened = self.camera.open()
        if not opened:
            log.warning("Camera not available yet (%s); will keep retrying for live sources",
                        self.camera.last_error or self.camera.status)
        self._thread = threading.Thread(target=self._loop, name="ai-pipeline", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=5)
        self.camera.release()
        if self.robot is not None:
            self.robot.close()          # safe shutdown: stop following, send stop, close the link
        self.logger.log("SYSTEM_STOP", reason=f"frames processed={self.frames_processed}")

    def wait(self, timeout: float | None = None) -> None:
        if self._thread:
            self._thread.join(timeout)

    # ------------------------------------------------------------------- loop
    def _loop(self) -> None:
        q = int(self.cfg.dashboard.jpeg_quality)
        try:
            while not self._stop.is_set():
                frame = self.camera.read(timeout=1.0)
                self.pipeline.on_camera_status(self.camera.status if frame is None else "ok")
                if frame is None:
                    if self.camera.status in ("ended",) or (self.camera.status == "error" and not self.camera.live):
                        log.info("Source finished (%s)", self.camera.status)
                        break
                    if self.camera.status == "error" and self.camera.live and self.camera._thread \
                            and not self.camera._thread.is_alive():
                        self.error = self.camera.last_error
                        log.error("Camera gave up: %s", self.camera.last_error)
                        break
                    continue
                result = self.pipeline.process(frame)
                if self.robot is not None:
                    active = self.alerts.active()
                    states = {p.fall_state for p in result.persons}
                    # ESP32 LED/buzzer mirror the AI state; RobotLink only sends on a change of state
                    self.robot.set_model_running(True, "AI pipeline is processing frames")
                    self.robot.set_fall_alert(any(a.severity == "emergency" for a in active),
                                              "confirmed fall alert active" if active else "no active fall alert")
                    self.robot.set_fall_flags(
                        suspected=any(a.severity == "warning" for a in active) or "FALL_SUSPECTED" in states,
                        confirmed=any(a.severity == "emergency" for a in active) or "FALL_CONFIRMED" in states)
                    if any(e["type"] in ("RECOVERED", "FALL_DISMISSED") for e in result.events) and \
                            not any(a.severity == "warning" for a in active):
                        self.robot.clear_warning("person recovered / possible fall dismissed")
                if self.follow is not None:
                    # age of the frame: following must never act on old pictures
                    age = max(0.0, time.time() - frame.timestamp) if self.camera.live else 0.0
                    try:
                        self.follow.on_frame(result, result.frame_size, age)
                    except Exception as exc:   # following must never stop the fall detection
                        log.error("Following failed, stopping it: %s", exc)
                        self.follow.stop(f"error: {exc}")
                annotated = draw_result(frame.image, result, self.cfg.detection.keypoint_conf)
                self.clips.add(annotated)
                ok, buf = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, q])
                with self._lock:
                    self._result = result
                    self._annotated = annotated
                    if ok:
                        self._jpeg = buf.tobytes()
                self.frames_processed += 1
                if self.max_frames and self.frames_processed >= self.max_frames:
                    break
        except Exception as exc:   # never die silently
            self.error = f"{type(exc).__name__}: {exc}"
            log.exception("Pipeline crashed")
            self.logger.log("SYSTEM_ERROR", reason=self.error)
        finally:
            self.finished = True
            if self.robot is not None:
                self.robot.set_model_running(False, self.error or "AI processing stopped")

    # -------------------------------------------------------------- read API
    def latest_jpeg(self) -> bytes | None:
        with self._lock:
            return self._jpeg

    def latest_frame(self):
        with self._lock:
            return self._annotated

    def status(self) -> dict[str, Any]:
        with self._lock:
            r = self._result
        persons = []
        if r:
            for p in r.persons:
                persons.append({"track_id": p.track_id, "posture": p.posture,
                                "assessment": assess(p.fall_state, p.posture, p.reliability),
                                "box": [round(float(v), 1) for v in p.box],
                                "posture_confidence": round(p.posture_conf, 3),
                                "posture_source": p.posture_source,
                                "keypoint_reliability": round(p.reliability, 3),
                                "fall_state": p.fall_state, "fall_score": round(p.fall_score, 3),
                                "detection_score": round(p.score, 3), "confirmed_track": p.confirmed})
        for a in self.alerts.active():
            if a.event_type == "TRACK_LOST_DURING_FALL" and not any(q["track_id"] == a.track_id for q in persons):
                persons.append({"track_id": a.track_id, "assessment": TRACKING_LOST, "posture": "UNKNOWN",
                                "fall_state": "LOST", "box": None})
        return {
            "frame_size": list(r.frame_size) if r else None,
            "system": _system_load(),
            "clips": self.clips.status(),
            "camera": {"source": describe_source(self.source), "status": self.camera.status,
                       "last_error": self.camera.last_error, "reconnects": self.camera.reconnects},
            "fps": round(r.fps, 2) if r else 0.0,
            "inference_ms": round(r.inference_ms, 1) if r else 0.0,
            "frame_ms": round(r.total_ms, 1) if r else 0.0,
            "frames_processed": self.frames_processed,
            "persons": persons,
            "objects": [{"class": o.class_name, "score": round(o.score, 2)} for o in (r.objects if r else [])],
            "alerts_active": [a.to_dict() for a in self.alerts.active()],
            "emergency": self.alerts.emergency_active,
            "models": self.pipeline.status(),
            "robot": self.robot.status() if self.robot is not None else {"link_enabled": False},
            "follow": self.follow.status() if self.follow is not None else {"available": False},
            "finished": self.finished, "error": self.error,
            "uptime_s": round(time.time() - self.started_at, 1),
        }

    def recent_events(self, n: int = 50) -> list[dict]:
        return list(reversed(self.logger.read_recent(n)))

    def acknowledge(self, alert_id: str | None = None) -> int:
        alerts = [a for a in self.alerts.active() if alert_id is None or a.alert_id == alert_id]
        n = self.alerts.acknowledge(alert_id)
        for a in alerts:
            self.logger.log("ALERT_ACKNOWLEDGED", track_id=a.track_id, reason=a.message,
                            acknowledged=True, extra={"alert_id": a.alert_id})
            if a.severity == "emergency":
                self.pipeline.request_reset(a.track_id)
        if self.robot is not None and alerts and not any(a.severity == "warning" for a in self.alerts.active()):
            self.robot.clear_warning("warning acknowledged")
        if self.robot is not None and not any(a.severity == "emergency" for a in self.alerts.active()):
            self.robot.set_fall_alert(False, "operator acknowledged the fall alert")
        return n

    def reset(self, track_id: int | None = None) -> None:
        self.pipeline.request_reset(track_id)

    # ------------------------------------------------------------- robot API
    ROBOT_ACTIONS = ("connect", "disconnect", "stop", "estop", "emergency", "mute", "clear_emergency",
                     "manual", "led_on", "led_off", "led_blink", "buzzer_test", "follow_select", "follow_resume",
                     "follow_stop", "ack_comm_fault")

    def robot_command(self, action: str, params: dict | None = None) -> dict:
        """Operator robot actions from the dashboard."""
        params = params or {}
        if self.robot is None:
            return {"ok": False, "error": "robot link disabled (robot.enabled: false in settings.yaml)"}
        from robot.commands import CommandError
        from robot.wifi_client import RobotNotConnected, RobotTimeout
        link = self.robot
        try:
            if action == "connect":
                link.client.set_enabled(True)
                return {"ok": True, "reply": "connecting"}
            if action == "disconnect":
                if self.follow is not None:
                    self.follow.disable("operator disconnected", send=False)
                link.client.set_enabled(False)
                return {"ok": True, "reply": "stop sent, link closed"}
            if action in ("estop", "emergency"):
                link.emergency("operator", "operator E-STOP button on dashboard")
                return {"ok": True, "reply": "emergency_stop sent (retried until the robot acknowledges)"}
            if action == "manual":
                d = link.manual(str(params.get("direction", "")))
                return {"ok": d.sent, "reply": d.reason} if d.sent else {"ok": False, "error": d.reason}
            if action == "follow_select":
                if self.follow is None:
                    return {"ok": False, "error": "following disabled (robot.follow.enabled: false)"}
                tid = params.get("track_id")
                if not isinstance(tid, int) or isinstance(tid, bool):
                    return {"ok": False, "error": "track_id must be an integer"}
                with self._lock:
                    persons = list(self._result.persons) if self._result else []
                ok, msg = self.follow.select(tid, persons)
                return {"ok": ok, "reply": msg} if ok else {"ok": False, "error": msg}
            if action == "follow_resume":
                if self.follow is None:
                    return {"ok": False, "error": "following disabled"}
                ok, msg = self.follow.resume()
                return {"ok": ok, "reply": msg} if ok else {"ok": False, "error": msg}
            if action == "follow_stop":
                if self.follow is not None:
                    self.follow.disable("operator button on dashboard")
                return {"ok": True, "reply": "following stopped"}
            if action == "ack_comm_fault":
                ok, msg = link.acknowledge_comm_fault()
                return {"ok": ok, "reply": msg} if ok else {"ok": False, "error": msg}
            if action == "clear_emergency":
                if params.get("operator_confirmed") is not True:
                    return {"ok": False, "error": "confirmation required"}
                r = link.clear_emergency(True)
                if r.ok:
                    self.logger.log("OPERATOR_RESET", reason="operator cleared the robot emergency",
                                    acknowledged=True, extra={"reset_time": time.strftime("%Y-%m-%dT%H:%M:%S")})
            elif action == "stop":
                r = link.stop("operator STOP button on dashboard")
            elif action == "mute":
                r = link.mute()
            elif action == "led_on":
                r = link.set_led("on")
            elif action == "led_off":
                r = link.set_led("off")
            elif action == "led_blink":
                r = link.set_led("blink", 500)
            elif action == "buzzer_test":
                r = link.buzzer("short_beep")
            else:
                return {"ok": False, "error": f"unknown action {action}"}
        except (RobotNotConnected, RobotTimeout, CommandError) as exc:
            return {"ok": False, "error": str(exc)}
        return {"ok": r.ok, "reply": r.raw} if r.ok else {"ok": False, "error": r.error or r.raw}


def _system_load() -> dict:
    try:
        import psutil
        p = psutil.Process()
        return {"cpu_percent": psutil.cpu_percent(interval=None), "memory_percent": psutil.virtual_memory().percent,
                "process_memory_mb": round(p.memory_info().rss / 1e6, 1)}
    except Exception:          # psutil is optional
        return {}
