"""The per-frame AI pipeline (no user-interface code here).

frame -> pose detection -> (object detection every N frames) -> tracking
      -> per-track posture -> per-track fall state machine -> events/alerts/log
"""
from __future__ import annotations

import logging
import queue
import time
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from .alerts.alert_manager import AlertManager
from .analysis.fall_detector import FallDetector, FallParams, TrainedFallModel
from .analysis.posture import PostureClassifier
from .camera.stream import Frame
from .config import Config, resolve_path
from .detection.yolo_onnx import ObjectDetection, ObjectDetector, PoseDetector
from .logging_system.event_logger import EventLogger
from .tracking.tracker import PersonTracker

log = logging.getLogger("sentinel.pipeline")

LOGGED_EVENTS = {"POSSIBLE_FALL", "FALL_SUSPECTED", "FALL_CONFIRMED", "FALL_DISMISSED",
                 "RECOVERED", "TRACK_LOST_DURING_FALL", "TRACK_HANDOVER"}


@dataclass
class PersonState:
    track_id: int
    box: np.ndarray
    keypoints: np.ndarray
    score: float
    posture: str
    posture_conf: float
    posture_source: str
    reliability: float
    fall_state: str
    fall_score: float
    confirmed: bool


@dataclass
class FrameResult:
    frame_id: int
    timestamp: float
    persons: list[PersonState] = field(default_factory=list)
    objects: list[ObjectDetection] = field(default_factory=list)
    events: list[dict] = field(default_factory=list)
    inference_ms: float = 0.0
    total_ms: float = 0.0
    fps: float = 0.0
    camera_status: str = "ok"
    frame_size: tuple = (0, 0)      # (width, height) of the analysed image


class SentinelPipeline:
    def __init__(self, cfg: Config, event_logger: EventLogger | None = None,
                 alerts: AlertManager | None = None, load_objects: bool | None = None):
        self.cfg = cfg
        d, m = cfg.detection, cfg.models
        providers = list(m.providers)
        self.pose = PoseDetector(resolve_path(m.pose_model), d.pose_imgsz, d.person_conf, d.nms_iou,
                                 providers, m.num_threads)
        use_objects = d.object_detection_enabled if load_objects is None else load_objects
        self.objects_detector = (ObjectDetector(resolve_path(m.object_model), d.object_imgsz,
                                                d.object_conf, d.nms_iou, list(d.object_classes),
                                                providers, m.num_threads) if use_objects else None)
        t = cfg.tracking
        self.tracker = PersonTracker(t.iou_match_threshold, t.max_center_distance, t.min_hits,
                                     t.max_missed_s, t.history_s)
        p = cfg.posture
        self.posture = PostureClassifier(resolve_path(m.posture_model), p.use_trained_model,
                                         d.keypoint_conf, p.min_visible_keypoints,
                                         p.unknown_proba_threshold, p.smoothing_window)
        if self.posture.model_error:
            log.warning("Posture model not used (%s); using rule baseline", self.posture.model_error)
        fall_model = None
        self.fall_model_error = None
        if cfg.fall.use_trained_model:
            try:
                fall_model = TrainedFallModel(resolve_path(m.fall_model))
            except Exception as exc:
                self.fall_model_error = f"{type(exc).__name__}: {exc}"
                log.warning("Fall model not used (%s); using rule baseline", self.fall_model_error)
        self.fall = FallDetector(FallParams.from_config(cfg.fall), fall_model, d.keypoint_conf)
        self.logger = event_logger
        self.alerts = alerts or AlertManager(cfg.alerts.require_acknowledgement, False)
        self._last_objects: list[ObjectDetection] = []
        # object detection rate; person-following raises it to see obstacles sooner
        self.object_every_n = int(d.object_every_n_frames)
        self._frame_times: deque = deque(maxlen=30)
        self._commands: queue.Queue = queue.Queue()
        self._last_t = 0.0
        self.camera_status = "ok"

    # ------------------------------------------------------------ control API
    def request_reset(self, track_id: int | None = None) -> None:
        """Thread-safe: reset fall state(s) on the next processed frame."""
        self._commands.put(("reset", track_id))

    def on_camera_status(self, status: str) -> None:
        if status != "ok" and self.camera_status == "ok":
            log.warning("Camera status %s: pausing fall evidence", status)
            self.fall.stream_interrupted()
            self.tracker.reset()
            if self.logger:
                self.logger.log("CAMERA_INTERRUPTED", reason=f"camera status {status}",
                                stream_time_s=self._last_t)
        elif status == "ok" and self.camera_status != "ok" and self.logger:
            self.logger.log("CAMERA_RESTORED", reason="frames received again", stream_time_s=self._last_t)
        self.camera_status = status

    def _apply_commands(self, t: float) -> None:
        while True:
            try:
                cmd, arg = self._commands.get_nowait()
            except queue.Empty:
                return
            if cmd == "reset":
                self.fall.reset(arg, t)
                if self.logger:
                    self.logger.log("OPERATOR_RESET", track_id=arg, reason="reset from dashboard",
                                    stream_time_s=t)

    # --------------------------------------------------------------- per frame
    def process(self, frame: Frame) -> FrameResult:
        t0 = time.perf_counter()
        t = frame.timestamp
        self._last_t = t
        self._apply_commands(t)
        img = frame.image
        h, w = img.shape[:2]
        res = FrameResult(frame.frame_id, t, camera_status=self.camera_status, frame_size=(w, h))

        dets = self.pose.detect(img)
        inf_ms = self.pose.last_timing.total_ms
        if self.objects_detector and (frame.frame_id % self.object_every_n == 1 or self.object_every_n == 1):
            self._last_objects = self.objects_detector.detect(img)
            inf_ms += self.objects_detector.last_timing.total_ms
        res.objects = self._last_objects
        visible = self.tracker.update(dets, t, (w, h))

        for tid in self.tracker.removed_ids:
            self.posture.forget(tid)
            res.events += self.fall.track_lost(tid, t)

        for tr in visible:
            posture, feats = self.posture.classify(tr.track_id, tr.keypoints, tr.box)
            if tr.confirmed:
                st, events = self.fall.update(tr.track_id, t, list(tr.history), posture.label, feats)
                res.events += events
                state, score = st.state, st.last_score
            else:
                state, score = "NORMAL", 0.0
            res.persons.append(PersonState(tr.track_id, tr.box, tr.keypoints, tr.score, posture.label,
                                           posture.confidence, posture.source, posture.reliability,
                                           state, score, tr.confirmed))

        res.events += self.fall.resolve_handover(
            t, {tr.track_id: tr.box for tr in visible if tr.confirmed})
        # a re-linked fall must show its state on the new id in this frame's results
        for ps in res.persons:
            st = self.fall.tracks.get(ps.track_id)
            if st is not None:
                ps.fall_state = st.state

        for ev in res.events:
            ev.setdefault("t", t)
            alert = self.alerts.handle_event(ev)
            if self.logger and ev["type"] in LOGGED_EVENTS:
                self.logger.log(ev["type"], track_id=ev.get("track_id"),
                                previous_posture=ev.get("previous_posture", ""),
                                current_posture=ev.get("current_posture", ""),
                                fall_state=ev.get("state", ""), confidence=ev.get("score"),
                                reason=ev.get("reason", ""), stream_time_s=t,
                                extra={"alert_id": alert.alert_id} if alert else None)

        now = time.perf_counter()
        self._frame_times.append(now)
        if len(self._frame_times) >= 2:
            res.fps = (len(self._frame_times) - 1) / max(self._frame_times[-1] - self._frame_times[0], 1e-6)
        res.inference_ms = inf_ms
        res.total_ms = (now - t0) * 1000
        return res

    def status(self) -> dict:
        if not hasattr(self, "_versions"):
            self._versions = _model_versions()
        return {
            "versions": self._versions,
            "posture_mode": self.posture.mode, "posture_model_error": self.posture.model_error,
            "fall_mode": self.fall.mode, "fall_model_error": self.fall_model_error,
            "providers": self.pose.providers,
            "objects_enabled": self.objects_detector is not None,
        }


def _model_versions() -> dict:
    """Deployed model versions from artifacts/model_registry.json (training/model_registry.py)."""
    import json
    try:
        reg = json.loads(resolve_path("artifacts/model_registry.json").read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    names = {"posture_classifier": "posture", "fall_classifier": "fall", "pose_estimation": "pose",
             "object_detection": "objects"}
    out = {}
    for kind, e in reg.get("models", {}).items():
        v = e.get("deployed")
        if v and kind in names:
            out[names[kind]] = f"{kind} {v} ({e['versions'][v]['sha256'][:8]})"
    return out
