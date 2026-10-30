import json
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

from src.alerts.alert_manager import AlertManager
from src.camera.stream import CameraStream
from src.config import DEFAULT_CONFIG_PATH, ConfigError, load_config, parse_source, validate
from src.detection.yolo_onnx import ObjectDetector, PoseDetector, letterbox
from src.logging_system.event_logger import EventLogger

ROOT = Path(__file__).resolve().parent.parent


# ------------------------------------------------------------------ config
def test_default_config_is_valid():
    cfg = load_config()
    assert cfg.fall.window_s > 0
    assert cfg["detection"]["person_conf"] == cfg.detection.person_conf


def test_config_overrides_and_errors(tmp_path):
    cfg = load_config(overrides={"camera": {"source": "video.mp4"}})
    assert cfg.camera.source == "video.mp4"
    data = yaml.safe_load(DEFAULT_CONFIG_PATH.read_text())
    data["detection"]["person_conf"] = 1.5
    data["fall"]["model_threshold"] = "sometimes"
    del data["tracking"]["min_hits"]
    problems = validate(data)
    assert any("person_conf" in p for p in problems)
    assert any("model_threshold" in p for p in problems)
    assert any("min_hits" in p for p in problems)
    bad = tmp_path / "bad.yaml"
    bad.write_text(yaml.safe_dump(data))
    with pytest.raises(ConfigError):
        load_config(bad)
    with pytest.raises(ConfigError):
        load_config(tmp_path / "missing.yaml")


def test_parse_source():
    assert parse_source("0") == 0 and parse_source(1) == 1
    assert parse_source("http://1.2.3.4:8080/video") == "http://1.2.3.4:8080/video"


# --------------------------------------------------------------- logging
def test_event_logger_writes_jsonl_and_csv(tmp_path):
    lg = EventLogger(tmp_path)
    lg.log("FALL_CONFIRMED", track_id=3, previous_posture="STANDING", current_posture="LYING",
           fall_state="FALL_CONFIRMED", confidence=0.91, reason="test")
    lg.log("RECOVERED", track_id=3)
    recs = lg.read_recent(10)
    assert [r["event_type"] for r in recs] == ["FALL_CONFIRMED", "RECOVERED"]
    assert recs[0]["confidence"] == 0.91 and len(recs[0]["event_id"]) == 12
    lines = (tmp_path / "events.csv").read_text().strip().splitlines()
    assert lines[0].startswith("timestamp,event_id") and len(lines) == 3


# ----------------------------------------------------------------- alerts
def test_alerts_dedup_ack_and_listener_safety():
    am = AlertManager()
    received = []
    am.add_listener(lambda a: received.append(a))
    am.add_listener(lambda a: 1 / 0)          # a broken listener must not break alerting
    ev = {"type": "FALL_CONFIRMED", "track_id": 1, "reason": "x"}
    a1 = am.handle_event(ev)
    assert a1 is not None and a1.severity == "emergency"
    assert am.handle_event(ev) is None        # duplicate while active
    assert am.handle_event({"type": "POSSIBLE_FALL", "track_id": 1}) is None
    assert am.emergency_active and len(received) == 1
    assert am.acknowledge(a1.alert_id) == 1
    assert not am.emergency_active
    assert am.handle_event(ev) is not None    # new alert after acknowledgement


# ----------------------------------------------------------------- camera
def _make_video(path: Path, n=12, fps=10):
    wr = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), fps, (64, 48))
    for i in range(n):
        wr.write(np.full((48, 64, 3), i * 10, np.uint8))
    wr.release()


def test_camera_reads_file_until_end(tmp_path):
    vid = tmp_path / "v.avi"
    _make_video(vid)
    cam = CameraStream(str(vid))
    assert cam.open() and cam.status == "ok"
    frames = []
    while (f := cam.read()) is not None:
        frames.append(f)
    assert len(frames) == 12 and cam.status == "ended"
    assert frames[1].timestamp == pytest.approx(0.1, abs=1e-6)
    cam.release()


def test_camera_missing_file_reports_error(tmp_path):
    cam = CameraStream(str(tmp_path / "nope.mp4"))
    assert not cam.open()
    assert cam.status == "error" and "not found" in cam.last_error


def test_camera_unreachable_stream_fails_cleanly():
    cam = CameraStream("http://127.0.0.1:9/video", stall_timeout_s=0.5, reconnect_delay_s=0.1,
                       max_reconnect_attempts=1)
    assert not cam.open()
    assert cam.status == "error"
    cam.release()
    assert cam.status == "stopped"


# --------------------------------------------------------------- detectors
def test_letterbox_maps_back():
    img = np.zeros((360, 640, 3), np.uint8)
    out, scale, (px, py) = letterbox(img, 640)
    assert out.shape == (640, 640, 3) and scale == 1.0 and px == 0 and py == 140


@pytest.fixture(scope="module")
def cfg():
    return load_config()


def test_pose_detector_on_dataset_frame(cfg):
    vids = sorted((ROOT / "data/raw/public_datasets/GMDCSA24").glob("Subject 2/ADL/01.mp4"))
    if not vids:
        pytest.skip("GMDCSA-24 not downloaded")
    cap = cv2.VideoCapture(str(vids[0]))
    for _ in range(30):
        ok, frame = cap.read()
    cap.release()
    det = PoseDetector(ROOT / cfg.models.pose_model)
    people = det.detect(frame)
    assert people, "expected the person on the bed to be detected"
    p = people[0]
    assert p.keypoints.shape == (17, 3) and p.score > 0.5
    h, w = frame.shape[:2]
    assert 0 <= p.box[0] < p.box[2] <= w and 0 <= p.box[1] < p.box[3] <= h
    assert det.detect(np.zeros((0, 0, 3), np.uint8)) == []


def test_object_detector_validates_classes(cfg):
    with pytest.raises(ValueError):
        ObjectDetector(ROOT / cfg.models.object_model, classes=["dragon"])
    od = ObjectDetector(ROOT / cfg.models.object_model, classes=["bed", "chair"])
    assert od.detect(np.zeros((480, 640, 3), np.uint8)) == []


def test_missing_model_file_message(tmp_path):
    with pytest.raises(FileNotFoundError):
        PoseDetector(tmp_path / "missing.onnx")
