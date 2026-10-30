"""End-to-end test: video file -> full pipeline -> dashboard API (no camera needed)."""
import time
from pathlib import Path

import pytest

from src.config import load_config

ROOT = Path(__file__).resolve().parent.parent
FALL_VIDEO = ROOT / "data/raw/public_datasets/GMDCSA24/Subject 2/Fall/02.mp4"


@pytest.fixture(scope="module")
def runner(tmp_path_factory):
    if not FALL_VIDEO.exists():
        pytest.skip("GMDCSA-24 not downloaded")
    from src.app_runner import AppRunner
    logs = tmp_path_factory.mktemp("logs")
    cfg = load_config(overrides={"camera": {"source": str(FALL_VIDEO)},
                                 "logging": {"directory": str(logs)},
                                 "alerts": {"beep_on_alert": False},
                                 "robot": {"enabled": False}})   # this test is about the app without a robot
    r = AppRunner(cfg)
    r.start()
    deadline = time.time() + 240
    while not r.finished and time.time() < deadline:
        time.sleep(0.2)
    yield r
    r.stop()


def test_pipeline_processes_whole_video(runner):
    st = runner.status()
    assert runner.error is None
    assert st["finished"] and st["frames_processed"] > 200
    assert st["camera"]["status"] in ("ended", "stopped")


def test_fall_video_raises_alert_and_logs_events(runner):
    types = [e["event_type"] for e in runner.recent_events(100)]
    assert "SYSTEM_START" in types
    assert any(t in types for t in ("FALL_SUSPECTED", "FALL_CONFIRMED")), types


def test_dashboard_endpoints(runner):
    from src.dashboard.server import create_app
    app = create_app(runner)
    c = app.test_client()
    assert c.get("/").status_code == 200
    st = c.get("/api/status").get_json()
    for key in ("camera", "fps", "persons", "alerts_active", "models", "inference_ms"):
        assert key in st
    ev = c.get("/api/events?n=5").get_json()
    assert isinstance(ev, list) and len(ev) <= 5
    n_active = len(st["alerts_active"])
    assert c.post("/api/ack", json={}).get_json()["acknowledged"] == n_active
    assert c.get("/api/status").get_json()["alerts_active"] == []
    assert c.post("/api/reset", json={"track_id": "x"}).status_code == 400
    assert c.post("/api/reset", json={}).get_json()["reset"] == "all"
    assert runner.latest_jpeg()[:2] == b"\xff\xd8"   # valid JPEG


def test_status_has_assessment_and_robot_disabled_actions_refused(runner):
    from src.analysis.assessment import LABELS
    from src.dashboard.server import create_app
    st = runner.status()
    assert st["robot"] == {"link_enabled": False}
    for p in st["persons"]:
        assert p["assessment"] in LABELS
    c = create_app(runner).test_client()
    r = c.post("/api/robot/stop", json={})
    assert r.status_code == 409 and "disabled" in r.get_json()["error"]
    assert c.post("/api/robot/jump", json={}).status_code == 404
    assert c.post("/api/robot/manual", data="x" * 2000, content_type="application/json").status_code == 413


@pytest.fixture(scope="module")
def robot_runner(tmp_path_factory):
    """App with the robot link enabled, talking to the Python simulator."""
    if not FALL_VIDEO.exists():
        pytest.skip("GMDCSA-24 not downloaded")
    from robot.sim_robot import SimRobotServer
    from src.app_runner import AppRunner
    sim = SimRobotServer(port=0).start()
    logs = tmp_path_factory.mktemp("logs_robot")
    cfg = load_config(overrides={"camera": {"source": str(FALL_VIDEO), "file_realtime": True},
                                 "logging": {"directory": str(logs)}, "alerts": {"beep_on_alert": False},
                                 "robot": {"enabled": True, "host": "127.0.0.1", "port": sim.port}})
    r = AppRunner(cfg, max_frames=400)
    r.start()
    assert r.robot.client.wait_connected(5) and r.robot.client.wait_status(5)
    yield r, sim
    r.stop()
    sim.stop()


def test_app_tells_the_esp32_that_the_model_runs(robot_runner):
    """The running AI sends MODEL_STARTED to the robot once (not once per frame)."""
    r, sim = robot_runner
    deadline = time.time() + 10
    while time.time() < deadline and not any('"MODEL_STARTED"' in l for l in sim.received):
        time.sleep(0.1)
    time.sleep(1.0)                                   # many more frames are processed meanwhile
    assert sum('"MODEL_STARTED"' in l for l in sim.received) == 1
    assert sim.core.model_running
    assert r.robot.status()["ai_state"]["model_running"]


def test_dashboard_robot_controls(robot_runner):
    from src.dashboard.server import create_app
    r, sim = robot_runner
    c = create_app(r).test_client()
    post = lambda a, body=None: c.post(f"/api/robot/{a}", json=body or {})
    st = c.get("/api/status").get_json()["robot"]
    assert st["link_enabled"] and st["connected"] and "safety" in st and "commanded" in st and "reported" in st
    assert post("buzzer_test").status_code == 200
    assert post("led_blink").status_code == 200
    m = post("manual", {"direction": "forward"})
    assert m.status_code in (200, 409)        # 409 if a fall warning from the video is active
    assert post("manual", {"direction": "up"}).status_code == 409
    assert post("follow_select", {"track_id": "x"}).status_code == 409
    assert post("follow_select", {"track_id": 999}).status_code == 409
    assert post("estop").status_code == 200
    time.sleep(0.3)
    assert sim.core.state == "EMERGENCY"
    assert post("clear_emergency", {}).status_code == 409                 # needs confirmation
    assert post("clear_emergency", {"operator_confirmed": True}).status_code == 200
    assert sim.core.state == "IDLE"
    assert post("stop").status_code == 200
    assert post("disconnect").status_code == 200
    time.sleep(0.3)
    assert not r.robot.client.connected and sim.core.state == "IDLE"
    assert post("connect").status_code == 200
    assert r.robot.client.wait_connected(5)
    types = [e["event_type"] for e in r.recent_events(200)]
    assert "ROBOT_EMERGENCY_SENT" in types and "OPERATOR_RESET" in types and "ROBOT_STOP" in types
