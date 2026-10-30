"""Stage 1 communication test: ESP32Client facade and the test console, against the real firmware
core (C++ simulator) and the Python simulator."""
import json
import subprocess
import sys
import time
from pathlib import Path

import pytest

from robot.commands import CommandError, NAMED_COMMANDS, make_command
from robot.esp32_client import ESP32Client
from robot.sim_robot import SimRobotServer

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_robot_comm import _CppSim, _PySim, cpp_sim_binary_or_none, robot  # noqa: E402,F401


@pytest.fixture
def esp(robot):
    e = ESP32Client("127.0.0.1", robot.port, ack_timeout_s=1.0)
    assert e.connect(timeout=5)
    yield e
    e.disconnect()


def test_named_command_schema():
    assert len(NAMED_COMMANDS) == 18
    for ev in ("MODEL_STARTED", "MODEL_STOPPED", "FALL_DETECTED", "FALL_CLEARED"):
        assert ev in NAMED_COMMANDS
        make_command("command", command=ev)
    with pytest.raises(CommandError):
        make_command("command", command="FALL_DETECTED", duration_ms=500)
    make_command("command", command="LEFT", duration_ms=500)
    for bad in (dict(command="JUMP"), dict(command="left"), dict(command="STOP", duration_ms=300),
                dict(command="LEFT", duration_ms=5000)):
        with pytest.raises(CommandError):
            make_command("command", **bad)


def test_every_command_is_acknowledged(esp):
    expect = {"FORWARD": "SIMULATED_MOVEMENT_RECEIVED: FORWARD", "LEFT": "SIMULATED_MOVEMENT_RECEIVED: LEFT",
              "STOP": "STOPPED", "LED_ON": "LED_ON", "BUZZER_SHORT_BEEP": "BUZZER_SHORT_BEEP",
              "STATUS": "STATUS_SENT"}
    for name, result in expect.items():
        r = esp.send_command(name)
        assert r.ok and r.result == result and r.command_id and r.latency_ms is not None, r
    assert not esp.send_command("JUMP").ok                     # refused locally, never sent


def test_simulated_movement_reported_and_never_drives(esp):
    assert esp.send_simulated_movement("right").ok
    st = esp.get_status()
    assert st["simulated_movement"] == "right" and st["motors_stopped"] and st["left"] == 0
    assert st["motors_enabled"] is False
    assert esp.client.commanded["simulated_movement"] == "right"
    assert esp.stop().ok and esp.get_status()["simulated_movement"] == "none"
    assert not esp.send_simulated_movement("jump").ok


def test_led_buzzer_status(esp):
    assert esp.set_led("blink").ok and esp.get_status()["led"] == "blink"
    assert esp.set_led("off").ok and esp.get_status()["led"] == "off"
    assert esp.set_buzzer("on").ok and esp.get_status()["buzzer"] == "on"
    assert esp.set_buzzer("off").ok and esp.get_status()["buzzer"] == "off"
    assert not esp.set_led("purple").ok and not esp.set_buzzer("loud").ok
    h = esp.hello()
    assert h["led_installed"] is True and h["buzzer_installed"] is True


def test_emergency_needs_explicit_reset(esp):
    assert esp.emergency_stop().ok
    assert esp.client.emergency_latched
    r = esp.send_simulated_movement("FORWARD")
    assert not r.ok and "EMERGENCY_ACTIVE" in r.error
    assert not esp.reset_emergency(confirmed=False).ok
    st = esp.get_status()
    assert st["emergency_active"] and st["led"] == "on"
    assert esp.reset_emergency(confirmed=True).ok and not esp.client.emergency_latched
    assert esp.get_status()["emergency_active"] is False
    assert esp.get_status()["simulated_movement"] == "none"    # nothing resumes


def test_lost_connection_clears_simulated_movement_and_nothing_resumes(esp):
    assert esp.send_simulated_movement("LEFT").ok
    esp.client._disconnect("test: Wi-Fi drop")
    end = time.monotonic() + 15            # generous: the reconnect waits ~1 s, the first status ~1 s
    while time.monotonic() < end and not (esp.connected and esp.responding() and esp.hello()):
        time.sleep(0.05)
    assert esp.connected
    assert esp.hello()["comm_fault"] is True
    st = esp.get_status()
    assert st["simulated_movement"] == "none" and st["watchdog_ok"] is False
    assert esp.client.link_lost_while_active
    assert esp.stop().ok and esp.get_status()["watchdog_ok"] is True


def test_offline_esp32_gives_readable_errors():
    import socket
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    e = ESP32Client("127.0.0.1", port, ack_timeout_s=0.3, connect_timeout_s=0.3)
    assert e.connect(timeout=1.0) is False
    assert "cannot connect" in e.last_error()
    r = e.set_led("on")
    assert not r.ok and "not connected" in r.error
    assert e.get_status() is None
    e.disconnect()


@pytest.mark.parametrize("which", ["cpp", "python"])
def test_console_run_all(which, cpp_sim_binary_or_none, tmp_path):
    if which == "cpp":
        if cpp_sim_binary_or_none is None:
            pytest.skip("C++ simulator not available")
        sim = _CppSim(cpp_sim_binary_or_none).start()
    else:
        sim = _PySim().start()
    try:
        r = subprocess.run([sys.executable, "-m", "robot.test_console", "--host", "127.0.0.1", "--port",
                            str(sim.port), "--run-all"], cwd=ROOT, capture_output=True, text=True, timeout=120)
    finally:
        sim.stop()
    assert r.returncode == 0, r.stdout[-3000:]
    assert "RESULT: ALL AS EXPECTED" in r.stdout
    assert "SIMULATED_MOVEMENT_RECEIVED: LEFT" in r.stdout and "UNKNOWN_COMMAND" in r.stdout
    log = [l for l in r.stdout.splitlines() if "Log saved to" in l][0].split("Log saved to ")[1].strip()
    assert Path(log).exists()
    Path(log).unlink()


def test_console_offline_exit_code():
    r = subprocess.run([sys.executable, "-m", "robot.test_console", "--host", "127.0.0.1", "--port", "1",
                        "--run-all", "--timeout", "0.3"], cwd=ROOT, capture_output=True, text=True, timeout=60)
    assert r.returncode == 2 and "Could not connect" in r.stdout
    log = [l for l in r.stdout.splitlines() if "Log saved to" in l][0].split("Log saved to ")[1].strip()
    Path(log).unlink()


# ------------------------------------------------------------- AI events (stage-1 update)
def test_ai_events_drive_led_and_buzzer_state(esp):
    """MODEL_STARTED -> LED on; FALL_DETECTED -> blink + buzzer; FALL_CLEARED -> LED on; MODEL_STOPPED -> off."""
    st = esp.get_status()
    assert st["led"] == "off" and st["model_running"] is False          # safe start
    r = esp.model_started()
    assert r.ok and r.result == "MODEL_RUNNING"
    st = esp.get_status()
    assert st["led"] == "on" and st["model_running"] is True and st["buzzer"] == "off"
    time.sleep(1.2)
    assert esp.get_status()["led"] == "on"                              # stays continuously on
    r = esp.fall_detected()
    assert r.ok and r.result == "FALL_ALERT_ACTIVE"
    st = esp.get_status()
    assert st["led"] == "blink_fall" and st["buzzer"] == "fall_alert" and st["fall_alert"] is True
    assert esp.fall_detected().ok                                       # repeat: no second alert
    assert esp.get_status()["fall_alert"] is True
    mv = esp.send_simulated_movement("LEFT")
    assert not mv.ok and "FALL_ALERT_ACTIVE" in mv.error               # movement blocked during the alert
    r = esp.fall_cleared()
    assert r.ok and r.result == "FALL_ALERT_CLEARED"
    st = esp.get_status()
    assert st["led"] == "on" and st["buzzer"] == "off" and st["fall_alert"] is False
    assert esp.fall_cleared().result == "NO_FALL_ALERT"
    r = esp.model_stopped()
    assert r.ok
    st = esp.get_status()
    assert st["led"] == "off" and st["buzzer"] == "off" and st["model_running"] is False
    assert esp.client.commanded["model_running"] is False and esp.client.commanded["fall_alert"] is False


def test_led_buzzer_test_works_without_model_and_led_off_returns_to_model_light(esp):
    assert esp.set_led("on").ok and esp.get_status()["led"] == "on"
    assert esp.set_led("off").ok and esp.get_status()["led"] == "off"
    assert esp.model_started().ok and esp.set_led("blink").ok
    assert esp.get_status()["led"] == "blink"
    assert esp.set_led("off").ok and esp.get_status()["led"] == "on"   # back to "model running"
    assert esp.set_buzzer("short_beep").ok


def test_emergency_has_priority_over_fall_alert(esp):
    assert esp.model_started().ok and esp.fall_detected().ok
    assert esp.emergency_stop().ok
    st = esp.get_status()
    assert st["emergency_active"] and st["led"] == "on" and st["buzzer"] == "emergency"
    assert esp.reset_emergency(confirmed=True).ok
    st = esp.get_status()
    assert st["fall_alert"] is True and st["led"] == "blink_fall"     # the fall alert needs its own clear
    assert esp.fall_cleared().ok and esp.get_status()["led"] == "on"


def test_disconnect_turns_model_light_off_and_resumes_nothing(esp):
    assert esp.model_started().ok and esp.send_simulated_movement("RIGHT").ok
    esp.client._disconnect("test: Wi-Fi drop")
    end = time.monotonic() + 15
    while time.monotonic() < end and not (esp.connected and esp.responding() and esp.hello()):
        time.sleep(0.05)
    st = esp.get_status()
    assert st["model_running"] is False and st["led"] == "off" and st["simulated_movement"] == "none"


def test_robot_link_sends_ai_events_on_state_changes_only(pysim):
    from types import SimpleNamespace
    from robot.robot_logic import RobotLink
    from robot.wifi_client import RobotClient
    events = []
    link = RobotLink(RobotClient("127.0.0.1", pysim.port, reconnect_delay_s=0.1),
                     event_logger=SimpleNamespace(log=lambda et, **kw: events.append((et, kw)))).start()
    try:
        assert link.client.wait_connected(3) and link.client.wait_status(3)
        n0 = sum('"MODEL_STARTED"' in l for l in pysim.received)
        for _ in range(30):                                   # 30 "frames": only ONE event is sent
            link.set_model_running(True)
        _wait_for(lambda: pysim.core.model_running)
        time.sleep(0.2)
        assert sum('"MODEL_STARTED"' in l for l in pysim.received) - n0 == 1
        link.on_alert(SimpleNamespace(severity="emergency", message="fall", alert_id="a1", track_id=1))
        _wait_for(lambda: pysim.core.fall_alert)
        assert pysim.core.state == "IDLE"                     # a fall is an alert, not an E-STOP latch
        assert not link.safety().allow_manual                 # movement blocked until acknowledged
        for _ in range(10):
            link.set_fall_alert(True)
        time.sleep(0.2)
        assert sum('"FALL_DETECTED"' in l for l in pysim.received) == 1
        # reconnect: the current AI state is shown again; nothing else resumes
        link.client._disconnect("test drop")
        _wait_for(lambda: link.client.connected and pysim.core.model_running, 10)
        assert pysim.core.fall_alert and pysim.core.sim_move == "none"
        link.set_fall_alert(False, "acknowledged")
        _wait_for(lambda: not pysim.core.fall_alert)
        link.set_model_running(False)
        _wait_for(lambda: not pysim.core.model_running)
        assert any(et == "ROBOT_AI_EVENT" for et, _ in events)
    finally:
        link.close()


def _wait_for(cond, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return
        time.sleep(0.02)
    raise AssertionError("condition not reached")


@pytest.fixture
def pysim():
    srv = SimRobotServer(port=0).start()
    yield srv
    srv.stop()
