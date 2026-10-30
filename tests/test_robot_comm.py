"""Laptop <-> robot communication (protocol v3): RobotClient, CommandManager, RobotLink.

Most tests run against two simulated robots:
  * "cpp"    - the REAL firmware core (firmware/sentinel_robot/robot_core.cpp) compiled for
               the PC with a TCP server around it (skipped if g++ is not installed);
  * "python" - robot/sim_robot.py (what you can run on Windows).
"""
import shutil
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from robot.command_manager import CommandManager
from robot.commands import CommandError, make_command
from robot.comm_test import run_comm_test
from robot.robot_logic import RobotLink
from robot.sim_robot import SimRobotServer
from robot.wifi_client import RobotClient, RobotNotConnected

ROOT = Path(__file__).resolve().parent.parent
FW = ROOT / "firmware"


def _free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def wait_for(cond, timeout=2.0, what="condition"):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return
        time.sleep(0.01)
    raise AssertionError(f"{what} not reached within {timeout} s")


@pytest.fixture(scope="session")
def cpp_sim_binary_or_none(tmp_path_factory):
    gxx = shutil.which("g++")
    if gxx is None or sys.platform.startswith("win"):
        return None
    out = tmp_path_factory.mktemp("sim") / "robot_sim"
    subprocess.run([gxx, "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror", f"-I{FW / 'sentinel_robot'}",
                    str(FW / "host_test" / "robot_sim.cpp"), str(FW / "sentinel_robot" / "robot_core.cpp"),
                    "-o", str(out), "-lpthread"], check=True, capture_output=True, text=True)
    return out


class _CppSim:
    kind = "cpp"

    def __init__(self, binary, motors=False):
        self.binary, self.motors = binary, motors
        self.port = _free_port()
        self.proc = None

    def start(self):
        self.proc = subprocess.Popen([str(self.binary), str(self.port), "1" if self.motors else "0", "0"],
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        deadline = time.time() + 5
        while time.time() < deadline:
            try:
                socket.create_connection(("127.0.0.1", self.port), timeout=0.2).close()
                time.sleep(0.05)   # let the simulator forget that probe connection
                return self
            except OSError:
                time.sleep(0.05)
        raise RuntimeError("C++ simulator did not start")

    def stop(self):
        if self.proc:
            self.proc.terminate()
            self.proc.wait(timeout=5)


class _PySim:
    kind = "python"

    def __init__(self, motors=False, port=0):
        self.srv = SimRobotServer(port=port, motors_enabled=motors)
        self.port = None

    def start(self):
        self.srv.start()
        self.port = self.srv.port
        return self

    def stop(self):
        self.srv.stop()


@pytest.fixture(params=["cpp", "python"])
def robot(request, cpp_sim_binary_or_none):
    if request.param == "cpp":
        if cpp_sim_binary_or_none is None:
            pytest.skip("C++ simulator not available")
        sim = _CppSim(cpp_sim_binary_or_none).start()
    else:
        sim = _PySim().start()
    yield sim
    sim.stop()


@pytest.fixture
def pysim():
    sim = _PySim().start()
    yield sim.srv
    sim.stop()


def connect(port, **kw):
    c = RobotClient("127.0.0.1", port, **kw).start()
    assert c.wait_connected(3) and c.wait_status(3)
    return c


# ----------------------------------------------------------- full test script
def test_comm_test_script_passes(robot, tmp_path):
    data = run_comm_test("127.0.0.1", robot.port, pings=20, client_seconds=1.0, out_dir=tmp_path)
    failed = [s for s in data["steps"] if not s["passed"]]
    assert data["all_passed"], failed
    assert len(data["steps"]) >= 13
    assert list(tmp_path.glob("comm_test_*.json"))


# ------------------------------------------------------------------ client
def test_client_heartbeat_keeps_the_watchdog_quiet(robot):
    c = connect(robot.port)
    try:
        r = c.send("motor", mode="manual", left=20, right=20, duration_ms=2000)
        assert r.ok and c.commanded["left"] == 20
        time.sleep(1.2)                                 # > 2 watchdog periods; pings keep it alive
        assert c.stats["watchdog_stops"] == 0 and c.stats["timeouts"] == 0
        assert c.status()["reported"]["watchdog_trips"] == 0
        assert c.send("stop").ok and c.commanded["left"] == 0
    finally:
        c.close()


def test_frozen_laptop_trips_watchdog_and_is_reported(robot):
    c = connect(robot.port)
    try:
        assert c.send("motor", mode="manual", left=20, right=20, duration_ms=2000).ok
        c.heartbeat_paused = True
        wait_for(lambda: c.stats["watchdog_stops"] == 1, 1.5, "watchdog_stop event")
        assert c.comm_fault_reported and c.commanded["left"] == 0
        c.heartbeat_paused = False
        wait_for(lambda: c.robot.get("watchdog_ok") is False, 2.0, "status watchdog_ok false")
        assert c.send("stop").ok
        c.send("status_request")
        wait_for(lambda: c.robot.get("watchdog_ok") is True, 1.0, "watchdog_ok restored by stop")
    finally:
        c.close()


def test_reconnect_never_resumes_motion_and_reports_comm_fault(robot):
    c = connect(robot.port, reconnect_delay_s=0.1)
    try:
        assert c.send("follow_mode", enabled=True, target_id=3).ok
        assert c.commanded["follow_mode"]
        # drop the TCP connection from the laptop side
        c._disconnect("test: simulated Wi-Fi drop")
        wait_for(lambda: c.connected and c.status_fresh, 3.0, "reconnection")
        assert c.link_lost_while_active
        assert c.commanded["follow_mode"] is False and c.commanded["left"] == 0
        c.send("status_request")
        time.sleep(0.1)
        assert c.robot["follow_mode"] is False and c.robot["motors_stopped"] is True
        c.acknowledge_comm_fault()
        assert not c.link_lost_while_active
    finally:
        c.close()


def test_status_and_commanded_are_kept_apart(robot):
    c = connect(robot.port)
    try:
        assert c.send("led", state="blink", interval_ms=300).ok
        assert c.commanded["led"] == "blink"
        c.send("status_request")
        wait_for(lambda: c.robot.get("led") == "blink", 1.0)
        assert c.send("follow_mode", enabled=True, target_id=4).ok
        assert c.send("led", state="off").ok
        c.send("status_request")
        wait_for(lambda: c.robot.get("led") == "blink_following", 1.0)
        assert c.commanded["led"] == "off"            # what we asked for differs from what the LED shows
        st = c.status()
        assert st["commanded"]["led"] == "off" and st["reported"]["led"] == "blink_following"
    finally:
        c.close()


def test_emergency_is_retried_until_acknowledged_and_relatched_after_robot_restart():
    port = _free_port()
    sim = _PySim(port=port).start()
    c = RobotClient("127.0.0.1", port, reconnect_delay_s=0.1, emergency_retry_s=0.05).start()
    try:
        c.trigger_emergency("fall_confirmed")         # before the link is up: queued
        assert c.emergency_pending
        assert c.wait_connected(3)
        wait_for(lambda: not c.emergency_pending, 2.0, "emergency ack")
        wait_for(lambda: sim.srv.core.state == "EMERGENCY", 1.0)
        sim.stop()                                    # robot reboots (power glitch): latch lost
        sim = _PySim(port=port).start()
        wait_for(lambda: sim.srv.core.state == "EMERGENCY", 5.0, "emergency re-latched after restart")
        assert c.emergency_latched
        assert c.clear_emergency().ok and not c.emergency_latched
    finally:
        c.close()
        sim.stop()


def test_replaced_connection_stops_reconnecting(pysim):
    a = connect(pysim.port, reconnect_delay_s=0.1)
    b = connect(pysim.port)
    try:
        wait_for(lambda: a.replaced and not a.connected, 2.0)
        time.sleep(0.4)
        assert not a.connected and b.connected        # no reconnect ping-pong
        a.set_enabled(True)                           # operator reconnects deliberately
        wait_for(lambda: a.connected, 3.0)
        wait_for(lambda: b.replaced, 2.0)
    finally:
        a.close()
        b.close()


def test_operator_disconnect_sends_stop_first(pysim):
    c = connect(pysim.port)
    try:
        assert c.send("motor", mode="manual", left=20, right=20, duration_ms=2000).ok
        c.set_enabled(False)
        assert not c.connected and not c.link_lost_while_active   # stopped cleanly: not a fault
        assert any('"type":"stop"' in line for line in pysim.received[-5:])
        assert pysim.core.state == "IDLE" and not pysim.core.comm_fault
        time.sleep(0.3)
        assert not c.connected                       # stays disconnected
    finally:
        c.close()


def test_close_sends_stop(pysim):
    c = connect(pysim.port)
    c.send("motor", mode="manual", left=20, right=20, duration_ms=2000)
    c.close()
    assert pysim.core.state == "IDLE" and '"type":"stop"' in pysim.received[-1]


def test_client_survives_garbage_and_overlong_lines():
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    port = srv.getsockname()[1]

    def serve():
        conn, _ = srv.accept()
        conn.sendall(b'{"version":3,"type":"event","event":"hello","protocol":3,"state":"idle",'
                     b'"motors_enabled":false,"sensor_installed":false,"comm_fault":false,"watchdog_trips":0}\n')
        conn.sendall(b"garbage\n[1,2]\n")
        time.sleep(0.2)
        conn.sendall(b"x" * 5000)                     # no newline: broken robot
        time.sleep(1.0)
        conn.close()

    t = threading.Thread(target=serve, daemon=True)
    t.start()
    c = RobotClient("127.0.0.1", port, reconnect_delay_s=5).start()
    try:
        assert c.wait_connected(2)
        wait_for(lambda: not c.connected, 2.0, "link reset after over-long data")
        assert "over-long" in c.last_error or "acks missing" in c.last_error
    finally:
        c.close()
        srv.close()


def test_protocol_mismatch_disconnects():
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    port = srv.getsockname()[1]
    threading.Thread(target=lambda: srv.accept()[0].sendall(
        b'{"version":2,"type":"event","event":"hello","protocol":2}\n') or time.sleep(1), daemon=True).start()
    c = RobotClient("127.0.0.1", port, reconnect_delay_s=5).start()
    try:
        wait_for(lambda: "protocol" in c.last_error, 2.0)
        assert not c.connected
    finally:
        c.close()
        srv.close()


def test_stale_motion_is_refused_locally(pysim):
    c = connect(pysim.port, max_command_age_s=0.2)
    try:
        with pytest.raises(CommandError):
            c.send_nowait("motor", created_at=time.monotonic() - 1.0, mode="manual", left=10, right=10,
                          duration_ms=300)
        assert c.stats["refused_local"] == 1
    finally:
        c.close()


def test_not_connected_raises():
    c = RobotClient("127.0.0.1", _free_port())
    with pytest.raises(RobotNotConnected):
        c.send("ping")


# --------------------------------------------------------- command manager
class _FakeClient:
    """Records what the manager hands to the link (no network)."""

    def __init__(self):
        self.sent, self.emergencies = [], []
        self.gate = threading.Event()

    def send_command(self, cmd, created_at=None, background=False):
        self.gate.wait(2)
        self.sent.append(cmd)
        return SimpleNamespace(wait=lambda t: None)

    def trigger_emergency(self, reason):
        self.emergencies.append(reason)


def test_manager_emergency_first_and_drops_queued_motion():
    fc = _FakeClient()
    m = CommandManager(fc, max_age_s=5).start()
    try:
        m.submit("led", state="on")                   # this one blocks inside the fake link
        time.sleep(0.05)
        t1 = m.submit("motor", mode="manual", left=10, right=10, duration_ms=300)
        m.submit("buzzer", pattern="short_beep")
        te = m.submit("emergency_stop", reason="fall_confirmed")
        assert t1.outcome == "dropped" and "emergency_stop" in t1.detail
        fc.gate.set()
        wait_for(lambda: m.status()["queued"] == 0)
        time.sleep(0.05)
        assert fc.emergencies == ["fall_confirmed"] and te.outcome == "sent"
        assert [c.type for c in fc.sent] == ["led", "buzzer"]          # motion never sent
    finally:
        m.close()


def test_manager_coalesces_motion_and_orders_by_priority():
    fc = _FakeClient()
    m = CommandManager(fc, max_age_s=5).start()
    try:
        m.submit("ping")
        time.sleep(0.05)                              # ping is now blocking in the fake link
        a = m.submit("follow", target_id=1, left=10, right=10, duration_ms=300, confidence=0.9)
        b = m.submit("follow", target_id=1, left=20, right=20, duration_ms=300, confidence=0.9)
        m.submit("led", state="blink")
        m.submit("obstacle", source="camera", distance_cm=0)
        m.submit("stop")
        assert a.outcome == "dropped" and b.outcome == "dropped"       # stop removes queued motion
        m.submit("motor", mode="manual", left=5, right=5, duration_ms=300)
        m.submit("motor", mode="manual", left=6, right=6, duration_ms=300)
        fc.gate.set()
        wait_for(lambda: m.status()["queued"] == 0)
        time.sleep(0.05)
        assert [str(c) for c in fc.sent] == ["ping()", "stop()", "obstacle(source=camera, distance_cm=0)",
                                             "led(state=blink)",
                                             "motor(mode=manual, left=6, right=6, duration_ms=300)"]
        assert m.stats["coalesced"] == 2 and m.stats["dropped_stop"] == 1
    finally:
        m.close()


def test_manager_drops_stale_motion_and_applies_gate():
    fc = _FakeClient()
    fc.gate.set()
    m = CommandManager(fc, max_age_s=0.2, gate=lambda cmd: "blocked" if cmd.get("left") == 99 else "").start()
    try:
        old = m.submit("motor", created_at=time.monotonic() - 1, mode="manual", left=10, right=10, duration_ms=300)
        wait_for(lambda: old.outcome != "queued")
        assert old.outcome == "dropped" and "stale" in old.detail
        g = m.submit("motor", mode="manual", left=99, right=0, duration_ms=300)
        wait_for(lambda: g.outcome != "queued")
        assert g.outcome == "dropped" and g.detail == "blocked"
        assert fc.sent == []
        with pytest.raises(CommandError):
            m.submit("motor", mode="manual", left=500, right=0, duration_ms=300)
    finally:
        m.close()


# --------------------------------------------------------------- robot link
@pytest.fixture
def link(pysim):
    events = []
    logger = SimpleNamespace(log=lambda et, **kw: events.append((et, kw)))
    c = RobotClient("127.0.0.1", pysim.port, reconnect_delay_s=0.1)
    lk = RobotLink(c, max_speed_pct=40, event_logger=logger).start()
    assert c.wait_connected(3) and c.wait_status(3)
    yield lk, pysim, events
    lk.close()


def test_link_manual_press_and_hold(link):
    lk, sim, _ = link
    d = lk.manual("forward")
    assert d.sent and d.ticket.wait(1).ok
    assert sim.core.state == "MOVING" and sim.core.move_dur == lk.manual_hold_ms
    d = lk.manual("left", speed_pct=90)               # capped to max_speed_pct
    assert d.ticket.wait(1).data["left"] == -40
    time.sleep(lk.manual_hold_ms / 1000 + 0.2)        # released: the command expires on the robot
    assert sim.core.state == "IDLE"
    assert not lk.manual("sideways").sent


def test_link_emergency_blocks_and_logs_commanded_vs_confirmed(link):
    lk, sim, events = link
    lk.emergency("operator")
    wait_for(lambda: sim.core.state == "EMERGENCY", 1.0)
    wait_for(lambda: not lk.client.emergency_pending, 1.0)
    d = lk.manual("backward")
    assert not d.sent and "emergency" in d.reason
    lk.client.send("status_request")
    wait_for(lambda: lk.client.robot_emergency, 1.0)
    assert lk.safety().level.name == "FIRMWARE_EMERGENCY"
    with pytest.raises(CommandError):
        lk.clear_emergency(False)
    assert lk.clear_emergency(True).ok
    types = [e[0] for e in events]
    assert "ROBOT_EMERGENCY_SENT" in types and "ROBOT_EMERGENCY_CLEARED" in types
    sent = next(kw for et, kw in events if et == "ROBOT_EMERGENCY_SENT")
    assert "robot" in sent["extra"] and "commanded" in sent["extra"]["note"]


def test_link_alerts_drive_the_robot_signals(link):
    lk, sim, _ = link
    lk.on_alert(SimpleNamespace(severity="warning", message="suspected", alert_id="w1", track_id=1))
    wait_for(lambda: sim.core.led_mode == 2 and sim.core.led_interval == 200, 1.0)
    assert sim.core.state == "IDLE"                   # a warning never latches an emergency
    lk.clear_warning("recovered")
    wait_for(lambda: sim.core.led_mode == 0, 1.0)
    lk.on_alert(SimpleNamespace(severity="emergency", message="fall", alert_id="a1", track_id=1))
    wait_for(lambda: sim.core.fall_alert and sim.core.state == "IDLE", 1.0)     # FALL_DETECTED, not a latch
    assert not lk.manual("forward").sent                        # movement refused while the alert is active
    lk.set_fall_alert(False, "operator acknowledged")
    wait_for(lambda: not sim.core.fall_alert, 1.0)


def test_link_comm_fault_needs_operator_stop(link):
    lk, sim, events = link
    assert lk.manual("forward").ticket.wait(1).ok
    lk.client._disconnect("test: Wi-Fi drop while moving")
    wait_for(lambda: lk.client.connected and lk.client.status_fresh, 3.0)
    d = lk.safety()
    assert d.level.name == "COMM_FAULT" and not d.allow_manual
    assert not lk.manual("forward").sent              # nothing resumes after the reconnection
    ok, msg = lk.acknowledge_comm_fault()             # operator: STOP = fresh control state
    assert ok, msg
    assert lk.safety().level.name in ("IDLE", "MANUAL")
    assert lk.manual("forward").sent


def test_link_logs_refusals_and_robot_events(link):
    lk, sim, events = link
    lk.report_obstacle(30)
    wait_for(lambda: sim.core.obstacle_active, 1.0)
    r = lk.manager.submit("motor", mode="manual", left=20, right=20, duration_ms=300).wait(1)
    assert r is not None and r.error == "OBSTACLE_AHEAD"
    wait_for(lambda: any(e[0] == "ROBOT_REFUSED" for e in events), 1.0)
    wait_for(lambda: any(e[0] == "ROBOT_OBSTACLE_CLEAR" for e in events), 2.5)   # 1.5 s hold expired
    ref = next(kw for et, kw in events if et == "ROBOT_REFUSED")
    assert ref["extra"]["robot_ack"] == "OBSTACLE_AHEAD"
