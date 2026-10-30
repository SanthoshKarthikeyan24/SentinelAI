"""robot.host: auto - the laptop finds the ESP32 on the local network by itself."""
import ipaddress
import socket
import threading
import time

import pytest

from robot import discovery
from robot.sim_robot import SimRobotServer
from robot.wifi_client import RobotClient


@pytest.fixture()
def cache(tmp_path, monkeypatch):
    f = tmp_path / "robot_last_host.txt"
    monkeypatch.setattr(discovery, "CACHE_FILE", f)
    return f


@pytest.fixture()
def sim():
    s = SimRobotServer(port=0).start()
    yield s
    s.stop()


class _NotARobot:
    """A TCP server on the same port that speaks something else (another device on the LAN)."""

    def __init__(self, greeting: bytes):
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", 0))
        self.srv.listen(4)
        self.port = self.srv.getsockname()[1]
        self.greeting = greeting
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            try:
                c, _ = self.srv.accept()
            except OSError:
                return
            try:
                c.sendall(self.greeting)
                time.sleep(0.05)
            finally:
                c.close()

    def close(self):
        self.srv.close()


def test_probe_accepts_only_the_sentinel_hello(sim):
    hello = discovery.probe("127.0.0.1", sim.port)
    assert hello and hello["event"] == "hello" and hello["protocol"] == 3
    other = _NotARobot(b'{"hello":"world"}\n')
    old = _NotARobot(b'{"version":2,"type":"event","event":"hello","protocol":2}\n')
    try:
        assert discovery.probe("127.0.0.1", other.port) is None
        assert discovery.probe("127.0.0.1", old.port) is None        # wrong protocol = not our robot
    finally:
        other.close()
        old.close()
    free = socket.socket()
    free.bind(("127.0.0.1", 0))
    port = free.getsockname()[1]
    free.close()
    assert discovery.probe("127.0.0.1", port) is None                # nothing listening


def test_probe_leaves_the_robot_idle(sim):
    for _ in range(3):
        assert discovery.probe("127.0.0.1", sim.port)
    time.sleep(0.2)
    assert sim.core.state == "IDLE" and not sim.core.model_running


def test_scan_finds_the_robot_and_remembers_it(sim, cache, monkeypatch):
    monkeypatch.setattr(discovery, "local_networks", lambda: [ipaddress.IPv4Network("127.0.0.0/29")])
    assert discovery.find_robot(sim.port, use_mdns=False) == "127.0.0.1"
    assert cache.read_text().strip() == "127.0.0.1"
    # next time the remembered address is tried first - no scan needed
    monkeypatch.setattr(discovery, "local_networks", lambda: [])
    assert discovery.find_robot(sim.port, use_mdns=False) == "127.0.0.1"


def test_not_found_returns_none(cache, monkeypatch):
    monkeypatch.setattr(discovery, "local_networks", lambda: [ipaddress.IPv4Network("127.0.0.0/30")])
    free = socket.socket()
    free.bind(("127.0.0.1", 0))
    port = free.getsockname()[1]
    free.close()
    t0 = time.monotonic()
    assert discovery.find_robot(port, use_mdns=False) is None
    assert time.monotonic() - t0 < 5


def test_local_networks_are_private_only():
    for net in discovery.local_networks():
        assert net.is_private and net.prefixlen == 24 and not net.is_loopback


def test_is_auto():
    assert discovery.is_auto("auto") and discovery.is_auto("AUTO ") and discovery.is_auto("")
    assert not discovery.is_auto("10.61.254.98")


def test_client_with_auto_host_connects_and_searches_again_after_losing_the_robot(cache, monkeypatch):
    s1 = SimRobotServer(port=0).start()
    port = s1.port
    calls = []

    def fake_find(p, stop=None, **kw):
        calls.append(p)
        return "127.0.0.1" if discovery.probe("127.0.0.1", p) else None

    monkeypatch.setattr(discovery, "find_robot", fake_find)
    c = RobotClient("auto", port, reconnect_delay_s=0.1, reconnect_max_delay_s=0.2).start()
    try:
        assert c.auto and c.wait_connected(5) and c.host == "127.0.0.1"
        assert c.status()["auto_host"] is True
        assert cache.read_text().strip() == "127.0.0.1"
        n = len(calls)
        s1.stop()                                   # robot gone (e.g. hotspot gave it a new address)
        deadline = time.time() + 5
        while time.time() < deadline and len(calls) == n:
            time.sleep(0.05)
        assert len(calls) > n                       # it searched again instead of insisting on the old address
        assert not c.connected and "ESP32 not found" in c.last_error
        s2 = SimRobotServer(port=port).start()      # robot back
        try:
            assert c.wait_connected(5)
            assert c.commanded["simulated_movement"] == "none" and c.commanded["left"] == 0
        finally:
            s2.stop()
    finally:
        c.close(send_stop=False)


def test_fixed_host_is_never_searched(monkeypatch, sim):
    monkeypatch.setattr(discovery, "find_robot", lambda *a, **k: pytest.fail("must not search"))
    c = RobotClient("127.0.0.1", sim.port).start()
    try:
        assert not c.auto and c.wait_connected(5)
    finally:
        c.close()
