"""Automated laptop <-> robot communication test, protocol v3 (docs/TESTING.md).

Runs, in order, against a real ESP32 or a simulator:
  1  connect and read the robot's hello (protocol 3)
  2  harmless messages: N x ping, acknowledgement latency
  3  stop is acknowledged
  4  invalid messages are rejected with the right error codes (link stays usable)
  5  periodic status arrives every second with all required fields
  6  a short motor command is accepted and ends by itself (move_done)*
  7  watchdog: laptop goes silent while moving -> robot stops, reports watchdog_ok:false
     until a fresh stop*
  8  disconnection while moving -> robot stops, next hello reports comm_fault, nothing resumes*
  9  emergency_stop latches (motors 0, LED on, buzzer), blocks motion and follow mode,
     mute keeps the latch, clear_emergency needs operator_confirmed:true
 10  emergency_stop sent behind a burst of 20 messages is still handled
 11  LED on / blink (interval) / off
 12  buzzer short_beep / warning / emergency / off and mute
 13  follow mode: target matching, low confidence refused, manual refused while following, stop
 14  camera obstacle: forward refused, reverse allowed, obstacle_clear, 1.5 s automatic expiry
 15  named commands (stage-1 test): simulated movement, LED, buzzer, unknown command, emergency
 16  simulated movement + silent laptop -> watchdog clears it
 17  connection lost during a simulated movement -> cleared, reported, never resumed
 18  RobotLink end to end: heartbeat, safety supervisor, emergency, clear, manual control
 (*) with MOTORS_ENABLED 0 in the firmware these only change the robot's internal state; no
     wheel moves. With motors enabled the script refuses to run them unless --allow-motion is
     given (wheels must be lifted off the ground!).

Optional manual step (--manual-network-test): you switch the laptop's Wi-Fi off for ~5 s
while the robot is "moving"; afterwards the script checks that the watchdog stopped it.

Usage:
    python -m robot.comm_test --host 192.168.43.50
    python -m robot.comm_test --host 127.0.0.1 --port 3333     # against python -m robot.sim_robot
Results: evaluation/communication/comm_test_<time>.json
"""
from __future__ import annotations

import argparse
import json
import socket
import statistics
import sys
import time
from datetime import datetime
from pathlib import Path

from .commands import encode, make_command
from .robot_logic import RobotLink
from .wifi_client import RobotClient

ROOT = Path(__file__).resolve().parent.parent
STATUS_FIELDS = ("state", "motors_stopped", "left", "right", "motors_enabled", "follow_mode", "target_id",
                 "emergency_active", "emergency_reason", "obstacle_detected", "obstacle_source",
                 "sensor_installed", "distance_cm", "watchdog_ok", "watchdog_trips", "led", "buzzer",
                 "last_command_id", "uptime_s")


class RawLink:
    """Minimal synchronous JSON-lines connection used for exact protocol checks."""

    def __init__(self, host: str, port: int, timeout: float = 3.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.buf = b""
        self.events: list[dict] = []
        self.statuses: list[tuple[float, dict]] = []
        self.seq = 0

    def readline(self, timeout: float) -> dict | None:
        end = time.monotonic() + timeout
        while b"\n" not in self.buf:
            left = end - time.monotonic()
            if left <= 0:
                return None
            self.sock.settimeout(left)
            try:
                data = self.sock.recv(1024)
            except socket.timeout:
                return None
            if not data:
                raise ConnectionError("robot closed the connection")
            self.buf += data
        line, self.buf = self.buf.split(b"\n", 1)
        try:
            msg = json.loads(line.decode("ascii", "replace"))
        except json.JSONDecodeError:
            return {"type": "invalid", "raw": line.decode("ascii", "replace")}
        return msg if isinstance(msg, dict) else {"type": "invalid", "raw": str(msg)}

    def _sort(self, msg: dict) -> bool:
        """Store events/status; True if msg was one of them."""
        if msg.get("type") == "event":
            self.events.append(msg)
            return True
        if msg.get("type") == "status":
            self.statuses.append((time.monotonic(), msg))
            return True
        return False

    def send_line(self, text: str) -> None:
        self.sock.sendall(text.encode("ascii") + b"\n")

    def request(self, msg_type: str, seq: int | None = None, timeout: float = 1.0, **fields
                ) -> tuple[dict | None, float]:
        """Send a valid command and wait for its ack. Returns (ack, latency_ms)."""
        if seq is None:
            self.seq += 1
            seq = self.seq
        t0 = time.perf_counter()
        self.sock.sendall(encode(seq, f"t{seq}", make_command(msg_type, **fields)))
        return self.wait_ack(timeout, t0)

    def request_raw(self, text: str, timeout: float = 1.0) -> dict | None:
        self.send_line(text)
        return self.wait_ack(timeout)[0]

    def wait_ack(self, timeout: float, t0: float | None = None) -> tuple[dict | None, float]:
        t0 = t0 or time.perf_counter()
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            msg = self.readline(end - time.monotonic())
            if msg is None:
                break
            if self._sort(msg):
                continue
            return msg, (time.perf_counter() - t0) * 1000.0
        return None, (time.perf_counter() - t0) * 1000.0

    def wait_event(self, name: str, timeout: float) -> tuple[dict | None, float]:
        t0 = time.perf_counter()
        for i, e in enumerate(self.events):
            if e.get("event") == name:
                return self.events.pop(i), 0.0
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            msg = self.readline(end - time.monotonic())
            if msg is None:
                break
            if msg.get("type") == "event" and msg.get("event") == name:
                return msg, (time.perf_counter() - t0) * 1000.0
            self._sort(msg)
        return None, (time.perf_counter() - t0) * 1000.0

    def status(self) -> dict:
        """Ask for a status now and return it ({} if none came)."""
        ack, _ = self.request("status_request")
        if not ack or not ack.get("ok"):
            return {}
        end = time.monotonic() + 1.0
        n = len(self.statuses)
        while time.monotonic() < end:
            if len(self.statuses) > n:
                return self.statuses[-1][1]
            msg = self.readline(end - time.monotonic())
            if msg is not None:
                self._sort(msg)
        return {}

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass


class Report:
    def __init__(self, host: str, port: int):
        self.data = {"host": host, "port": port, "protocol": 3,
                     "started": datetime.now().isoformat(timespec="seconds"), "steps": []}

    def step(self, name: str, passed: bool, **details) -> bool:
        self.data["steps"].append({"step": name, "passed": bool(passed), **details})
        mark = "PASS" if passed else "FAIL"
        extra = ", ".join(f"{k}={v}" for k, v in details.items() if k not in ("replies", "status"))
        print(f"  [{mark}] {name}" + (f"  ({extra})" if extra else ""), flush=True)
        return passed

    @property
    def passed(self) -> bool:
        return all(s["passed"] for s in self.data["steps"]) and bool(self.data["steps"])


def _stats(values: list[float]) -> dict:
    if not values:
        return {}
    v = sorted(values)
    return {"n": len(v), "mean_ms": round(statistics.mean(v), 2), "median_ms": round(statistics.median(v), 2),
            "p95_ms": round(v[int(0.95 * (len(v) - 1))], 2), "max_ms": round(v[-1], 2)}


def _ok(a: dict | None) -> bool:
    return bool(a) and a.get("ok") is True


def _err(a: dict | None, code: str) -> bool:
    return bool(a) and a.get("ok") is False and a.get("error") == code


def run_comm_test(host: str, port: int = 3333, pings: int = 50, allow_motion: bool = False,
                  watchdog_ms: int = 500, client_seconds: float = 3.0, manual_network_test: bool = False,
                  save: bool = True, out_dir: Path | None = None) -> dict:
    rep = Report(host, port)
    print(f"Communication test (protocol v3) against {host}:{port}")
    try:
        link = RawLink(host, port)
    except OSError as exc:
        rep.step("1 connect", False, error=str(exc))
        return _finish(rep, save, out_dir)
    try:
        hello, _ = link.wait_event("hello", 2.0)
        good = bool(hello) and hello.get("protocol") == 3
        rep.step("1 connect + hello (protocol 3)", good, hello=hello)
        if not good:
            return _finish(rep, save, out_dir)
        rep.data["robot"] = hello
        if hello.get("state") == "emergency":
            r, _ = link.request("clear_emergency", operator_confirmed=True)
            rep.step("0 clear an emergency left from earlier", _ok(r), reply=r)
        motion_ok = (not hello.get("motors_enabled")) or allow_motion
        rep.data["motors_enabled"] = hello.get("motors_enabled")

        # 2 harmless messages
        lat, ok = [], 0
        for _ in range(pings):
            r, ms = link.request("ping")
            if _ok(r) and r.get("seq") == link.seq:
                ok += 1
                lat.append(ms)
        rep.step("2 ping acknowledgements", ok == pings, sent=pings, acknowledged=ok,
                 success_rate=round(ok / pings, 4) if pings else None, latency=_stats(lat))

        # 3 stop
        r, ms = link.request("stop")
        rep.step("3 stop acknowledged", _ok(r), reply=r, latency_ms=round(ms, 2))

        # 4 invalid messages
        n = link.seq
        cases = [
            ("hello", "BAD_JSON"),
            ('{"version":3,"type":"ping","command_id":"a","seq":1', "BAD_JSON"),
            (f'{{"version":2,"type":"ping","command_id":"a","seq":{n + 1}}}', "BAD_VERSION"),
            (f'{{"version":3,"type":"ping","command_id":"bad id","seq":{n + 1}}}', "BAD_FIELD"),
            (f'{{"version":3,"type":"jump","command_id":"a","seq":{n + 1}}}', "UNKNOWN_TYPE"),
            (f'{{"version":3,"type":"motor","command_id":"a","seq":{n + 2},"mode":"manual","left":150,'
             f'"right":0,"duration_ms":300}}', "BAD_FIELD"),
            (f'{{"version":3,"type":"motor","command_id":"a","seq":{n + 3},"mode":"manual","left":10,'
             f'"right":0,"duration_ms":99999}}', "BAD_FIELD"),
            (f'{{"version":3,"type":"motor","command_id":"a","seq":{n + 4},"mode":"manual","left":10}}',
             "MISSING_FIELD"),
            (f'{{"version":3,"type":"ping","command_id":"a","seq":{n + 5},"x":1}}', "UNKNOWN_FIELD"),
            (f'{{"version":3,"type":"clear_emergency","command_id":"a","seq":{n + 6},"operator_confirmed":false}}',
             "NEEDS_CONFIRM"),
            ('{"version":3,"type":"ping","command_id":"a","seq":1}', "BAD_SEQ"),
            ('{"version":3,"type":"ping","command_id":"a","seq":' + str(n + 7) + ',"pad":"' + "x" * 200 + '"}',
             "TOO_LONG"),
        ]
        replies, good = [], True
        for text, expect in cases:
            r = link.request_raw(text)
            replies.append({"sent": text[:70], "reply": r, "expected": expect})
            good &= _err(r, expect)
        link.seq = n + 7
        r, _ = link.request("ping")
        good &= _ok(r)
        rep.step("4 invalid messages rejected (and link still usable)", good, cases=len(cases), replies=replies)

        # 5 periodic status
        link.statuses.clear()
        t_end = time.monotonic() + 2.6
        while time.monotonic() < t_end:
            m = link.readline(t_end - time.monotonic())
            if m is not None:
                link._sort(m)
        sts = [s for _, s in link.statuses]
        times = [t for t, _ in link.statuses]
        gaps = [b - a for a, b in zip(times, times[1:])]
        missing = sorted(set(STATUS_FIELDS) - set(sts[-1])) if sts else list(STATUS_FIELDS)
        rep.step("5 periodic status (every 1 s, all fields)",
                 len(sts) >= 2 and not missing and all(0.7 <= g <= 1.4 for g in gaps),
                 received=len(sts), gaps_s=[round(g, 2) for g in gaps], missing_fields=missing,
                 status=sts[-1] if sts else None)

        if motion_ok:
            # 6 short movement ends by itself
            r, _ = link.request("motor", mode="manual", left=30, right=30, duration_ms=300)
            evt, ms = link.wait_event("move_done", 1.5)
            rep.step("6 motor command accepted and ends by itself", _ok(r) and bool(evt),
                     reply=r, move_done_after_ms=round(ms, 1))
            # 7 watchdog
            r, _ = link.request("motor", mode="manual", left=30, right=30, duration_ms=2000)
            evt, ms = link.wait_event("watchdog_stop", 3.0)
            st = link.status()
            r2, _ = link.request("stop")
            st2 = link.status()
            rep.step("7 watchdog stops motion; watchdog_ok false until a fresh stop",
                     bool(evt) and st.get("motors_stopped") is True and st.get("watchdog_ok") is False
                     and _ok(r2) and st2.get("watchdog_ok") is True and watchdog_ms * 0.8 <= ms <= watchdog_ms + 600,
                     stopped_after_ms=round(ms, 1), expected_ms=watchdog_ms, status=st)
            # 8 disconnect during movement
            link.request("motor", mode="manual", left=30, right=30, duration_ms=2000)
            link.close()
            time.sleep(0.2)
            link = RawLink(host, port)
            hello2, _ = link.wait_event("hello", 2.0)
            st = link.status()
            rep.step("8 dropped connection stops motion; hello reports comm_fault; nothing resumes",
                     bool(hello2) and hello2.get("comm_fault") is True and st.get("motors_stopped") is True
                     and st.get("state") == "idle", hello_after_reconnect=hello2, status=st)
            link.request("stop")
        else:
            rep.step("6-8 motion tests skipped (motors enabled; use --allow-motion with wheels lifted)", True,
                     skipped=True)

        # 9 emergency
        r, ms = link.request("emergency_stop", reason="comm_test")
        em_ms = ms
        st = link.status()
        rm, _ = link.request("motor", mode="manual", left=20, right=20, duration_ms=300)
        rf, _ = link.request("follow_mode", enabled=True, target_id=1)
        rmute, _ = link.request("mute")
        st_mute = link.status()
        rr1 = link.request_raw(f'{{"version":3,"type":"clear_emergency","command_id":"c","seq":{link.seq + 1},'
                               f'"operator_confirmed":false}}')
        link.seq += 1
        rr2, _ = link.request("clear_emergency", operator_confirmed=True)
        st_after = link.status()
        ok9 = (_ok(r) and st.get("emergency_active") is True and st.get("emergency_reason") == "comm_test"
               and st.get("motors_stopped") is True and st.get("led") == "on" and st.get("buzzer") == "emergency"
               and _err(rm, "EMERGENCY_ACTIVE") and _err(rf, "EMERGENCY_ACTIVE") and _ok(rmute)
               and st_mute.get("emergency_active") is True and st_mute.get("buzzer") == "off"
               and _err(rr1, "NEEDS_CONFIRM") and _ok(rr2) and st_after.get("state") == "idle"
               and st_after.get("motors_stopped") is True and st_after.get("follow_mode") is False)
        rep.step("9 emergency latches, blocks motion/follow, mute keeps latch, clear needs confirmation", ok9,
                 emergency_ack_ms=round(em_ms, 2), replies=[r, rm, rf, rmute, rr1, rr2],
                 status_in_emergency=st, state_after_clear=st_after.get("state"),
                 note="red LED on + emergency buzzer until mute")

        # 10 emergency behind a burst
        first = link.seq + 1
        for i in range(20):
            link.sock.sendall(encode(first + i, f"b{first + i}", make_command("ping")))
        link.seq = first + 20
        t0 = time.perf_counter()
        link.sock.sendall(encode(link.seq, "estop", make_command("emergency_stop", reason="burst")))
        got, n = None, 0
        while n < 30:
            m, _ = link.wait_ack(2.0)
            if m is None:
                break
            n += 1
            if m.get("seq") == link.seq:
                got = m
                break
        ms = (time.perf_counter() - t0) * 1000.0
        r, _ = link.request("clear_emergency", operator_confirmed=True)
        rep.step("10 emergency_stop handled behind a burst of 20 messages", _ok(got) and _ok(r),
                 emergency_ack_ms=round(ms, 2), acks_before=n - 1 if got else n)

        # 11 LED
        replies, good = [], True
        for state, interval, expect in (("on", None, "on"), ("blink", 250, "blink"), ("off", None, "off")):
            r, _ = link.request("led", state=state, interval_ms=interval)
            st = link.status()
            replies.append(r)
            good &= _ok(r) and st.get("led") == expect
        rep.step("11 LED on / blink 250 ms / off", good, replies=replies, note="watch the red LED")

        # 12 buzzer
        replies, good = [], True
        for pattern, expect, wait in (("short_beep", "short_beep", 0.4), ("warning", "warning", 1.0),
                                      ("emergency", "emergency", 0.3), ("off", "off", 0.1)):
            r, _ = link.request("buzzer", pattern=pattern)
            st = link.status()
            replies.append(r)
            good &= _ok(r) and st.get("buzzer") == expect
            time.sleep(wait)
        link.request("buzzer", pattern="emergency")
        rm, _ = link.request("mute")
        good &= _ok(rm) and link.status().get("buzzer") == "off"
        rep.step("12 buzzer short_beep / warning / emergency / off, mute", good, replies=replies + [rm])

        # 13 follow mode
        r1, _ = link.request("follow_mode", enabled=True, target_id=5)
        st1 = link.status()
        rx, _ = link.request("follow", target_id=6, left=10, right=10, duration_ms=300, confidence=0.9)
        rlow, _ = link.request("follow", target_id=5, left=10, right=10, duration_ms=300, confidence=0.3)
        rman, _ = link.request("motor", mode="manual", left=10, right=10, duration_ms=300)
        rgo = None
        if motion_ok:
            rgo, _ = link.request("follow", target_id=5, left=20, right=15, duration_ms=300, confidence=0.9)
        rstop, _ = link.request("stop")
        st2 = link.status()
        rnf, _ = link.request("follow", target_id=5, left=10, right=10, duration_ms=300, confidence=0.9)
        ok13 = (_ok(r1) and st1.get("follow_mode") is True and st1.get("target_id") == 5
                and _err(rx, "TARGET_MISMATCH") and _err(rlow, "LOW_CONFIDENCE") and _err(rman, "FOLLOW_ACTIVE")
                and (rgo is None or _ok(rgo)) and _ok(rstop) and st2.get("follow_mode") is False
                and _err(rnf, "FOLLOW_NOT_ENABLED"))
        rep.step("13 follow mode: target match, low confidence, manual refused, stop ends it", ok13,
                 replies=[r1, rx, rlow, rman, rgo, rstop, rnf], note="one beep when follow mode starts")

        # 14 obstacle reports from the laptop camera
        st0 = link.status()
        if st0.get("obstacle_detected"):
            rep.step("14 obstacle skipped: the robot's own sensor already sees an obstacle", True, skipped=True,
                     distance_cm=st0.get("distance_cm"))
        else:
            r1, _ = link.request("obstacle", source="camera", distance_cm=40)
            st1 = link.status()
            rf = rb = None
            if motion_ok:
                rf, _ = link.request("motor", mode="manual", left=20, right=20, duration_ms=300)
                rb, _ = link.request("motor", mode="manual", left=-20, right=-20, duration_ms=300)
                link.request("stop")
            rc, _ = link.request("obstacle_clear")
            ev1, _ = link.wait_event("obstacle_clear", 1.0)
            r2, _ = link.request("obstacle", source="camera", distance_cm=0)
            t_obs = time.perf_counter()
            ev2, _ = link.wait_event("obstacle_clear", 3.0)
            hold_ms = (time.perf_counter() - t_obs) * 1000.0
            ok14 = (_ok(r1) and st1.get("obstacle_detected") is True and st1.get("obstacle_source") in ("camera", "both")
                    and (not motion_ok or (_err(rf, "OBSTACLE_AHEAD") and _ok(rb)))
                    and _ok(rc) and bool(ev1) and _ok(r2) and bool(ev2) and 1200 <= hold_ms <= 2300)
            rep.step("14 camera obstacle blocks forward (not reverse), obstacle_clear, 1.5 s expiry", ok14,
                     replies=[r1, rf, rb, rc, ev1, r2, ev2], hold_expired_after_ms=round(hold_ms, 1),
                     note="LED blinks fast while an obstacle is active")
        if hello.get("sensor_installed"):
            rep.data["sensor_distance_cm"] = link.status().get("distance_cm")

        # 15 named commands (stage-1 test commands): simulated movement, LED, buzzer, emergency
        replies, good = [], True
        r, _ = link.request("command", command="LEFT")
        replies.append(r)
        good &= _ok(r) and r.get("result") == "SIMULATED_MOVEMENT_RECEIVED: LEFT"
        st = link.status()
        good &= st.get("simulated_movement") == "left" and st.get("motors_stopped") is True
        r, _ = link.request("command", command="STOP")
        replies.append(r)
        good &= _ok(r) and link.status().get("simulated_movement") == "none"
        for name, key, val in (("LED_ON", "led", "on"), ("LED_BLINK", "led", "blink"), ("LED_OFF", "led", "off"),
                               ("BUZZER_ON", "buzzer", "on"), ("BUZZER_OFF", "buzzer", "off")):
            r, _ = link.request("command", command=name)
            replies.append(r)
            good &= _ok(r) and link.status().get(key) == val
        r = link.request_raw(f'{{"version":3,"type":"command","command_id":"u","seq":{link.seq + 1},'
                             f'"command":"JUMP"}}')
        link.seq += 1
        replies.append(r)
        good &= _err(r, "UNKNOWN_COMMAND")
        r, _ = link.request("command", command="EMERGENCY_STOP")
        r2, _ = link.request("command", command="RIGHT")
        r3, _ = link.request("clear_emergency", operator_confirmed=True)
        replies += [r, r2, r3]
        good &= _ok(r) and _err(r2, "EMERGENCY_ACTIVE") and _ok(r3)
        rep.step("15 named commands: simulated movement, LED, buzzer, unknown command, emergency", good,
                 replies=replies, note="LED and buzzer react if wired; no motor output is ever used")

        # 16 simulated movement + silence -> cleared by the watchdog
        r, _ = link.request("command", command="FORWARD")
        evt, ms = link.wait_event("watchdog_stop", 2.0)
        st = link.status()
        r2, _ = link.request("command", command="STOP")
        rep.step("16 silent laptop during a simulated movement -> watchdog clears it",
                 _ok(r) and bool(evt) and evt.get("simulated") is True and st.get("simulated_movement") == "none"
                 and st.get("watchdog_ok") is False and _ok(r2), cleared_after_ms=round(ms, 1))

        # 17 connection lost during a simulated movement -> cleared, reported, not resumed
        link.request("command", command="LEFT")
        link.close()
        time.sleep(0.3)
        link = RawLink(host, port)
        hello3, _ = link.wait_event("hello", 2.0)
        st = link.status()
        rep.step("17 connection lost during a simulated movement -> cleared, comm_fault reported, not resumed",
                 bool(hello3) and hello3.get("comm_fault") is True and st.get("simulated_movement") == "none",
                 hello_after_reconnect=hello3)
        link.request("command", command="STOP")
        link.request("led", state="off")
        link.close()

        # 18 RobotLink end to end
        client = RobotClient(host, port)
        robot = RobotLink(client).start()
        connected = client.wait_connected(3.0) and client.wait_status(3.0)
        time.sleep(client_seconds)
        st = client.status()
        level_idle = robot.safety().level.name
        t0 = time.perf_counter()
        robot.emergency("comm_test")
        end = time.monotonic() + 2.0
        while time.monotonic() < end and (client.emergency_pending or not client.commanded["emergency"]):
            time.sleep(0.005)
        em_ms = (time.perf_counter() - t0) * 1000.0
        client.send("status_request")
        client.wait_status(1.5)
        confirmed = client.robot_emergency
        blocked = robot.manual("forward")
        reset = robot.clear_emergency(True)
        client.send("status_request")
        client.wait_status(1.5)
        allowed = robot.manual("backward") if motion_ok else None
        if allowed is not None and allowed.ticket is not None:
            allowed_reply = allowed.ticket.wait(1.0)
        else:
            allowed_reply = None
        robot.close()
        ok15 = (connected and st["stats"]["timeouts"] == 0 and level_idle == "IDLE" and em_ms < 1000 and confirmed
                and not blocked.sent and reset.ok and (allowed is None or (allowed.sent and _ok(
                    allowed_reply.data if allowed_reply else None))))
        rep.step("18 RobotLink: heartbeat, supervisor, emergency (commanded + confirmed), clear, manual", ok15,
                 heartbeat_seconds=client_seconds, acks=st["stats"]["ok"], timeouts=st["stats"]["timeouts"],
                 rtt_mean_ms=st["rtt_mean_ms"], rtt_p95_ms=st["rtt_p95_ms"], safety_level_idle=level_idle,
                 emergency_ack_ms=round(em_ms, 1), emergency_confirmed_by_status=confirmed,
                 manual_blocked_reason=blocked.reason, manual_after_clear=allowed.reason if allowed else "skipped")

        if manual_network_test and motion_ok:
            _manual_network_test(host, port, rep)
    except (OSError, ConnectionError) as exc:
        rep.step("unexpected connection error", False, error=str(exc))
    finally:
        try:
            link.close()
        except Exception:
            pass
    return _finish(rep, save, out_dir)


def _manual_network_test(host: str, port: int, rep: Report) -> None:
    print("\nMANUAL STEP: the robot will be told to keep 'moving' (logical only if motors are disabled).")
    input("Press Enter, then within 5 s switch the LAPTOP's Wi-Fi OFF, wait 5 s, switch it ON again...")
    link = RawLink(host, port)
    link.wait_event("hello", 2.0)
    before = link.status().get("watchdog_trips", 0)
    end = time.monotonic() + 20
    try:
        while time.monotonic() < end:     # keep renewing the movement until the network disappears
            link.request("motor", timeout=0.3, mode="manual", left=20, right=20, duration_ms=600)
            time.sleep(0.2)
    except OSError:
        pass
    link.close()
    print("Waiting for the network to come back (up to 60 s)...")
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline:
        try:
            link = RawLink(host, port, timeout=2)
            break
        except OSError:
            time.sleep(1)
    else:
        rep.step("M network loss stops the robot", False, error="robot not reachable again within 60 s")
        return
    hello, _ = link.wait_event("hello", 2.0)
    st = link.status()
    link.close()
    rep.step("M network loss stops the robot (watchdog / disconnect)",
             st.get("motors_stopped") is True and (st.get("watchdog_trips", 0) > before
                                                   or bool(hello and hello.get("comm_fault"))),
             watchdog_trips_before=before, hello=hello, after=st)


def _finish(rep: Report, save: bool, out_dir: Path | None) -> dict:
    rep.data["finished"] = datetime.now().isoformat(timespec="seconds")
    rep.data["all_passed"] = rep.passed
    if save:
        d = out_dir or (ROOT / "evaluation" / "communication")
        d.mkdir(parents=True, exist_ok=True)
        path = d / f"comm_test_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        path.write_text(json.dumps(rep.data, indent=2), encoding="utf-8")
        rep.data["saved_to"] = str(path)
        print(f"Saved {path}")
    print("RESULT:", "ALL PASSED" if rep.passed else "SOME STEPS FAILED")
    return rep.data


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default=None, help="robot IP (default: robot.host in config/settings.yaml)")
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--pings", type=int, default=50)
    ap.add_argument("--allow-motion", action="store_true", help="run motion steps even if motors are enabled")
    ap.add_argument("--manual-network-test", action="store_true")
    args = ap.parse_args()
    host, port = args.host, args.port
    if host is None or port is None:
        sys.path.insert(0, str(ROOT))
        from src.config import load_config
        cfg = load_config()
        host = host or cfg.robot.host
        port = port or cfg.robot.port
    data = run_comm_test(host, port, pings=args.pings, allow_motion=args.allow_motion,
                         manual_network_test=args.manual_network_test)
    return 0 if data["all_passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
