"""Laptop side of the laptop <-> ESP32 Wi-Fi link (TCP, protocol v3 JSON lines).

RobotClient keeps one TCP connection to the robot in a background thread:
  * connects, reads the robot's ``hello`` event, reconnects with back-off when the
    link drops - but a reconnection NEVER resumes movement (nothing is re-sent
    except a latched emergency);
  * sends a ``ping`` heartbeat every ``heartbeat_s`` so the robot's watchdog knows
    the laptop is alive (no valid message for 500 ms while moving -> robot stops);
  * gives every command a unique ``command_id`` and a per-connection ``seq``,
    matches every ``ack`` to its command and measures the round-trip time;
  * keeps the robot's latest ``status`` (sent every second) separate from what
    the laptop *commanded*: the dashboard shows both ("commanded" vs "reported");
  * gives ``emergency_stop`` top priority: sent immediately, re-sent until the
    robot acknowledges it, and re-sent after a reconnection if the robot came
    back without its emergency latched (e.g. after a power glitch);
  * refuses stale motion commands (older than ``max_command_age_s``);
  * limits incoming line length (a robot line is at most 640 characters).

Nothing here blocks the AI pipeline: public methods either return quickly or
wait at most ``ack_timeout_s``. Higher-level ordering (emergency first,
coalescing motion) is done by robot/command_manager.py.
"""
from __future__ import annotations

import logging
import socket
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable

from . import discovery
from .commands import (MAX_SEQ, MOTION_TYPES, PROTOCOL_VERSION, Command, CommandError, ProtocolError, Reply,
                       encode, make_command, parse_line)

log = logging.getLogger("sentinel.robot")

MAX_RX_LINE = 1024          # robot lines are <= 640 characters; anything longer = broken link


class RobotNotConnected(ConnectionError):
    pass


class RobotTimeout(TimeoutError):
    pass


@dataclass
class Pending:
    """One sent command waiting for its ack."""
    seq: int
    command_id: str
    cmd: Command
    sent_at: float
    event: threading.Event = field(default_factory=threading.Event)
    reply: Reply | None = None
    background: bool = False
    timed_out: bool = False

    def wait(self, timeout: float) -> Reply | None:
        self.event.wait(timeout)
        return self.reply


class RobotClient:
    def __init__(self, host: str, port: int = 3333, heartbeat_s: float = 0.15, ack_timeout_s: float = 0.4,
                 connect_timeout_s: float = 2.0, reconnect_delay_s: float = 1.0,
                 reconnect_max_delay_s: float = 5.0, emergency_retry_s: float = 0.25,
                 max_command_age_s: float = 0.3, max_missed_acks: int = 3, status_stale_s: float = 2.5):
        # host "auto" (or empty): find the ESP32 on the local network (robot/discovery.py) and
        # search again whenever it cannot be reached (a hotspot may give it a new address)
        self.auto = discovery.is_auto(host)
        self.host = (discovery.read_cache() or "") if self.auto else host
        self.port = int(port)
        self._search_needed = self.auto and not self.host
        self.heartbeat_s = heartbeat_s
        self.ack_timeout_s = ack_timeout_s
        self.connect_timeout_s = connect_timeout_s
        self.reconnect_delay_s = reconnect_delay_s
        self.reconnect_max_delay_s = reconnect_max_delay_s
        self.emergency_retry_s = emergency_retry_s
        self.max_command_age_s = max_command_age_s
        self.max_missed_acks = max_missed_acks
        self.status_stale_s = status_stale_s

        self._sock: socket.socket | None = None
        self._write_lock = threading.Lock()
        self._state_lock = threading.RLock()
        self._pending: dict[int, Pending] = {}
        self._seq = 0
        self._stop = threading.Event()
        self._enabled = threading.Event()
        self._enabled.set()
        self._thread: threading.Thread | None = None
        self._connected = threading.Event()
        self._rx = b""
        self._last_send = 0.0
        self._missed = 0
        self._emergency_reason: str | None = None   # emergency to (re)send until acknowledged
        self._emergency_last_try = 0.0
        self.emergency_latched = False              # laptop believes the robot must be in emergency
        self.heartbeat_paused = False               # tests only: simulate a frozen laptop
        self._listeners: list[Callable[[Reply], None]] = []
        self._ack_listeners: list[Callable[[Pending, Reply | None], None]] = []

        self.hello: dict = {}                       # last hello event
        self.robot: dict = {}                       # last status message (what the robot reports)
        self.status_time = 0.0
        self.commanded: dict = {"left": 0, "right": 0, "led": "off", "buzzer": "off", "follow_mode": False,
                                "target_id": None, "emergency": False, "last_command_id": None,
                                "simulated_movement": "none", "model_running": False, "fall_alert": False}
        self.comm_fault_reported = False            # robot said: previous link failed while moving
        self.link_lost_while_active = False         # laptop saw the link drop while moving/following
        self.replaced = False                       # another laptop took over - we stopped reconnecting
        self.stats = {"sent": 0, "ok": 0, "err": 0, "timeouts": 0, "events": 0, "status": 0, "connects": 0,
                      "disconnects": 0, "emergency_sent": 0, "refused_local": 0, "watchdog_stops": 0}
        self._rtts: deque = deque(maxlen=200)
        self._raw_log: deque = deque(maxlen=50)
        self.last_error = ""
        self.last_rx_time = 0.0
        self.connected_since = 0.0

    # ---------------------------------------------------------------- lifecycle
    def start(self) -> "RobotClient":
        if self._thread and self._thread.is_alive():
            return self
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="robot-link", daemon=True)
        self._thread.start()
        return self

    def close(self, send_stop: bool = True) -> None:
        """Safe shutdown: ask the robot to stop, then close the link (the robot also stops by itself)."""
        if send_stop and self.connected:
            try:
                p = self._transmit(make_command("stop"), background=True)
                p.wait(self.ack_timeout_s)
            except (RobotNotConnected, CommandError):
                pass
        self._stop.set()
        self._disconnect("client closed")
        if self._thread:
            self._thread.join(timeout=3)

    def set_enabled(self, enabled: bool) -> None:
        """Dashboard connect / disconnect. Disconnecting first asks the robot to stop."""
        if enabled:
            self.replaced = False
            self._enabled.set()
            return
        self._enabled.clear()
        if self.connected:
            try:
                self._transmit(make_command("stop"), background=True).wait(self.ack_timeout_s)
            except (RobotNotConnected, CommandError):
                pass
        self._disconnect("operator disconnected")

    @property
    def enabled(self) -> bool:
        return self._enabled.is_set()

    def wait_connected(self, timeout: float = 5.0) -> bool:
        return self._connected.wait(timeout)

    def wait_status(self, timeout: float = 3.0) -> bool:
        t0 = time.monotonic()
        before = self.status_time
        while time.monotonic() - t0 < timeout:
            if self.status_time > before:
                return True
            time.sleep(0.01)
        return False

    @property
    def connected(self) -> bool:
        return self._connected.is_set()

    def add_listener(self, fn: Callable[[Reply], None]) -> None:
        """Called (from the link thread) for every event and status message from the robot."""
        self._listeners.append(fn)

    def add_ack_listener(self, fn: Callable[[Pending, Reply | None], None]) -> None:
        """Called for every command when its ack arrives (or None when it timed out). For the event log."""
        self._ack_listeners.append(fn)

    # --------------------------------------------------------------- public API
    def send(self, msg_type: str, timeout: float | None = None, **fields) -> Reply:
        """Validate, send and wait for the ack. Raises CommandError/RobotNotConnected/RobotTimeout."""
        p = self.send_command(make_command(msg_type, **fields))
        return self.wait_reply(p, timeout)

    def wait_reply(self, p: Pending, timeout: float | None = None) -> Reply:
        if not p.event.wait(self.ack_timeout_s if timeout is None else timeout):
            self._expire(p)
            raise RobotTimeout(f"no ack for {p.cmd} ({p.command_id}) within {self.ack_timeout_s:.2f}s")
        if p.reply is None:
            raise RobotNotConnected("link dropped while waiting for the ack" if not p.timed_out
                                    else f"no ack for {p.cmd}")
        return p.reply

    def send_nowait(self, msg_type: str, created_at: float | None = None, **fields) -> Pending:
        """Validate and send without waiting (the ack is still matched, timed and logged)."""
        return self.send_command(make_command(msg_type, **fields), created_at=created_at, background=True)

    def send_command(self, cmd: Command, created_at: float | None = None, background: bool = False) -> Pending:
        if cmd.type in MOTION_TYPES and created_at is not None:
            age = time.monotonic() - created_at
            if age > self.max_command_age_s:
                self.stats["refused_local"] += 1
                raise CommandError(f"stale {cmd.type} command ({age:.2f}s old) not sent")
        return self._transmit(cmd, background=background)

    def trigger_emergency(self, reason: str = "fall_confirmed") -> None:
        """Highest priority. Returns immediately; the link thread retries until acknowledged."""
        make_command("emergency_stop", reason=reason)          # validates the reason
        with self._state_lock:
            self._emergency_reason = reason
            self.emergency_latched = True
            self.commanded["emergency"] = True
            self._emergency_last_try = 0.0
        if self.connected:
            self._try_emergency()

    def clear_emergency(self, timeout: float | None = None) -> Reply:
        """Operator-confirmed reset. The laptop latch is released only when the robot accepted it."""
        reply = self.send("clear_emergency", timeout=timeout, operator_confirmed=True)
        if reply.ok:
            with self._state_lock:
                self._emergency_reason = None
                self.emergency_latched = False
                self.commanded.update(emergency=False, buzzer="off", led="off", follow_mode=False, target_id=None,
                                      simulated_movement="none")
        return reply

    @property
    def emergency_pending(self) -> bool:
        return self._emergency_reason is not None

    # ------------------------------------------------------ robot-reported state
    def status_age_s(self) -> float:
        return time.monotonic() - self.status_time if self.status_time else float("inf")

    def link_age_s(self) -> float:
        """Seconds since anything was received from the robot (inf if never)."""
        return time.monotonic() - self.last_rx_time if self.last_rx_time else float("inf")

    @property
    def status_fresh(self) -> bool:
        return self.connected and self.status_age_s() <= self.status_stale_s

    @property
    def robot_state(self) -> str:
        return self.robot.get("state", "unknown") if self.connected else "unknown"

    @property
    def robot_emergency(self) -> bool:
        return bool(self.connected and self.robot.get("emergency_active"))

    @property
    def robot_obstacle(self) -> bool:
        return bool(self.connected and self.robot.get("obstacle_detected"))

    @property
    def robot_follow(self) -> bool:
        return bool(self.connected and self.robot.get("follow_mode"))

    @property
    def robot_watchdog_ok(self) -> bool:
        return bool(self.robot.get("watchdog_ok", True))

    def status(self) -> dict:
        rtts = sorted(self._rtts)
        reported = dict(self.robot) if self.connected else {}
        reported.pop("version", None)
        reported.pop("type", None)
        return {
            "host": self.host, "port": self.port, "auto_host": self.auto,
            "enabled": self.enabled, "connected": self.connected,
            "replaced_by_other_laptop": self.replaced,
            "protocol": self.hello.get("protocol"), "motors_enabled": self.hello.get("motors_enabled"),
            "sensor_installed": self.hello.get("sensor_installed"),
            "comm_fault_reported": self.comm_fault_reported,
            "link_lost_while_active": self.link_lost_while_active,
            "emergency_latched": self.emergency_latched, "emergency_pending": self.emergency_pending,
            "commanded": dict(self.commanded), "reported": reported,
            "status_age_s": round(self.status_age_s(), 2) if self.status_time and self.connected else None,
            "last_rx_age_s": round(self.link_age_s(), 3) if self.last_rx_time else None,
            "rtt_last_ms": round(self._rtts[-1], 1) if self._rtts else None,
            "rtt_mean_ms": round(sum(rtts) / len(rtts), 1) if rtts else None,
            "rtt_p95_ms": round(rtts[int(0.95 * (len(rtts) - 1))], 1) if rtts else None,
            "stats": dict(self.stats), "last_error": self.last_error,
        }

    def rtt_samples(self) -> list[float]:
        return list(self._rtts)

    # ------------------------------------------------------- tests / tooling
    def send_raw(self, text: str) -> None:
        """Write an arbitrary line (no validation). Only for protocol tests."""
        sock = self._sock
        if sock is None:
            raise RobotNotConnected("not connected")
        with self._write_lock:
            sock.sendall(text.encode("ascii", "replace") + b"\n")

    def send_test_line(self, template: str) -> int:
        """Tests only: send a hand-made (possibly invalid) line. '{seq}' in the template is replaced by
        the next sequence number, which is reserved so later commands stay in order. Returns that seq."""
        sock = self._sock
        if sock is None:
            raise RobotNotConnected("not connected")
        with self._write_lock:
            with self._state_lock:
                self._seq = self._seq + 1 if self._seq < MAX_SEQ else 1
                seq = self._seq
            sock.sendall(template.replace("{seq}", str(seq)).encode("ascii", "replace") + b"\n")
            self._last_send = time.monotonic()
        return seq

    def last_raw_lines(self) -> list[str]:
        return list(self._raw_log)

    # ---------------------------------------------------------------- internals
    def _command_id(self, seq: int) -> str:
        return f"L{self.stats['connects']}-{seq}"

    def _transmit(self, cmd: Command, background: bool = False) -> Pending:
        sock = self._sock
        if sock is None or not self.connected:
            raise RobotNotConnected(f"robot {self.host}:{self.port} not connected"
                                    + (f" ({self.last_error})" if self.last_error else ""))
        with self._write_lock:
            with self._state_lock:
                self._seq = self._seq + 1 if self._seq < MAX_SEQ else 1
                seq = self._seq
                p = Pending(seq, self._command_id(seq), cmd, time.monotonic(), background=background)
                self._pending[seq] = p
            try:
                sock.sendall(encode(seq, p.command_id, cmd))
            except OSError as exc:
                with self._state_lock:
                    self._pending.pop(seq, None)
                self._disconnect(f"send failed: {exc}")
                raise RobotNotConnected(str(exc)) from exc
            self._last_send = time.monotonic()
            self.stats["sent"] += 1
        return p

    def _expire(self, p: Pending) -> None:
        with self._state_lock:
            if self._pending.pop(p.seq, None) is None:
                return
            self.stats["timeouts"] += 1
            self._missed += 1
            p.timed_out = True
        p.event.set()
        self._notify_ack(p, None)

    def _try_emergency(self) -> None:
        with self._state_lock:
            reason = self._emergency_reason
            if reason is None or time.monotonic() - self._emergency_last_try < self.emergency_retry_s:
                return
            self._emergency_last_try = time.monotonic()
        try:
            log.warning("Sending emergency_stop (%s) to robot", reason)
            self._transmit(make_command("emergency_stop", reason=reason), background=True)
            self.stats["emergency_sent"] += 1
        except RobotNotConnected:
            log.warning("emergency_stop queued: robot not connected (will retry)")

    def _run(self) -> None:
        delay = self.reconnect_delay_s
        while not self._stop.is_set():
            if not self._enabled.is_set():
                self._enabled.wait(0.2)
                continue
            if not self._connect():
                self._stop.wait(delay)
                delay = min(delay * 2, self.reconnect_max_delay_s)
                continue
            delay = self.reconnect_delay_s
            while not self._stop.is_set() and self.connected:
                self._poll()
            self._fail_pending()
            self._stop.wait(self.reconnect_delay_s)      # never hammer a robot that just dropped us

    def _connect(self) -> bool:
        if self.auto and self._search_needed:
            self.last_error = "searching the local network for the ESP32 ..."
            found = discovery.find_robot(self.port, stop=self._stop)
            if not found:
                nets = ", ".join(str(n) for n in discovery.local_networks()) or "no local network"
                self.last_error = (f"ESP32 not found (tried {discovery.MDNS_NAME} and port {self.port} on {nets}) - "
                                   "is it powered and on the same Wi-Fi as this laptop?")
                return False
            self.host = found
            self._search_needed = False
        try:
            sock = socket.create_connection((self.host, self.port), timeout=self.connect_timeout_s)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(0.02)
        except OSError as exc:
            self.last_error = f"cannot connect to {self.host or '(robot not found yet)'}:{self.port}: {exc}"
            if self.auto:
                self._search_needed = True      # the address may have changed - search again next time
            return False
        if self.auto:
            discovery.write_cache(self.host)
        with self._state_lock:
            self._sock = sock
            self._rx = b""
            self._seq = 0                  # the robot expects sequence numbers to restart
            self._missed = 0
            self.robot = {}
            self.status_time = 0.0
            self.hello = {}
            # after a reconnection the robot is stopped and not following
            self.commanded.update(left=0, right=0, follow_mode=False, target_id=None, simulated_movement="none",
                                  model_running=False)   # the ESP32 turns the model light off on disconnect
        self.stats["connects"] += 1
        self.connected_since = time.monotonic()
        self.last_error = ""
        self._connected.set()
        log.info("Robot link up: %s:%d", self.host, self.port)
        return True

    def _disconnect(self, reason: str) -> None:
        with self._state_lock:
            sock, self._sock = self._sock, None
            # decided before closing: closing makes the I/O thread's recv fail, and its own
            # _disconnect could otherwise clear _connected before this check (race)
            was_connected = sock is not None and self._connected.is_set()
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
            if was_connected:
                c = self.commanded
                if c["left"] or c["right"] or c["follow_mode"] or c.get("simulated_movement", "none") != "none":
                    self.link_lost_while_active = True
                self.stats["disconnects"] += 1
                log.warning("Robot link down: %s", reason)
            self.last_error = reason
        self._connected.clear()

    def _fail_pending(self) -> None:
        with self._state_lock:
            pending, self._pending = self._pending, {}
        for p in pending.values():
            p.event.set()          # reply stays None -> RobotNotConnected for waiters
            self._notify_ack(p, None)

    def _poll(self) -> None:
        sock = self._sock
        if sock is None:
            return
        try:
            data = sock.recv(4096)
            if not data:
                self._disconnect("robot closed the connection")
                return
            self._rx += data
        except socket.timeout:
            pass
        except OSError as exc:
            self._disconnect(f"receive failed: {exc}")
            return
        while b"\n" in self._rx:
            line, self._rx = self._rx.split(b"\n", 1)
            if len(line) > MAX_RX_LINE:
                log.warning("robot line too long (%d bytes) - ignored", len(line))
                continue
            self._handle_line(line.decode("ascii", "replace"))
            if not self.connected:
                return
        if len(self._rx) > MAX_RX_LINE:
            self._disconnect("robot sent an over-long line - link reset")
            return
        now = time.monotonic()
        # background commands (heartbeats, emergency, following) time out silently
        with self._state_lock:
            expired = [p for p in self._pending.values() if now - p.sent_at > self.ack_timeout_s and p.background]
        for p in expired:
            self._expire(p)
        if self._missed >= self.max_missed_acks:
            self._disconnect(f"{self._missed} acks missing - link unreliable")
            return
        if self._emergency_reason is not None:
            self._try_emergency()
        if not self.heartbeat_paused and now - self._last_send >= self.heartbeat_s:
            try:
                self._transmit(make_command("ping"), background=True)
            except RobotNotConnected:
                return

    def _notify_ack(self, p: Pending, reply: Reply | None) -> None:
        for fn in list(self._ack_listeners):
            try:
                fn(p, reply)
            except Exception as exc:   # a listener must never break the link
                log.error("robot ack listener failed: %s", exc)

    def _handle_line(self, text: str) -> None:
        self._raw_log.append(text)
        self.last_rx_time = time.monotonic()
        try:
            reply = parse_line(text)
        except ProtocolError as exc:
            log.warning("%s", exc)
            if "speaks protocol" in str(exc):
                self._incompatible(str(exc))
            return
        if reply.kind == "status":
            self.stats["status"] += 1
            self.robot = reply.data
            self.status_time = self.last_rx_time
            self._check_emergency_consistency()
            self._notify(reply)
            return
        if reply.kind == "event":
            self.stats["events"] += 1
            self._handle_event(reply)
            self._notify(reply)
            return
        with self._state_lock:
            p = self._pending.pop(reply.seq, None)
            if p is not None and p.command_id != reply.command_id:
                log.warning("ack %s does not match command %s", reply.command_id, p.command_id)
        self._missed = 0
        if reply.ok:
            self.stats["ok"] += 1
        else:
            self.stats["err"] += 1
            if p is None or not p.background or p.cmd.type not in ("ping",):
                log.info("robot refused %s: %s", p.cmd if p else reply.command_id, reply.error)
        if p is not None:
            self._rtts.append((self.last_rx_time - p.sent_at) * 1000.0)
            if reply.ok:
                self._apply_commanded(p.cmd, reply)
            elif p.cmd.type == "emergency_stop":
                pass                      # keep retrying
            p.reply = reply
            p.event.set()
            self._notify_ack(p, reply)

    def _apply_commanded(self, cmd: Command, reply: Reply) -> None:
        c = self.commanded
        c["last_command_id"] = reply.command_id
        t = cmd.type
        if t in MOTION_TYPES:
            c["left"], c["right"] = reply.data.get("left", 0), reply.data.get("right", 0)
        elif t == "stop":
            c.update(left=0, right=0, follow_mode=False, target_id=None)
        elif t == "follow_mode":
            c.update(follow_mode=cmd.get("enabled"), target_id=cmd.get("target_id"), left=0, right=0)
        elif t == "led":
            c["led"] = cmd.get("state")
        elif t == "buzzer":
            c["buzzer"] = cmd.get("pattern")
        elif t == "mute":
            c["buzzer"] = "off"
        elif t == "emergency_stop":
            c.update(left=0, right=0, follow_mode=False, target_id=None, emergency=True, led="on",
                     buzzer="emergency")
            with self._state_lock:
                if self._emergency_reason == cmd.get("reason"):
                    self._emergency_reason = None
            log.warning("Robot acknowledged emergency_stop (%s)", cmd.get("reason"))
        elif t == "command":
            self._apply_named(cmd.get("command"))

    def _apply_named(self, name: str) -> None:
        """Bookkeeping for accepted named commands (same meaning as the typed messages)."""
        c = self.commanded
        if name in ("FORWARD", "BACKWARD", "LEFT", "RIGHT"):
            c.update(simulated_movement=name.lower(), left=0, right=0)
        elif name == "STOP":
            c.update(simulated_movement="none", left=0, right=0, follow_mode=False, target_id=None)
        elif name.startswith("LED_"):
            c["led"] = name[4:].lower()
        elif name.startswith("BUZZER_"):
            c["buzzer"] = {"ON": "on", "OFF": "off", "SHORT_BEEP": "short_beep", "WARNING": "warning"}[name[7:]]
        elif name == "MODEL_STARTED":
            c.update(model_running=True, led="off")          # LED test overrides end; the model light shows
        elif name == "MODEL_STOPPED":
            c.update(model_running=False, fall_alert=False, led="off", buzzer="off")
        elif name == "FALL_DETECTED":
            c.update(fall_alert=True, simulated_movement="none", left=0, right=0, follow_mode=False, target_id=None)
        elif name == "FALL_CLEARED":
            c["fall_alert"] = False
        elif name == "EMERGENCY_STOP":
            # the robot latched an emergency: the laptop latches too (re-sent after a robot reboot)
            with self._state_lock:
                self.emergency_latched = True
            c.update(simulated_movement="none", left=0, right=0, follow_mode=False, target_id=None,
                     emergency=True, led="on", buzzer="emergency")
            log.warning("Robot acknowledged EMERGENCY_STOP")

    def _check_emergency_consistency(self) -> None:
        """The laptop latched an emergency but the robot reports none (e.g. robot rebooted) -> re-send."""
        if self.emergency_latched and not self.robot.get("emergency_active") and self._emergency_reason is None:
            log.warning("Robot reports no emergency while the laptop has one latched - re-sending emergency_stop")
            with self._state_lock:
                self._emergency_reason = "relatch"
                self._emergency_last_try = 0.0

    def _handle_event(self, reply: Reply) -> None:
        ev = reply.event
        if ev == "hello":
            self.hello = reply.data
            if reply.data.get("protocol") != PROTOCOL_VERSION:
                self._incompatible(f"robot speaks protocol v{reply.data.get('protocol')}, laptop "
                                   f"v{PROTOCOL_VERSION} - flash the matching firmware")
                return
            if reply.data.get("comm_fault"):
                self.comm_fault_reported = True
                log.warning("Robot reports a communication fault: the previous link failed while it was moving")
            if self.emergency_latched and reply.data.get("state") != "emergency":
                log.warning("Robot reconnected without its emergency latch - re-sending emergency_stop")
                with self._state_lock:
                    self._emergency_reason = self._emergency_reason or "relatch"
                    self._emergency_last_try = 0.0
        elif ev == "watchdog_stop":
            self.stats["watchdog_stops"] += 1
            self.comm_fault_reported = True
            self.commanded.update(left=0, right=0, simulated_movement="none")
            log.warning("Robot watchdog stopped %s (no valid message in time)",
                        "the simulated movement" if reply.data.get("simulated") else "the motors")
        elif ev == "move_done":
            self.commanded.update(left=0, right=0, simulated_movement="none")
        elif ev == "obstacle":
            log.warning("Robot sensor reports an obstacle at %s cm", reply.data.get("distance_cm"))
        elif ev == "buzzer_timeout":
            self.commanded["buzzer"] = "off"
        elif ev == "replaced":
            log.warning("Another laptop connected to the robot - this laptop stops reconnecting")
            self.replaced = True
            self._enabled.clear()
            self._disconnect("replaced by another laptop connection")

    def _incompatible(self, msg: str) -> None:
        """Wrong protocol version: stop reconnecting until the operator connects again."""
        log.error("%s", msg)
        if self.auto:                  # probably another device got the old address: look again
            self._search_needed = True
            self._disconnect(msg)
            self.last_error = msg
            return
        self._enabled.clear()
        self._disconnect(msg)
        self.last_error = msg

    def acknowledge_comm_fault(self) -> None:
        self.comm_fault_reported = False
        self.link_lost_while_active = False

    def _notify(self, reply: Reply) -> None:
        for fn in list(self._listeners):
            try:
                fn(reply)
            except Exception as exc:   # a listener must never break the link
                log.error("robot event listener failed: %s", exc)
