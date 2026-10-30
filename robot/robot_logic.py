"""Robot decision layer on the laptop (protocol v3).

RobotLink is the ONE place through which the AI system, the follow controller
and the dashboard talk to the robot:

    caller -> RobotLink (asks the SafetySupervisor) -> CommandManager (priority queue)
           -> RobotClient (TCP) -> ESP32 (checks everything again)

  AI event / operator action            robot command(s)
  ------------------------------------------------------------------------------
  AI model processing frames            command MODEL_STARTED  (LED continuously ON) - once per state change
  AI model stopped / app closing        command MODEL_STOPPED  (LED and buzzer OFF)
  FALL_CONFIRMED (emergency alert)      command FALL_DETECTED  (all motion + following stopped, LED blinks,
                                        buzzer alert; movement refused until FALL_CLEARED)
  fall alert acknowledged by operator   command FALL_CLEARED   (back to LED ON while the model runs)
  (these AI states are re-sent after every reconnection, so the ESP32 always shows the current state)
  FALL_SUSPECTED / track lost in fall   led blink 200 ms + buzzer warning; following ends
  operator STOP                         stop  (also the "fresh control state" that
                                        acknowledges a communication fault)
  operator E-STOP                       emergency_stop {reason:"operator"}
  operator clear emergency (confirmed)  clear_emergency {operator_confirmed:true}
  operator press-and-hold arrows        motor {mode:"manual", ..., duration_ms: hold_ms},
                                        repeated while the button is held
  follow controller                     follow_mode / follow {target_id, ..., confidence}
  camera obstacle                       obstacle {source:"camera"} (repeated), obstacle_clear
  test buttons                          led / buzzer / mute

What the laptop *commanded* and what the robot *reports* (status every second)
are kept apart: an accepted command is not proof that the wheels stopped.
"""
from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass

from src.safety.safety_supervisor import SafetyDecision, SafetyInputs, SafetySupervisor

from .command_manager import CommandManager, Ticket
from .commands import CommandError, Reply
from .wifi_client import Pending, RobotClient, RobotNotConnected, RobotTimeout

log = logging.getLogger("sentinel.robot_logic")

MANUAL_DIRECTIONS = ("forward", "backward", "left", "right")
LOGGED_COMMANDS = ("stop", "emergency_stop", "clear_emergency", "follow_mode", "led", "buzzer", "mute",
                   "obstacle_clear")


@dataclass
class MoveDecision:
    sent: bool
    reason: str
    ticket: Ticket | None = None
    reply: Reply | None = None


class RobotLink:
    def __init__(self, client: RobotClient, max_speed_pct: int = 40, default_speed_pct: int = 30,
                 manual_hold_ms: int = 400, manual_active_s: float = 2.0, max_command_age_s: float = 0.3,
                 event_logger=None, signal_on_suspected: bool = True, operator_timeout_s: float = 1.0):
        self.client = client
        self.max_speed_pct = int(max_speed_pct)
        self.default_speed_pct = int(default_speed_pct)
        self.manual_hold_ms = int(manual_hold_ms)
        self.manual_active_s = float(manual_active_s)
        self.signal_on_suspected = signal_on_suspected
        self.operator_timeout_s = operator_timeout_s
        self.logger = event_logger
        self.supervisor = SafetySupervisor()
        self.manager = CommandManager(client, max_age_s=max_command_age_s)
        self.follow = None                      # FollowManager attaches itself
        self.camera_obstacle = False
        self.fall_suspected = False
        self.fall_confirmed = False
        self.last_manual = 0.0
        self.last_decision: MoveDecision | None = None
        self.warning_signal_on = False
        # AI state mirrored on the ESP32 (LED/buzzer). Changed only on state changes, from any thread.
        self._ai_lock = threading.Lock()
        self.model_running = False
        self.fall_alert = False
        client.add_listener(self._on_robot_message)
        client.add_ack_listener(self._on_ack)

    # -------------------------------------------------------------- lifecycle
    def start(self) -> "RobotLink":
        self.client.start()
        self.manager.start()
        return self

    def close(self) -> None:
        """Safe shutdown: stop following, tell the ESP32 the model stopped, stop the robot, close the link."""
        if self.follow is not None:
            self.follow.disable("application closing", send=False)
        self.set_model_running(False, "application closing")
        deadline = time.monotonic() + 1.0   # let queued messages (e.g. MODEL_STOPPED) reach the ESP32
        while self.manager.status()["queued"] and time.monotonic() < deadline and self.client.connected:
            time.sleep(0.02)
        time.sleep(0.05)
        self.manager.close()
        self.client.close(send_stop=True)
        self._log("ROBOT_SHUTDOWN", reason="laptop sent stop and closed the link")

    # ---------------------------------------------------------------- safety
    def safety_inputs(self) -> SafetyInputs:
        c = self.client
        return SafetyInputs(
            robot_connected=c.connected, status_fresh=c.status_fresh, robot_emergency=c.robot_emergency,
            laptop_emergency=c.emergency_latched, watchdog_ok=c.robot_watchdog_ok,
            comm_fault_reported=c.comm_fault_reported or c.link_lost_while_active,
            robot_obstacle=c.robot_obstacle, camera_obstacle=self.camera_obstacle,
            fall_suspected=self.fall_suspected,
            fall_confirmed=self.fall_confirmed or self.fall_alert or bool(c.connected and c.robot.get("fall_alert")),
            follow_state=self.follow.sm.state.value if self.follow is not None else "FOLLOW_DISABLED",
            manual_active=self.manual_active())

    def safety(self) -> SafetyDecision:
        return self.supervisor.evaluate(self.safety_inputs())

    def manual_active(self) -> bool:
        return time.monotonic() - self.last_manual < self.manual_active_s

    def set_fall_flags(self, suspected: bool, confirmed: bool) -> None:
        """Called by the app after every frame (from the alert manager / fall states)."""
        self.fall_suspected, self.fall_confirmed = bool(suspected), bool(confirmed)

    # ------------------------------------------------------------- AI alerts
    # ------------------------------------------------------------ AI state
    def set_model_running(self, running: bool, reason: str = "") -> Ticket | None:
        """AI model started / stopped. Sends MODEL_STARTED / MODEL_STOPPED only when the state changes."""
        with self._ai_lock:
            if bool(running) == self.model_running:
                return None
            self.model_running = bool(running)
        return self._send_ai("MODEL_STARTED" if running else "MODEL_STOPPED", reason)

    def set_fall_alert(self, active: bool, reason: str = "", track_id=None) -> Ticket | None:
        """A confirmed fall alert is active / was cleared. Sends FALL_DETECTED / FALL_CLEARED on changes only."""
        with self._ai_lock:
            if bool(active) == self.fall_alert:
                return None
            self.fall_alert = bool(active)
        if active and self.follow is not None:
            self.follow.disable("fall detected - following stopped", send=False)
        return self._send_ai("FALL_DETECTED" if active else "FALL_CLEARED", reason, track_id)

    def _send_ai(self, name: str, reason: str = "", track_id=None) -> Ticket | None:
        self._log("ROBOT_AI_EVENT", track_id=track_id, reason=f"{name}" + (f": {reason}" if reason else ""),
                  extra={"robot_command": name})
        if not self.client.connected:
            log.info("ESP32 not connected - %s will be sent as soon as it connects", name)
            return None                   # sent automatically after the next connection (_resync_ai)
        log.info("ESP32 <- %s (%s)", name, reason)
        try:
            return self.manager.submit("command", "ai", command=name)
        except CommandError as exc:
            log.error("AI event %s not sent: %s", name, exc)
            return None

    def _resync_ai(self) -> None:
        """After every (re)connection: show the laptop's current AI state on the ESP32."""
        with self._ai_lock:
            running, alert = self.model_running, self.fall_alert
        log.info("ESP32 connected at %s - sending the current AI state (model %s, fall alert %s)",
                 self.client.host, "running" if running else "not running", "ACTIVE" if alert else "none")
        try:
            self.manager.submit("command", "ai", command="FALL_DETECTED" if alert else "FALL_CLEARED")
            if running:
                self.manager.submit("command", "ai", command="MODEL_STARTED")
        except CommandError as exc:
            log.error("AI state not re-sent: %s", exc)

    def on_alert(self, alert) -> None:
        """AlertManager listener."""
        severity = getattr(alert, "severity", "")
        if severity == "emergency":
            # confirmed fall: FALL_DETECTED stops all motion and following on the ESP32, blinks the LED and
            # sounds the fall alert; movement stays refused until the operator acknowledges (FALL_CLEARED).
            # The operator E-STOP (emergency_stop) is separate and unchanged.
            log.warning("Forwarding fall alert to robot: %s", alert.message)
            self.set_fall_alert(True, f"alert {alert.alert_id}: {alert.message}",
                                track_id=getattr(alert, "track_id", None))
        elif severity == "warning":
            if self.follow is not None:
                self.follow.disable("possible fall - following stopped")
            if self.signal_on_suspected:
                self.signal_warning(f"alert {getattr(alert, 'alert_id', '')}: {getattr(alert, 'message', '')}")

    def signal_warning(self, reason: str = "") -> None:
        """Warning signal on the robot (LED blink 200 ms + warning beeps). Never blocks."""
        if self.client.emergency_latched or not self.client.connected:
            return
        try:
            self.manager.submit("led", "warning", state="blink", interval_ms=200)
            self.manager.submit("buzzer", "warning", pattern="warning")
            self.warning_signal_on = True
            self._log("ROBOT_WARNING_SIGNAL", reason=reason)
        except CommandError as exc:
            log.warning("warning signal not sent: %s", exc)

    def clear_warning(self, reason: str = "") -> None:
        """End the warning signal (person recovered / warning acknowledged). Never blocks."""
        if not self.warning_signal_on or self.client.emergency_latched or not self.client.connected:
            return
        self.manager.submit("led", "warning", state="off")
        self.warning_signal_on = False
        self._log("ROBOT_WARNING_CLEARED", reason=reason)

    def emergency(self, reason: str = "operator", detail: str = "", track_id=None) -> Ticket:
        """Highest priority. Never blocks; retried by the link until the robot acknowledges it."""
        if self.follow is not None:
            self.follow.disable("emergency stop", send=False)
        t = self.manager.submit("emergency_stop", "emergency", reason=reason)
        self._log("ROBOT_EMERGENCY_SENT", track_id=track_id, reason=detail or reason,
                  extra={"emergency_reason": reason, "note": "commanded; see robot status for confirmation"})
        return t

    # ---------------------------------------------------------- operator tools
    def _operator(self, msg_type: str, **fields) -> Reply:
        t = self.manager.submit(msg_type, "operator", **fields)
        reply = t.wait(self.operator_timeout_s)
        if reply is None:
            if t.outcome in ("dropped", "failed"):
                raise RobotNotConnected(t.detail or t.outcome)
            raise RobotTimeout(f"no ack for {msg_type}")
        return reply

    def stop(self, reason: str = "operator stop") -> Reply:
        """Stop everything. Also the fresh control state that acknowledges a communication fault."""
        if self.follow is not None:
            self.follow.disable(reason, send=False)
        reply = self._operator("stop")
        if reply.ok:
            # a STOP accepted on a healthy link is the fresh control state that ends a comm fault;
            # ask for a fresh status so "reported" (e.g. watchdog_ok) is up to date at once
            self.client.acknowledge_comm_fault()
            try:
                self.client.send_nowait("status_request")
                self.client.wait_status(0.5)
            except RobotNotConnected:
                pass
            self.supervisor.acknowledge_comm_fault(self.safety_inputs())
        self._log("ROBOT_STOP", reason=reason, acknowledged=reply.ok)
        return reply

    def mute(self) -> Reply:
        return self._operator("mute")

    def clear_emergency(self, operator_confirmed: bool) -> Reply:
        """Operator reset after reviewing the situation. Motors stay stopped afterwards."""
        if operator_confirmed is not True:
            raise CommandError("clear_emergency needs the operator's confirmation")
        reply = self.client.clear_emergency(timeout=self.operator_timeout_s)
        if reply.ok:
            self.warning_signal_on = False
        self._log("ROBOT_EMERGENCY_CLEARED" if reply.ok else "ROBOT_EMERGENCY_CLEAR_REFUSED",
                  reason="operator confirmed clear_emergency" if reply.ok else f"robot refused: {reply.error}",
                  acknowledged=reply.ok, extra={"reset_time": time.strftime("%Y-%m-%dT%H:%M:%S")})
        return reply

    def set_led(self, state: str, interval_ms: int | None = None) -> Reply:
        return self._operator("led", state=str(state).lower(), interval_ms=interval_ms)

    def buzzer(self, pattern: str) -> Reply:
        return self._operator("buzzer", pattern=str(pattern).lower())

    def acknowledge_comm_fault(self) -> tuple[bool, str]:
        """Same as STOP (a fresh control state)."""
        try:
            r = self.stop("operator acknowledged the communication fault")
        except (RobotNotConnected, RobotTimeout) as exc:
            return False, str(exc)
        return r.ok, "acknowledged" if r.ok else f"robot refused: {r.error}"

    # ------------------------------------------------------------- manual drive
    def manual(self, direction: str, speed_pct: int | None = None, created_at: float | None = None
               ) -> MoveDecision:
        """Press-and-hold control: call repeatedly (every ~150 ms) while the button is held.
        Each command expires on the robot after manual_hold_ms, so releasing the button
        (or a crashed browser) stops the robot within that time."""
        direction = str(direction).lower()
        if direction not in MANUAL_DIRECTIONS:
            return self._decide(MoveDecision(False, f"unknown direction {direction}"))
        s = min(self.default_speed_pct if speed_pct is None else int(speed_pct), self.max_speed_pct)
        if s <= 0:
            return self._decide(MoveDecision(False, "speed must be > 0"))
        left, right = {"forward": (s, s), "backward": (-s, -s), "left": (-s, s), "right": (s, -s)}[direction]
        return self.drive(left, right, self.manual_hold_ms, created_at)

    def drive(self, left: int, right: int, duration_ms: int | None = None, created_at: float | None = None
              ) -> MoveDecision:
        try:
            left, right = int(left), int(right)
        except (TypeError, ValueError):
            return self._decide(MoveDecision(False, "wheel speeds must be whole numbers"))
        if not (-100 <= left <= 100 and -100 <= right <= 100):
            return self._decide(MoveDecision(False, f"wheel speeds {left},{right} outside -100..100"))
        lim = self.max_speed_pct
        left, right = max(-lim, min(lim, left)), max(-lim, min(lim, right))
        why = self.supervisor.check_motion(self.safety_inputs(), "manual", left, right)
        if why:
            return self._decide(MoveDecision(False, why))
        self.last_manual = time.monotonic()
        try:
            t = self.manager.submit("motor", "manual", created_at=created_at, mode="manual", left=left,
                                    right=right, duration_ms=int(duration_ms or self.manual_hold_ms))
        except CommandError as exc:
            return self._decide(MoveDecision(False, str(exc)))
        return self._decide(MoveDecision(True, f"motor {left}/{right} queued", t))

    # -------------------------------------------------------------- following
    def follow_mode(self, enabled: bool, target_id: int | None = None) -> Ticket:
        return self.manager.submit("follow_mode", "follow", enabled=bool(enabled),
                                   target_id=target_id if enabled else None)

    def follow_drive(self, target_id: int, left: int, right: int, duration_ms: int, confidence: float,
                     created_at: float | None = None) -> MoveDecision:
        lim = self.max_speed_pct
        left, right = max(-lim, min(lim, int(left))), max(-lim, min(lim, int(right)))
        why = self.supervisor.check_motion(self.safety_inputs(), "follow", left, right)
        if why:
            left, right = 0, 0               # still send a zero-speed request: the robot holds still
        try:
            t = self.manager.submit("follow", "follow", created_at=created_at, target_id=int(target_id),
                                    left=left, right=right, duration_ms=int(duration_ms),
                                    confidence=max(0.0, min(1.0, float(confidence))))
        except CommandError as exc:
            return self._decide(MoveDecision(False, str(exc)))
        return self._decide(MoveDecision(not why, why or f"follow {left}/{right} queued", t))

    # ------------------------------------------------------------- obstacles
    def report_obstacle(self, distance_cm: int = 0) -> None:
        """Camera saw something in the path. Repeat while seen (the robot holds it for 1.5 s)."""
        self.camera_obstacle = True
        try:
            self.manager.submit("obstacle", "camera", source="camera", distance_cm=max(0, min(999, int(distance_cm))))
        except CommandError as exc:
            log.debug("obstacle report not sent: %s", exc)

    def clear_obstacle(self) -> None:
        self.camera_obstacle = False
        self.manager.submit("obstacle_clear", "camera")

    # ------------------------------------------------------------------ status
    def status(self) -> dict:
        st = self.client.status()
        st["link_enabled"] = True
        st["safety"] = self.safety().to_dict()
        st["max_speed_pct"] = self.max_speed_pct
        st["manual_hold_ms"] = self.manual_hold_ms
        st["queue"] = self.manager.status()
        st["camera_obstacle"] = self.camera_obstacle
        st["ai_state"] = {"model_running": self.model_running, "fall_alert": self.fall_alert}
        if self.last_decision:
            st["last_move"] = {"sent": self.last_decision.sent, "reason": self.last_decision.reason}
        return st

    def snapshot(self) -> dict:
        """Short robot state for the event log."""
        c = self.client
        r = c.robot if c.connected else {}
        return {"comm": "connected" if c.connected else "disconnected",
                "robot_state": r.get("state"), "motors_reported": f"{r.get('left')}/{r.get('right')}" if r else None,
                "motors_commanded": f"{c.commanded['left']}/{c.commanded['right']}",
                "led": r.get("led"), "buzzer": r.get("buzzer"), "emergency_active": r.get("emergency_active")}

    # -------------------------------------------------------------- internals
    def _decide(self, d: MoveDecision) -> MoveDecision:
        self.last_decision = d
        return d

    def _on_ack(self, p: Pending, reply: Reply | None) -> None:
        if p.cmd.type not in LOGGED_COMMANDS and (reply is None or reply.ok or p.cmd.type in ("ping",)
                                                  or reply.error == "LOW_CONFIDENCE"):
            return
        if p.cmd.type == "ping":
            return
        if reply is None:
            self._log("ROBOT_NO_ACK", reason=f"{p.cmd} ({p.command_id}) not acknowledged",
                      extra={"robot_command": str(p.cmd), "command_id": p.command_id})
        else:
            self._log("ROBOT_ACK" if reply.ok else "ROBOT_REFUSED",
                      reason=f"{p.cmd} -> {'ok' if reply.ok else reply.error}",
                      extra={"robot_command": str(p.cmd), "command_id": p.command_id,
                             "robot_ack": "ok" if reply.ok else reply.error})

    def _on_robot_message(self, reply: Reply) -> None:
        if reply.kind != "event":
            return
        ev = reply.event
        if ev in ("watchdog_stop", "obstacle", "obstacle_clear", "buzzer_timeout", "replaced"):
            self._log(f"ROBOT_{ev.upper()}", reason=reply.raw)
        elif ev == "hello":
            self._log("ROBOT_CONNECTED", reason=f"robot state {reply.data.get('state')}, motors_enabled="
                      f"{reply.data.get('motors_enabled')}, comm_fault={reply.data.get('comm_fault')}")
            if self.follow is not None:
                # the robot is never in follow mode after a (re)connection: nothing resumes by itself
                self.follow.disable("link re-established - select the person again", send=False)
            self._resync_ai()

    def _log(self, etype: str, **kw) -> None:
        if self.logger is None:
            return
        extra = dict(kw.pop("extra", None) or {})
        extra["robot"] = self.snapshot()
        try:
            self.logger.log(etype, extra=extra, **kw)
        except Exception as exc:   # logging must never break robot safety
            log.error("event log failed: %s", exc)


def create_robot_link(robot_cfg, event_logger=None) -> RobotLink:
    """Build RobotClient + RobotLink from the 'robot' section of settings.yaml (not started)."""
    c = robot_cfg
    client = RobotClient(c["host"], c["port"], heartbeat_s=c["heartbeat_s"], ack_timeout_s=c["ack_timeout_s"],
                         connect_timeout_s=c["connect_timeout_s"], reconnect_delay_s=c["reconnect_delay_s"],
                         reconnect_max_delay_s=c["reconnect_max_delay_s"],
                         emergency_retry_s=c["emergency_retry_s"], max_command_age_s=c["max_command_age_s"],
                         max_missed_acks=c["max_missed_acks"], status_stale_s=c.get("status_stale_s", 2.5))
    return RobotLink(client, max_speed_pct=c["max_speed_pct"], default_speed_pct=c["default_speed_pct"],
                     manual_hold_ms=c.get("manual_hold_ms", 400), manual_active_s=c.get("manual_active_s", 2.0),
                     max_command_age_s=c["max_command_age_s"], event_logger=event_logger,
                     signal_on_suspected=c.get("signal_on_suspected", True))
