"""Simple, high-level ESP32 client: the methods the test console, the dashboard and the AI use.

It is a thin layer over robot/wifi_client.RobotClient (connection thread, heartbeat, ack
matching, reconnection, emergency retry) - nothing is duplicated, so what you test with the
console is exactly what the main SentinelAI app uses.

    from robot.esp32_client import ESP32Client
    esp = ESP32Client("192.168.1.50")
    if esp.connect():
        print(esp.set_led("on"))                  # -> CommandResult (ok, result, latency, ...)
        print(esp.send_simulated_movement("LEFT"))
        esp.model_started()                      # LED continuously ON
        esp.fall_detected()                      # LED blinks + buzzer pattern (only for a real or deliberate test fall)
        esp.fall_cleared()                       # back to LED ON
        esp.model_stopped()                      # LED and buzzer OFF
        print(esp.get_status())
        esp.emergency_stop()
        esp.reset_emergency(confirmed=True)
    esp.disconnect()                              # sends STOP first, then closes

Every method returns quickly and never raises for network problems: it returns a
CommandResult with ok=False and a readable `error`.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field

from .commands import AI_EVENTS, MOVEMENT_COMMANDS, NAMED_COMMANDS, CommandError
from .wifi_client import RobotClient, RobotNotConnected, RobotTimeout

LED_NAMES = {"on": "LED_ON", "off": "LED_OFF", "blink": "LED_BLINK"}
BUZZER_NAMES = {"on": "BUZZER_ON", "off": "BUZZER_OFF", "short_beep": "BUZZER_SHORT_BEEP",
                "warning": "BUZZER_WARNING"}


@dataclass
class CommandResult:
    command: str
    ok: bool
    command_id: str | None = None
    result: str | None = None        # e.g. "SIMULATED_MOVEMENT_RECEIVED: LEFT"
    error: str | None = None         # robot error code or a laptop-side problem
    latency_ms: float | None = None
    reply: dict = field(default_factory=dict)

    def __str__(self) -> str:
        if self.ok:
            return f"{self.command}: OK ({self.result}) id={self.command_id} in {self.latency_ms:.1f} ms"
        return f"{self.command}: FAILED - {self.error}"


class ESP32Client:
    def __init__(self, host: str, port: int = 3333, ack_timeout_s: float = 1.0, client: RobotClient | None = None,
                 **client_kwargs):
        """client: reuse an existing RobotClient (e.g. the one inside RobotLink) instead of a new one."""
        self.client = client or RobotClient(host, port, ack_timeout_s=ack_timeout_s, **client_kwargs)
        self.ack_timeout_s = ack_timeout_s
        self._started = client is not None

    # ------------------------------------------------------------ connection
    def connect(self, timeout: float = 6.0) -> bool:
        """Start the link and wait for the ESP32's hello and first status. False = not (yet) connected;
        the link keeps retrying in the background - see last_error()."""
        if not self._started:
            self.client.start()
            self._started = True
        self.client.set_enabled(True)
        return self.client.wait_connected(timeout) and self.client.wait_status(min(timeout, 3.0))

    def disconnect(self) -> None:
        """Safe shutdown: send STOP (clears any movement request), then close the connection."""
        stopped = self.client.connected and self.send_command("STOP").ok
        self.client.close(send_stop=not stopped)
        self._started = False

    @property
    def connected(self) -> bool:
        return self.client.connected

    def responding(self, max_age_s: float = 2.5) -> bool:
        """Connected AND a status message arrived recently (the ESP32 is alive, not just the socket)."""
        return self.client.connected and self.client.status_age_s() <= max_age_s

    def last_error(self) -> str:
        return self.client.last_error

    def hello(self) -> dict:
        return dict(self.client.hello)

    # -------------------------------------------------------------- commands
    def send_command(self, name: str, duration_ms: int | None = None) -> CommandResult:
        """Send one named command (see robot.commands.NAMED_COMMANDS) and wait for its ack."""
        name = str(name).strip().upper()
        if name not in NAMED_COMMANDS:
            return CommandResult(name, False, error=f"unknown command (valid: {', '.join(NAMED_COMMANDS)})")
        t0 = time.perf_counter()
        try:
            r = self.client.send("command", timeout=self.ack_timeout_s, command=name, duration_ms=duration_ms)
        except CommandError as exc:
            return CommandResult(name, False, error=f"not sent: {exc}")
        except RobotTimeout:
            return CommandResult(name, False, error=f"no acknowledgement within {self.ack_timeout_s:.1f} s")
        except RobotNotConnected as exc:
            return CommandResult(name, False, error=f"not connected ({exc})")
        ms = (time.perf_counter() - t0) * 1000.0
        return CommandResult(name, r.ok, r.command_id, r.data.get("result"),
                             None if r.ok else f"refused by ESP32: {r.error}" + (
                                 f" (field {r.data['field']})" if r.data.get("field") else ""), ms, r.data)

    def send_simulated_movement(self, direction: str, duration_ms: int | None = None) -> CommandResult:
        """FORWARD/BACKWARD/LEFT/RIGHT (simulated in this firmware) or STOP."""
        d = str(direction).strip().upper()
        if d not in MOVEMENT_COMMANDS + ("STOP",):
            return CommandResult(d, False, error="movement must be FORWARD, BACKWARD, LEFT, RIGHT or STOP")
        return self.send_command(d, duration_ms if d != "STOP" else None)

    def stop(self) -> CommandResult:
        return self.send_command("STOP")

    def set_led(self, state: str) -> CommandResult:
        name = LED_NAMES.get(str(state).strip().lower())
        if name is None:
            return CommandResult(f"LED {state}", False, error="LED state must be on, off or blink")
        return self.send_command(name)

    def set_buzzer(self, pattern: str) -> CommandResult:
        name = BUZZER_NAMES.get(str(pattern).strip().lower())
        if name is None:
            return CommandResult(f"BUZZER {pattern}", False, error="buzzer must be on, off, short_beep or warning")
        return self.send_command(name)

    def get_status(self, timeout: float = 1.5) -> dict | None:
        """Ask for a fresh status message. None if the ESP32 did not answer."""
        before = self.client.status_time
        r = self.send_command("STATUS")
        if not r.ok:
            return None
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.client.status_time > before:
                st = dict(self.client.robot)
                st.pop("version", None)
                st.pop("type", None)
                return st
            time.sleep(0.01)
        return None

    # ------------------------------------------------------------ AI events
    def send_event(self, name: str) -> CommandResult:
        """MODEL_STARTED, MODEL_STOPPED, FALL_DETECTED or FALL_CLEARED."""
        name = str(name).strip().upper()
        if name not in AI_EVENTS:
            return CommandResult(name, False, error=f"event must be one of {', '.join(AI_EVENTS)}")
        return self.send_command(name)

    def model_started(self) -> CommandResult:
        return self.send_event("MODEL_STARTED")

    def model_stopped(self) -> CommandResult:
        return self.send_event("MODEL_STOPPED")

    def fall_detected(self) -> CommandResult:
        return self.send_event("FALL_DETECTED")

    def fall_cleared(self) -> CommandResult:
        return self.send_event("FALL_CLEARED")

    def emergency_stop(self) -> CommandResult:
        """Highest priority. If the ack does not come, the link keeps re-sending until the ESP32 confirms."""
        r = self.send_command("EMERGENCY_STOP")
        if not r.ok and not (r.error or "").startswith("refused"):
            self.client.trigger_emergency("operator")      # retried in the background, also after reconnect
            r.error = (r.error or "") + " - emergency is queued and will be re-sent until acknowledged"
        return r

    def reset_emergency(self, confirmed: bool) -> CommandResult:
        """Operator action: only works with confirmed=True. Movement stays stopped afterwards."""
        if confirmed is not True:
            return CommandResult("RESET_EMERGENCY", False, error="the operator must confirm the reset")
        t0 = time.perf_counter()
        try:
            r = self.client.clear_emergency(timeout=self.ack_timeout_s)
        except RobotTimeout:
            return CommandResult("RESET_EMERGENCY", False, error="no acknowledgement")
        except RobotNotConnected as exc:
            return CommandResult("RESET_EMERGENCY", False, error=f"not connected ({exc})")
        return CommandResult("RESET_EMERGENCY", r.ok, r.command_id, "EMERGENCY_CLEARED" if r.ok else None,
                             None if r.ok else f"refused by ESP32: {r.error}", (time.perf_counter() - t0) * 1000,
                             r.data)
