"""SentinelAI robot protocol v3 (JSON lines) - laptop side: build, validate, encode, parse.

Specification: docs/PROTOCOL.md. The robot side is firmware/sentinel_robot/robot_core.cpp
(and its Python mirror robot/sim_robot.py). Every rule here matches the firmware, so an
invalid command is refused on the laptop before it is ever sent - and the robot checks
everything again.
"""
from __future__ import annotations

import json
import re
from dataclasses import dataclass, field

PROTOCOL_VERSION = 3
MAX_LINE = 200
MAX_SEQ = 999_999_999
MIN_MOVE_MS, MAX_MOVE_MS = 50, 2000
ID_RE = re.compile(r"^[A-Za-z0-9_.\-]{1,24}$")

LED_STATES = ("on", "off", "blink")
BUZZER_PATTERNS = ("off", "short_beep", "warning", "emergency", "on")
# Named commands (message type "command") - a readable alias layer used by the test console and
# later by the dashboard. FORWARD/BACKWARD/LEFT/RIGHT are SIMULATED in this firmware version.
MOVEMENT_COMMANDS = ("FORWARD", "BACKWARD", "LEFT", "RIGHT")
# AI state events sent by the laptop app when its state changes (never once per video frame)
AI_EVENTS = ("MODEL_STARTED", "MODEL_STOPPED", "FALL_DETECTED", "FALL_CLEARED")
NAMED_COMMANDS = MOVEMENT_COMMANDS + ("STOP", "LED_ON", "LED_OFF", "LED_BLINK", "BUZZER_ON", "BUZZER_OFF",
                                      "BUZZER_SHORT_BEEP", "BUZZER_WARNING", "STATUS",
                                      "EMERGENCY_STOP") + AI_EVENTS

# type -> ordered (name, kind, required) field specs. kind: see _check()
SCHEMA: dict[str, list[tuple[str, str, bool]]] = {
    "ping": [],
    "status_request": [],
    "motor": [("mode", "manual", True), ("left", "wheel", True), ("right", "wheel", True),
              ("duration_ms", "duration", True)],
    "stop": [],
    "follow_mode": [("enabled", "bool", True), ("target_id", "target", False)],
    "follow": [("target_id", "target", True), ("left", "wheel", True), ("right", "wheel", True),
               ("duration_ms", "duration", True), ("confidence", "unit", True)],
    "led": [("state", "led", True), ("interval_ms", "interval", False)],
    "buzzer": [("pattern", "buzzer", True)],
    "mute": [],
    "emergency_stop": [("reason", "id", True)],
    "clear_emergency": [("operator_confirmed", "bool", True)],
    "obstacle": [("source", "camera", True), ("distance_cm", "distance", True)],
    "obstacle_clear": [],
    "command": [("command", "named", True), ("duration_ms", "duration", False)],
}
MOTION_TYPES = ("motor", "follow")
ERROR_CODES = {
    "BAD_JSON": "the line is not a valid (flat) JSON object",
    "TOO_LONG": "line longer than 200 characters",
    "BAD_VERSION": "protocol version is not 3",
    "MISSING_FIELD": "a required field is missing",
    "BAD_FIELD": "a field has the wrong type or is out of range",
    "UNKNOWN_FIELD": "the message contains a field the type does not allow",
    "UNKNOWN_TYPE": "message type not recognised",
    "BAD_SEQ": "sequence number not larger than the previous one (duplicate/stale)",
    "EMERGENCY_ACTIVE": "refused while the emergency is latched",
    "OBSTACLE_AHEAD": "motion refused while an obstacle is reported",
    "FOLLOW_ACTIVE": "manual motor commands are refused while following",
    "FOLLOW_NOT_ENABLED": "follow drive requests need follow mode",
    "TARGET_MISMATCH": "follow request for another target than the one being followed",
    "LOW_CONFIDENCE": "target confidence below the robot's minimum (motors stopped)",
    "NEEDS_CONFIRM": "clear_emergency needs operator_confirmed: true",
    "UNKNOWN_COMMAND": "the named command is not one of the supported commands",
    "FALL_ALERT_ACTIVE": "movement is refused while a fall alert is active (send FALL_CLEARED first)",
}


class CommandError(ValueError):
    """The laptop refused to build an invalid command (it is never sent)."""


class ProtocolError(ValueError):
    """A line from the robot did not follow the protocol."""


def _check(msg_type: str, name: str, kind: str, v):
    def fail(why: str):
        raise CommandError(f"{msg_type}.{name}: {why} (got {v!r})")

    def whole(lo: int, hi: int) -> int:
        if isinstance(v, bool) or not isinstance(v, int):
            fail("must be a whole number")
        if not lo <= v <= hi:
            fail(f"must be {lo}..{hi}")
        return v

    if kind == "wheel":
        return whole(-100, 100)
    if kind == "duration":
        return whole(MIN_MOVE_MS, MAX_MOVE_MS)
    if kind == "target":
        return whole(0, 65535)
    if kind == "interval":
        return whole(100, 2000)
    if kind == "distance":
        return whole(0, 999)
    if kind == "bool":
        if not isinstance(v, bool):
            fail("must be true or false")
        return v
    if kind == "unit":
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not 0 <= v <= 1:
            fail("must be a number 0..1")
        return round(float(v), 6)
    if kind == "id":
        if not isinstance(v, str) or not ID_RE.match(v):
            fail("must be 1-24 characters A-Z a-z 0-9 _ - .")
        return v
    choices = {"manual": ("manual",), "camera": ("camera",), "led": LED_STATES, "buzzer": BUZZER_PATTERNS,
               "named": NAMED_COMMANDS}[kind]
    if v not in choices:
        fail(f"must be one of {choices}")
    return v


@dataclass(frozen=True)
class Command:
    type: str
    fields: tuple = ()          # ((name, value), ...) in schema order

    def get(self, name: str, default=None):
        return dict(self.fields).get(name, default)

    def __str__(self) -> str:
        return self.type + "(" + ", ".join(f"{k}={v}" for k, v in self.fields) + ")"


def make_command(msg_type: str, **values) -> Command:
    """Validate and build a command. Raises CommandError for anything the robot would refuse."""
    msg_type = str(msg_type).strip().lower()
    if msg_type not in SCHEMA:
        raise CommandError(f"unknown message type '{msg_type}'. Valid: {', '.join(SCHEMA)}")
    spec = SCHEMA[msg_type]
    allowed = {n for n, _, _ in spec}
    extra = set(values) - allowed
    if extra:
        raise CommandError(f"{msg_type}: unknown field(s) {sorted(extra)}")
    out = []
    for name, kind, required in spec:
        if name not in values or values[name] is None:
            if required:
                raise CommandError(f"{msg_type}: missing field '{name}'")
            continue
        out.append((name, _check(msg_type, name, kind, values[name])))
    cmd = Command(msg_type, tuple(out))
    if msg_type == "follow_mode" and cmd.get("enabled") and cmd.get("target_id") is None:
        raise CommandError("follow_mode: target_id is required when enabled is true")
    if msg_type == "clear_emergency" and cmd.get("operator_confirmed") is not True:
        raise CommandError("clear_emergency: operator_confirmed must be true")
    if msg_type == "command" and cmd.get("duration_ms") is not None and cmd.get("command") not in MOVEMENT_COMMANDS:
        raise CommandError("command: duration_ms is only allowed for FORWARD/BACKWARD/LEFT/RIGHT")
    return cmd


def encode(seq: int, command_id: str, cmd: Command) -> bytes:
    if not 1 <= seq <= MAX_SEQ:
        raise CommandError(f"sequence number {seq} out of range")
    if not ID_RE.match(command_id):
        raise CommandError(f"invalid command_id {command_id!r}")
    msg = {"version": PROTOCOL_VERSION, "type": cmd.type, "command_id": command_id, "seq": seq}
    msg.update(cmd.fields)
    line = json.dumps(msg, separators=(",", ":"))
    if len(line) > MAX_LINE:
        raise CommandError(f"message too long ({len(line)} > {MAX_LINE})")
    return (line + "\n").encode("ascii")


@dataclass
class Reply:
    kind: str                 # "ack", "status" or "event"
    data: dict = field(default_factory=dict)
    raw: str = ""

    @property
    def ok(self) -> bool:
        return self.kind == "ack" and self.data.get("ok") is True

    @property
    def error(self) -> str | None:
        return self.data.get("error")

    @property
    def seq(self) -> int:
        return int(self.data.get("seq") or 0)

    @property
    def command_id(self) -> str | None:
        return self.data.get("command_id")

    @property
    def event(self) -> str | None:
        return self.data.get("event")


def parse_line(line: str) -> Reply:
    """Parse one line received from the robot."""
    text = line.strip()
    try:
        data = json.loads(text)
    except json.JSONDecodeError as exc:
        raise ProtocolError(f"robot sent invalid JSON: {text[:80]!r} ({exc.msg})") from None
    if not isinstance(data, dict) or data.get("type") not in ("ack", "status", "event"):
        raise ProtocolError(f"unexpected message from robot: {text[:80]!r}")
    if data.get("version") != PROTOCOL_VERSION:
        raise ProtocolError(f"robot speaks protocol {data.get('version')}, laptop {PROTOCOL_VERSION}")
    return Reply(data["type"], data, text)
