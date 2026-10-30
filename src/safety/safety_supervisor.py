"""Central safety supervisor (laptop side).

Every motion decision on the laptop asks this one object first. It looks at all
safety conditions at once and reports the highest-priority one as the current
safety *level*:

  1 FIRMWARE_EMERGENCY  the robot itself reports an emergency (status emergency_active)
  2 EMERGENCY_STOP      the laptop latched an emergency (fall confirmed / operator e-stop)
  3 COMM_FAULT          robot not connected, status too old, or a latched communication fault
  4 OBSTACLE            the robot (sensor/report) or the camera (verified over frames) sees an obstacle
  5 FALL_RESPONSE       a possible fall is being handled (warning alert not yet resolved)
  6 TARGET_LOST         following lost or doubts its target
  7 FOLLOWING           following is active
  8 MANUAL              the operator drove the robot recently
  9 IDLE

Permissions are the AND of every active condition (not only the top one), so for
example an obstacle together with a possible fall blocks everything the fall
response blocks AND forward motion.

Latches (never cleared automatically):
  * EMERGENCY_STOP - cleared only by an operator-confirmed clear_emergency that
    the robot accepted;
  * communication fault - set when the link failed while the robot was moving or
    following, or when the robot reports a watchdog stop / comm_fault; cleared
    only when the operator sends a fresh control state (STOP) on a healthy link.
After a reconnection nothing resumes by itself.

The supervisor is pure logic (no I/O) so tests can drive it directly.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum


class Level(IntEnum):
    FIRMWARE_EMERGENCY = 1
    EMERGENCY_STOP = 2
    COMM_FAULT = 3
    OBSTACLE = 4
    FALL_RESPONSE = 5
    TARGET_LOST = 6
    FOLLOWING = 7
    MANUAL = 8
    IDLE = 9


@dataclass
class SafetyInputs:
    robot_connected: bool = False
    status_fresh: bool = False            # a status message arrived recently
    robot_emergency: bool = False         # robot status emergency_active
    laptop_emergency: bool = False        # laptop emergency latch (sent or pending)
    watchdog_ok: bool = True              # robot status watchdog_ok
    comm_fault_reported: bool = False     # robot hello comm_fault / watchdog_stop event / link lost while active
    robot_obstacle: bool = False          # robot status obstacle_detected
    camera_obstacle: bool = False         # laptop ObstacleDetector (several frames)
    fall_suspected: bool = False          # unresolved possible-fall warning
    fall_confirmed: bool = False          # unacknowledged confirmed fall
    follow_state: str = "FOLLOW_DISABLED"
    manual_active: bool = False           # manual command within the last few seconds


@dataclass
class SafetyDecision:
    level: Level
    reason: str
    allow_follow: bool
    allow_manual: bool
    allow_forward: bool
    conditions: list = field(default_factory=list)   # [(level name, reason), ...] all active ones

    def to_dict(self) -> dict:
        return {"level": int(self.level), "level_name": self.level.name, "reason": self.reason,
                "allow_follow": self.allow_follow, "allow_manual": self.allow_manual,
                "allow_forward": self.allow_forward,
                "conditions": [{"level": n, "reason": r} for n, r in self.conditions]}


UNCERTAIN_FOLLOW_STATES = ("TARGET_UNCERTAIN", "TARGET_LOST")
ACTIVE_FOLLOW_STATES = ("FOLLOW_READY", "FOLLOWING", "TARGET_UNCERTAIN", "OBSTACLE_STOP")


class SafetySupervisor:
    def __init__(self):
        self.comm_fault_latched = False
        self.comm_fault_reason = ""
        self.last: SafetyDecision | None = None

    # ------------------------------------------------------------ operator
    def acknowledge_comm_fault(self, inp: SafetyInputs) -> tuple[bool, str]:
        """Operator sent a fresh control state (STOP). Only possible on a healthy link."""
        if not (inp.robot_connected and inp.status_fresh):
            return False, "cannot clear the communication fault while the robot is not connected"
        self.comm_fault_latched = False
        self.comm_fault_reason = ""
        return True, "communication fault acknowledged"

    # ------------------------------------------------------------- evaluate
    def evaluate(self, inp: SafetyInputs) -> SafetyDecision:
        if inp.comm_fault_reported and not self.comm_fault_latched:
            self.comm_fault_latched = True
            self.comm_fault_reason = "communication failed while the robot was moving or following"
        conds: list[tuple[Level, str]] = []
        if inp.robot_emergency:
            conds.append((Level.FIRMWARE_EMERGENCY, "robot reports an emergency (latched in the firmware)"))
        if inp.laptop_emergency:
            conds.append((Level.EMERGENCY_STOP, "emergency stop latched on the laptop"))
        if not inp.robot_connected:
            conds.append((Level.COMM_FAULT, "robot not connected"))
        elif not inp.status_fresh:
            conds.append((Level.COMM_FAULT, "no status from the robot recently"))
        if self.comm_fault_latched:
            conds.append((Level.COMM_FAULT, self.comm_fault_reason + " - press STOP to acknowledge"))
        elif inp.robot_connected and not inp.watchdog_ok:
            conds.append((Level.COMM_FAULT, "robot watchdog reported a fault - press STOP to acknowledge"))
        if inp.robot_obstacle:
            conds.append((Level.OBSTACLE, "robot reports an obstacle"))
        if inp.camera_obstacle:
            conds.append((Level.OBSTACLE, "camera sees something in the path (not a distance measurement)"))
        if inp.fall_confirmed:
            conds.append((Level.FALL_RESPONSE, "confirmed fall - waiting for operator review"))
        elif inp.fall_suspected:
            conds.append((Level.FALL_RESPONSE, "possible fall - acknowledge the warning to continue"))
        if inp.follow_state in UNCERTAIN_FOLLOW_STATES:
            conds.append((Level.TARGET_LOST, f"following: {inp.follow_state}"))
        if inp.follow_state in ("FOLLOWING",):
            conds.append((Level.FOLLOWING, "following the selected person"))
        if inp.manual_active:
            conds.append((Level.MANUAL, "manual control"))
        conds.sort(key=lambda c: c[0])
        levels = {c[0] for c in conds}
        hard_stop = any(lv <= Level.COMM_FAULT for lv in levels)
        allow_follow = not hard_stop and Level.OBSTACLE not in levels and Level.FALL_RESPONSE not in levels
        allow_manual = (not hard_stop and Level.FALL_RESPONSE not in levels
                        and inp.follow_state not in ACTIVE_FOLLOW_STATES)
        allow_forward = Level.OBSTACLE not in levels
        top = conds[0] if conds else (Level.IDLE, "idle")
        d = SafetyDecision(top[0], top[1], allow_follow, allow_manual, allow_forward and not hard_stop,
                           [(lv.name, r) for lv, r in conds])
        self.last = d
        return d

    def check_motion(self, inp: SafetyInputs, kind: str, left: int, right: int) -> str:
        """'' if this motion may be sent, else the reason it is refused. kind: manual | follow."""
        d = self.evaluate(inp)
        moving = left != 0 or right != 0
        if not moving:
            return ""                      # a zero-speed command is always allowed (it stops)
        if kind == "follow":
            if not d.allow_follow:
                return f"following blocked: {d.reason}"
            return ""
        if not d.allow_manual:
            if inp.follow_state in ACTIVE_FOLLOW_STATES and d.level > Level.FALL_RESPONSE:
                return "manual control is disabled while following - disable following first"
            return f"manual motion blocked: {d.reason}"
        if left + right > 0 and not d.allow_forward:
            return f"forward motion blocked: {d.reason} (reverse or turning is allowed)"
        return ""
