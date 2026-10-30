"""Person-following state machine (laptop side, pure logic).

States
  FOLLOW_DISABLED   following is off (start state; after a stop, fall response or link loss)
  FOLLOW_READY      the operator selected a person; the robot is in follow mode but holds still.
                    Moves to FOLLOWING when the target is clearly visible and following is
                    "armed" (armed = the operator pressed Follow / Resume).
  FOLLOWING         drive commands are sent toward the selected person
  TARGET_UNCERTAIN  the target was not seen in the last frames or its detection is weak:
                    robot stops; back to FOLLOWING automatically when the SAME person is
                    clearly seen again
  TARGET_LOST       not seen for ``lost_grace_s`` (or uncertain too long): robot stops and
                    leaves follow mode; the operator must select a person again
  OBSTACLE_STOP     an obstacle is reported: robot stops. When the path has been clear for
                    ``obstacle_clear_s`` the state becomes FOLLOW_READY and the operator decides
                    whether to resume (``obstacle_auto_resume`` true = resume by itself)
  EMERGENCY_STOP    an emergency or communication fault: stopped; after it is cleared the
                    state becomes FOLLOW_DISABLED (select the person again)

Rules that never change:
  * the target is chosen by the operator only - the machine never switches to
    another person (a different track id is never followed);
  * a possible/confirmed fall of the target, or a fall response, ends following;
  * only FOLLOWING allows motion.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class FollowState(str, Enum):
    FOLLOW_DISABLED = "FOLLOW_DISABLED"
    FOLLOW_READY = "FOLLOW_READY"
    FOLLOWING = "FOLLOWING"
    TARGET_UNCERTAIN = "TARGET_UNCERTAIN"
    TARGET_LOST = "TARGET_LOST"
    OBSTACLE_STOP = "OBSTACLE_STOP"
    EMERGENCY_STOP = "EMERGENCY_STOP"


# states in which the robot should be in follow mode (robot side follow_mode true)
ROBOT_FOLLOW_MODE_STATES = (FollowState.FOLLOW_READY, FollowState.FOLLOWING, FollowState.TARGET_UNCERTAIN,
                            FollowState.OBSTACLE_STOP)


@dataclass
class FollowInputs:
    target_visible: bool = False      # the selected track id is in this frame
    target_confident: bool = False    # visible, confirmed track, good score and keypoints
    target_down: bool = False         # target's fall state is not NORMAL or posture LYING
    obstacle: bool = False            # robot or camera obstacle
    emergency: bool = False           # safety level FIRMWARE_EMERGENCY / EMERGENCY_STOP
    comm_ok: bool = True              # robot connected, status fresh, no latched comm fault
    fall_response: bool = False       # any unresolved fall warning/emergency


class FollowStateMachine:
    def __init__(self, lost_grace_s: float = 0.7, uncertain_max_s: float = 3.0, obstacle_clear_s: float = 1.0,
                 obstacle_auto_resume: bool = False):
        self.lost_grace_s = lost_grace_s
        self.uncertain_max_s = uncertain_max_s
        self.obstacle_clear_s = obstacle_clear_s
        self.obstacle_auto_resume = obstacle_auto_resume
        self.state = FollowState.FOLLOW_DISABLED
        self.target_id: int | None = None
        self.armed = False
        self.reason = "following is off"
        self.since = 0.0
        self._last_seen = 0.0
        self._uncertain_since = 0.0
        self._clear_since: float | None = None
        self.transitions: list[tuple[float, str, str, str]] = []   # (t, from, to, reason), last 50
        self.transition_count = 0                                  # total, for "what is new" checks

    # ------------------------------------------------------------- operator
    def select_target(self, track_id: int, now: float, start: bool = True) -> tuple[bool, str]:
        if self.state == FollowState.EMERGENCY_STOP:
            return False, "emergency active - clear it first"
        self.target_id = int(track_id)
        self.armed = bool(start)
        self._last_seen = now
        self._go(FollowState.FOLLOW_READY, now, f"operator selected person #{track_id}")
        return True, self.reason

    def resume(self, now: float) -> tuple[bool, str]:
        if self.state != FollowState.FOLLOW_READY or self.target_id is None:
            return False, f"cannot resume in state {self.state.value}"
        self.armed = True
        self._last_seen = now
        self.reason = "operator resumed following"
        return True, self.reason

    def disable(self, now: float, reason: str = "operator stopped following") -> None:
        self.target_id = None
        self.armed = False
        self._go(FollowState.FOLLOW_DISABLED, now, reason)

    # --------------------------------------------------------------- update
    def step(self, now: float, inp: FollowInputs) -> FollowState:
        s = self.state
        if s == FollowState.FOLLOW_DISABLED:
            return s
        if s == FollowState.EMERGENCY_STOP:
            if not inp.emergency:
                self.target_id, self.armed = None, False
                self._go(FollowState.FOLLOW_DISABLED, now, "emergency cleared - select the person again")
            return self.state
        if inp.emergency:
            self.armed = False
            self._go(FollowState.EMERGENCY_STOP, now, "emergency stop")
            return self.state
        if not inp.comm_ok:
            self.disable(now, "communication problem - following stopped (robot leaves follow mode)")
            return self.state
        if inp.fall_response or (inp.target_visible and inp.target_down):
            self.disable(now, "fall response - following stopped")
            return self.state
        if s == FollowState.TARGET_LOST:
            return s
        if inp.target_visible:
            self._last_seen = now
        if inp.obstacle:
            self._clear_since = None
            if s != FollowState.OBSTACLE_STOP:
                self._go(FollowState.OBSTACLE_STOP, now, "obstacle - stopped")
            return self.state
        if s == FollowState.OBSTACLE_STOP:
            if self._clear_since is None:
                self._clear_since = now
            if now - self._clear_since < self.obstacle_clear_s:
                return s
            self.armed = self.obstacle_auto_resume
            self._go(FollowState.FOLLOW_READY, now, "path clear - " + (
                "resuming" if self.armed else "press Resume to continue following"))
            s = self.state
        unseen = now - self._last_seen
        if s == FollowState.FOLLOW_READY:
            if unseen > self.lost_grace_s and not inp.target_visible:
                self._lost(now)
            elif self.armed and inp.target_confident:
                self._go(FollowState.FOLLOWING, now, f"following person #{self.target_id}")
            return self.state
        if s == FollowState.FOLLOWING:
            if not inp.target_confident:
                self._uncertain_since = now
                self._go(FollowState.TARGET_UNCERTAIN, now,
                         "target not visible" if not inp.target_visible else "target detection uncertain")
            return self.state
        if s == FollowState.TARGET_UNCERTAIN:
            if inp.target_confident:
                self._go(FollowState.FOLLOWING, now, f"person #{self.target_id} clearly visible again")
            elif not inp.target_visible and unseen > self.lost_grace_s:
                self._lost(now)
            elif now - self._uncertain_since > self.uncertain_max_s:
                self._lost(now, f"target uncertain for more than {self.uncertain_max_s:.0f} s")
        return self.state

    @property
    def motion_allowed(self) -> bool:
        return self.state == FollowState.FOLLOWING

    @property
    def robot_follow_mode(self) -> bool:
        return self.state in ROBOT_FOLLOW_MODE_STATES

    def to_dict(self) -> dict:
        return {"state": self.state.value, "target_id": self.target_id, "armed": self.armed, "reason": self.reason}

    # ------------------------------------------------------------ internals
    def _lost(self, now: float, why: str | None = None) -> None:
        self.armed = False
        self._go(FollowState.TARGET_LOST, now,
                 (why or f"person #{self.target_id} not seen for {self.lost_grace_s:.1f} s")
                 + " - select the person again")

    def _go(self, new: FollowState, now: float, reason: str) -> None:
        if new != self.state:
            self.transitions.append((now, self.state.value, new.value, reason))
            self.transitions = self.transitions[-50:]
            self.transition_count += 1
            self.since = now
        self.state = new
        self.reason = reason
        if new != FollowState.OBSTACLE_STOP:
            self._clear_since = None
