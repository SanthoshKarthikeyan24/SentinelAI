"""Person-following and camera-based obstacle detection (laptop side).

Assumption: the phone camera is mounted on the robot, looking forward, so the
image centre is the robot's heading.

* The OPERATOR selects the person to follow (click on them in the dashboard).
  The robot never picks a person by itself and never switches to another one.
* FollowStateMachine (src/safety/robot_state_machine.py) decides WHETHER the
  robot may move; FollowController decides HOW (one differential-drive
  command per frame); RampLimiter bounds the acceleration; RobotLink asks the
  SafetySupervisor before anything is queued.

  situation                              -> action
  ---------------------------------------------------------------------
  person left / right of centre          -> turn toward them (spin in place if far off-centre)
  person near centre and far away        -> drive forward (speed grows with the gap)
  person box taller than too_close_ratio -> no forward motion
  person not seen / weak detection       -> TARGET_UNCERTAIN: stop
  not seen for lost_grace_s              -> TARGET_LOST: stop, leave follow mode
  obstacle (camera or robot)             -> OBSTACLE_STOP; operator resumes
  target falls / fall warning            -> following ends (fall response)
  emergency / link lost                  -> EMERGENCY_STOP / disabled (robot stops by itself too)

The bounding-box height is only a rough cue for "nearer / farther" (it depends
on the person's height, pose and camera angle) - it is NEVER a distance
measurement and the system never claims a precise following distance.
The robot never reverses while following.

ObstacleDetector flags objects (chairs, tables, bags, other people, ...) whose
box reaches the lower part of the image inside a central "driving corridor".
This is a heuristic, not a reliable obstacle sensor: it cannot measure
distance, misses low/thin/transparent objects and objects outside the
detector's 80 classes. An optional ultrasonic sensor on the robot (firmware
OBSTACLE_SENSOR_ENABLED) is reported separately as source "sensor".
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field

from src.safety.robot_state_machine import FollowInputs, FollowState, FollowStateMachine

log = logging.getLogger("sentinel.follow")

DEFAULT_OBSTACLE_CLASSES = ["chair", "couch", "bed", "dining table", "toilet", "tv", "laptop", "bottle", "cup",
                            "backpack", "handbag", "suitcase", "potted plant", "bench", "dog", "cat"]


@dataclass
class ObstacleParams:
    enabled: bool = True
    corridor_width: float = 0.40      # fraction of the image width centred on the heading
    bottom_frac: float = 0.80         # object box must reach below this fraction of the image height
    min_area_frac: float = 0.03       # ignore tiny boxes (fraction of the image area)
    include_people: bool = True       # other people (not the target) count as obstacles
    clear_frames: int = 5             # frames without an obstacle before CLEAR is sent
    report_interval_s: float = 0.5    # repeat OBSTACLE while seen (robot holds for 1.5 s)
    object_every_n_frames: int = 2    # object detector rate while following (normally 10)
    classes: list = field(default_factory=lambda: list(DEFAULT_OBSTACLE_CLASSES))


@dataclass
class FollowParams:
    command_ms: int = 400             # each follow command expires on the robot after this (renewed every frame)
    max_speed_pct: int = 35
    min_speed_pct: int = 18           # below this the motors usually stall (tune in Phase 3)
    turn_gain: float = 40.0           # turn speed % per unit of horizontal error (-1..1)
    forward_gain: float = 60.0        # forward speed % per unit of relative distance error
    deadband: float = 0.12            # |error| below this = centred, no turning
    spin_threshold: float = 0.45      # |error| above this = turn in place, no forward
    target_height_ratio: float = 0.55 # desired person box height / image height ("follow distance")
    too_close_ratio: float = 0.80     # box taller than this = too close, no forward
    lost_grace_s: float = 0.7         # target unseen this long -> TARGET_LOST
    uncertain_max_s: float = 3.0      # target uncertain this long -> TARGET_LOST
    min_detection_score: float = 0.5  # weaker detections = TARGET_UNCERTAIN (robot needs >= 0.5 as well)
    min_keypoint_reliability: float = 0.25
    max_accel_pct_per_s: float = 60.0 # speed may grow by at most this much per second (slowing is instant)
    obstacle_clear_s: float = 1.0     # path clear this long before following may resume
    obstacle_auto_resume: bool = False  # false = the operator presses Resume after an obstacle
    obstacle: ObstacleParams = field(default_factory=ObstacleParams)

    @classmethod
    def from_config(cls, d: dict | None) -> "FollowParams":
        d = dict(d or {})
        obs = ObstacleParams(**{k: v for k, v in dict(d.pop("obstacle", {}) or {}).items()
                                if k in ObstacleParams.__dataclass_fields__})
        d.pop("enabled", None)
        p = cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__}, obstacle=obs)
        problems = p.problems()
        if problems:
            raise ValueError("robot.follow: " + "; ".join(problems))
        return p

    def problems(self) -> list[str]:
        out = []
        if not 0 < self.max_speed_pct <= 100:
            out.append("max_speed_pct must be 1..100")
        if not 0 <= self.min_speed_pct <= self.max_speed_pct:
            out.append("min_speed_pct must be between 0 and max_speed_pct")
        if not 50 <= self.command_ms <= 2000:
            out.append("command_ms must be 50..2000 (firmware limits)")
        if not 0 <= self.deadband < self.spin_threshold <= 1:
            out.append("need 0 <= deadband < spin_threshold <= 1")
        if self.min_detection_score < 0.5:
            out.append("min_detection_score must be >= 0.5 (the robot refuses weaker targets)")
        if self.max_accel_pct_per_s <= 0:
            out.append("max_accel_pct_per_s must be > 0")
        if not 0 < self.target_height_ratio < self.too_close_ratio <= 1:
            out.append("need 0 < target_height_ratio < too_close_ratio <= 1")
        o = self.obstacle
        if not 0 < o.corridor_width <= 1 or not 0 < o.bottom_frac <= 1 or not 0 <= o.min_area_frac < 1:
            out.append("obstacle corridor_width/bottom_frac/min_area_frac out of range")
        if o.clear_frames < 1 or o.object_every_n_frames < 1:
            out.append("obstacle clear_frames and object_every_n_frames must be >= 1")
        return out


@dataclass
class FollowDecision:
    left: int
    right: int
    reason: str
    target_id: int | None
    error_x: float | None = None
    height_ratio: float | None = None


@dataclass
class ObstacleState:
    seen: bool
    send_report: bool          # time to (re)send OBSTACLE
    send_clear: bool           # time to send CLEAR
    reason: str = ""


def _box_wh(box):
    return float(box[2] - box[0]), float(box[3] - box[1])


def _inside_fraction(inner, outer) -> float:
    """Fraction of box `inner` that lies inside box `outer`."""
    iw = max(0.0, min(inner[2], outer[2]) - max(inner[0], outer[0]))
    ih = max(0.0, min(inner[3], outer[3]) - max(inner[1], outer[1]))
    w, h = _box_wh(inner)
    return iw * ih / max(w * h, 1e-6)


class ObstacleDetector:
    def __init__(self, p: ObstacleParams):
        self.p = p
        self.active = False
        self._clear_count = 0
        self._last_report = -1e9
        self.last_reason = ""

    def reset(self) -> None:
        self.active = False
        self._clear_count = 0
        self._last_report = -1e9

    def blocking(self, w: int, h: int, objects, persons, target_id) -> str:
        """Return a description of the first object blocking the path, or ''."""
        if not self.p.enabled or w <= 0 or h <= 0:
            return ""
        c0 = w * (0.5 - self.p.corridor_width / 2)
        c1 = w * (0.5 + self.p.corridor_width / 2)
        target_box = next((p.box for p in persons if p.track_id == target_id), None)
        candidates = [(o.class_name, o.box) for o in objects if o.class_name in self.p.classes
                      and not (target_box is not None and _inside_fraction(o.box, target_box) > 0.6)]
        # (an object box mostly inside the followed person's box is that person mis-labelled,
        #  or something they hold/sit on - not an obstacle between robot and person)
        if self.p.include_people:
            candidates += [("person", p.box) for p in persons if p.track_id != target_id]
        for name, box in candidates:
            bw, bh = _box_wh(box)
            overlap = max(0.0, min(box[2], c1) - max(box[0], c0))
            if (overlap >= 0.3 * min(bw, c1 - c0) and box[3] >= self.p.bottom_frac * h
                    and bw * bh >= self.p.min_area_frac * w * h):
                return f"{name} in the path"
        return ""

    def update(self, now: float, w: int, h: int, objects, persons, target_id) -> ObstacleState:
        reason = self.blocking(w, h, objects, persons, target_id)
        if reason:
            self._clear_count = 0
            self.last_reason = reason
            newly = not self.active
            self.active = True
            report = newly or now - self._last_report >= self.p.report_interval_s
            if report:
                self._last_report = now
            return ObstacleState(True, report, False, reason)
        if self.active:
            self._clear_count += 1
            if self._clear_count >= self.p.clear_frames:
                self.active = False
                return ObstacleState(False, False, True, "path clear")
            return ObstacleState(True, False, False, self.last_reason + " (confirming clear)")
        return ObstacleState(False, False, False, "")


def target_quality(person, p: FollowParams) -> tuple[bool, str]:
    """Is this detection of the selected person good enough to drive toward?"""
    if not getattr(person, "confirmed", True):
        return False, "track not confirmed yet"
    if person.score < p.min_detection_score:
        return False, f"detection score {person.score:.2f} < {p.min_detection_score:.2f}"
    if getattr(person, "reliability", 1.0) < p.min_keypoint_reliability:
        return False, "too few visible body keypoints"
    return True, "clearly visible"


def target_down(person) -> bool:
    return person.fall_state != "NORMAL" or getattr(person, "posture", "") == "LYING"


class RampLimiter:
    """Bounded acceleration: each wheel's speed may grow by at most max_accel per second.
    Slowing down and stopping are never delayed; a change of direction passes through 0."""

    def __init__(self, max_accel_pct_per_s: float):
        self.a = float(max_accel_pct_per_s)
        self.left = 0.0
        self.right = 0.0
        self._t: float | None = None

    def reset(self) -> None:
        self.left = self.right = 0.0
        self._t = None

    def _one(self, cur: float, want: float, step: float) -> float:
        if want == 0 or (cur != 0 and (want > 0) != (cur > 0)):
            return 0.0                                  # stop / reverse: through zero, immediately
        if abs(want) <= abs(cur):
            return want                                 # slowing down: immediately
        return cur + max(-step, min(step, want - cur))

    def apply(self, now: float, left: int, right: int) -> tuple[int, int]:
        dt = 0.1 if self._t is None else max(0.0, min(0.5, now - self._t))
        self._t = now
        step = self.a * dt
        self.left = self._one(self.left, float(left), step)
        self.right = self._one(self.right, float(right), step)
        return int(round(self.left)), int(round(self.right))


class FollowController:
    """Pure geometry: one differential-drive command toward the given (selected) person."""

    def __init__(self, p: FollowParams):
        self.p = p

    def compute(self, w: int, h: int, target, obstacle: bool = False) -> FollowDecision:
        x1, y1, x2, y2 = (float(v) for v in target.box)
        err = ((x1 + x2) / 2 - w / 2) / (w / 2)
        hr = (y2 - y1) / h
        p = self.p
        # forward: only when not too close; never reverse
        if hr >= p.too_close_ratio:
            fwd, why = 0.0, "too close"
        else:
            gap = (p.target_height_ratio - hr) / p.target_height_ratio
            fwd = p.forward_gain * gap if gap > 0.05 else 0.0
            why = "approaching" if fwd > 0 else "near the follow distance (box-size estimate)"
        if abs(err) < p.deadband:
            turn = 0.0
        else:
            turn = p.turn_gain * err
            if abs(err) > p.spin_threshold:
                fwd, why = 0.0, "turning toward the person"
        if obstacle and fwd > 0:
            fwd, why = 0.0, "obstacle"
        if 0 < fwd < p.min_speed_pct:
            fwd = float(p.min_speed_pct)
        if fwd == 0 and 0 < abs(turn) < p.min_speed_pct:
            turn = p.min_speed_pct if turn > 0 else -p.min_speed_pct
        fwd = min(fwd, p.max_speed_pct)
        left = int(round(max(-p.max_speed_pct, min(p.max_speed_pct, fwd + turn))))
        right = int(round(max(-p.max_speed_pct, min(p.max_speed_pct, fwd - turn))))
        side = "centred" if turn == 0 else ("person on the right" if err > 0 else "person on the left")
        return FollowDecision(left, right, f"{why}, {side}", target.track_id, round(err, 3), round(hr, 3))


class FollowManager:
    """Glue between the AI pipeline, the follow state machine and RobotLink.
    Call on_frame() after every processed frame."""

    def __init__(self, link, params: FollowParams, pipeline=None, event_logger=None):
        self.link = link
        self.p = params
        self.ctrl = FollowController(params)
        self.ramp = RampLimiter(params.max_accel_pct_per_s)
        self.obstacles = ObstacleDetector(params.obstacle)
        self.sm = FollowStateMachine(params.lost_grace_s, params.uncertain_max_s, params.obstacle_clear_s,
                                     params.obstacle_auto_resume)
        self.pipeline = pipeline
        self.logger = event_logger
        self.last: FollowDecision | None = None
        self.last_obstacle = ObstacleState(False, False, False)
        self.commands_sent = 0
        self.refused = 0
        self._robot_mode: tuple[bool, int | None] = (False, None)   # what we last asked the robot
        self._was_moving = False
        self._saved_object_rate = None
        self._n_transitions = 0
        link.follow = self

    # ------------------------------------------------------------- operator
    @property
    def active(self) -> bool:
        return self.sm.state != FollowState.FOLLOW_DISABLED

    def select(self, track_id: int, persons=None) -> tuple[bool, str]:
        """Operator clicked a person. Starts following that person only."""
        d = self.link.safety()
        if not d.allow_follow:
            return False, f"cannot follow now: {d.reason}"
        if persons is not None:
            q = next((x for x in persons if x.track_id == track_id), None)
            if q is None:
                return False, f"person #{track_id} is not visible right now"
            if target_down(q):
                return False, f"person #{track_id} is lying / in a fall state - not followed"
        ok, msg = self.sm.select_target(track_id, time.monotonic(), start=True)
        if not ok:
            return False, msg
        self.ramp.reset()
        self.obstacles.reset()
        self._sync_robot_mode(wait=True)
        if self.sm.state == FollowState.FOLLOW_DISABLED:
            return False, self.sm.reason
        if self.pipeline is not None and self._saved_object_rate is None:   # more obstacle checks while active
            self._saved_object_rate = self.pipeline.object_every_n
            self.pipeline.object_every_n = self.p.obstacle.object_every_n_frames
        self._log_transitions()
        return True, f"following person #{track_id}"

    def resume(self) -> tuple[bool, str]:
        d = self.link.safety()
        if not d.allow_follow:
            return False, f"cannot resume: {d.reason}"
        ok, msg = self.sm.resume(time.monotonic())
        return ok, msg

    def disable(self, reason: str = "operator", send: bool = True) -> None:
        was = self.active
        self.sm.disable(time.monotonic(), reason)
        self.ramp.reset()
        self._was_moving = False
        if self.pipeline is not None and self._saved_object_rate is not None:
            self.pipeline.object_every_n = self._saved_object_rate
            self._saved_object_rate = None
        if send:
            self._sync_robot_mode()
        else:
            self._robot_mode = (False, None)     # caller sends stop / emergency, which ends follow mode
        if self.last_obstacle.seen:
            self.link.clear_obstacle()
            self.last_obstacle = ObstacleState(False, False, False)
        if was:
            log.info("Following stopped: %s", reason)
        self._log_transitions()

    # stop() kept for older callers
    def stop(self, reason: str = "operator") -> None:
        self.disable(reason)

    # ---------------------------------------------------------------- frame
    def on_frame(self, result, frame_size, frame_age_s: float = 0.0) -> None:
        now = time.monotonic()
        w, h = frame_size
        persons = result.persons
        target = next((q for q in persons if q.track_id == self.sm.target_id), None) \
            if self.sm.target_id is not None else None
        # camera obstacle check while following or shortly after manual driving
        watch = self.active or self.link.manual_active()
        if watch:
            obstacle = self.obstacles.update(now, w, h, result.objects, persons, self.sm.target_id)
            if obstacle.send_report:
                self.link.report_obstacle(0)
            elif obstacle.send_clear:
                self.link.clear_obstacle()
            if obstacle.seen != self.last_obstacle.seen and self.logger is not None:
                self._log("FOLLOW_OBSTACLE" if obstacle.seen else "FOLLOW_OBSTACLE_CLEAR",
                          obstacle.reason, uncertainty="camera heuristic, not a distance measurement")
            self.last_obstacle = obstacle
        elif self.last_obstacle.seen:
            self.link.clear_obstacle()
            self.obstacles.reset()
            self.last_obstacle = ObstacleState(False, False, False)
        if not self.active:
            return
        safety = self.link.safety()
        confident, why_unsure = target_quality(target, self.p) if target is not None else (False, "not visible")
        inp = FollowInputs(
            target_visible=target is not None, target_confident=confident,
            target_down=target is not None and target_down(target),
            obstacle=self.last_obstacle.seen or self.link.client.robot_obstacle,
            emergency=safety.level <= 2, comm_ok=safety.level != 3 and not self.link.supervisor.comm_fault_latched,
            fall_response=self.link.fall_suspected or self.link.fall_confirmed)
        state = self.sm.step(now, inp)
        self._log_transitions()
        if state in (FollowState.FOLLOW_DISABLED, FollowState.TARGET_LOST, FollowState.EMERGENCY_STOP):
            self.ramp.reset()
            self._was_moving = False
            if state != FollowState.EMERGENCY_STOP:
                self._sync_robot_mode()
            if state == FollowState.FOLLOW_DISABLED and self.pipeline is not None \
                    and self._saved_object_rate is not None:
                self.pipeline.object_every_n = self._saved_object_rate
                self._saved_object_rate = None
            self.last = FollowDecision(0, 0, self.sm.reason, self.sm.target_id)
            return
        self._sync_robot_mode()
        conf = float(target.score) if target is not None else 0.0
        created = now - max(0.0, frame_age_s)
        if state == FollowState.FOLLOWING:
            d = self.ctrl.compute(w, h, target, obstacle=inp.obstacle)
            left, right = self.ramp.apply(now, d.left, d.right)
            d = FollowDecision(left, right, d.reason, d.target_id, d.error_x, d.height_ratio)
        else:
            self.ramp.reset()
            reason = self.sm.reason if state != FollowState.TARGET_UNCERTAIN else f"holding: {why_unsure}"
            d = FollowDecision(0, 0, reason, self.sm.target_id)
        moving = d.left != 0 or d.right != 0
        if moving or self._was_moving:
            md = self.link.follow_drive(self.sm.target_id, d.left, d.right, self.p.command_ms, conf, created)
            if md.sent:
                self.commands_sent += 1
            elif moving:
                self.refused += 1
                d = FollowDecision(0, 0, f"{d.reason} | not sent: {md.reason}", d.target_id, d.error_x,
                                   d.height_ratio)
                moving = False
        self._was_moving = moving
        self.last = d

    # ------------------------------------------------------------- internals
    def _sync_robot_mode(self, wait: bool = False) -> None:
        want = (self.sm.robot_follow_mode, self.sm.target_id if self.sm.robot_follow_mode else None)
        if want == self._robot_mode:
            return
        self._robot_mode = want
        t = self.link.follow_mode(want[0], want[1])
        if wait:
            reply = t.wait(self.link.operator_timeout_s)
            if reply is None or not reply.ok:
                why = t.detail or (reply.error if reply is not None else "no ack from the robot")
                self.sm.disable(time.monotonic(), f"robot did not enter follow mode: {why}")
                self._robot_mode = (False, None)

    def _log_transitions(self) -> None:
        n_new = min(self.sm.transition_count - self._n_transitions, len(self.sm.transitions))
        self._n_transitions = self.sm.transition_count
        for _, a, b, reason in (self.sm.transitions[-n_new:] if n_new > 0 else []):
            self._log("FOLLOW_STATE", f"{a} -> {b}: {reason}")

    def _log(self, etype: str, reason: str, uncertainty: str = "") -> None:
        if self.logger is None:
            return
        extra = {"follow_state": self.sm.state.value, "target_id": self.sm.target_id}
        if uncertainty:
            extra["uncertainty"] = uncertainty
        try:
            self.logger.log(etype, track_id=self.sm.target_id, reason=reason, extra=extra)
        except Exception as exc:
            log.error("event log failed: %s", exc)

    def status(self) -> dict:
        d = self.last
        return {"available": True, "active": self.active, **self.sm.to_dict(),
                "left": d.left if d else 0, "right": d.right if d else 0,
                "decision": d.reason if d else "", "obstacle": self.last_obstacle.seen,
                "obstacle_reason": self.last_obstacle.reason, "commands_sent": self.commands_sent,
                "refused": self.refused,
                "note": "distance is estimated from the person's box size - not measured"}
