"""Temporal fall detection: evidence over a time window + a per-track state machine.

    NORMAL --(fall-like motion)--> POSSIBLE_FALL --(person stays down)--> FALL_SUSPECTED
       ^                                |                                       |
       |                     (timeout, never went down)              (still down for confirm_down_s)
       |                                v                                       v
       +------------------------- NORMAL (dismissed)                     FALL_CONFIRMED
       |                                                                        |
       +----------- RECOVERY_OR_RESET <---(upright for recovery_up_s, or reset)-+

Evidence used
  * motion trigger: trained fall-window classifier probability (if a model is
    loaded and enabled) or the rule baseline (fast downward hip/box-top motion
    plus a significant drop / torso rotation).
  * "down" test per frame: posture LYING, torso tilted beyond
    ``torso_angle_low_deg``, or the hips far below where they were before the
    event (covers ending up seated on the floor).

Important behaviours
  * Timers advance only with real observations of that track. A camera
    disconnect or missing frames can never confirm a fall.
  * A track lost while FALL_SUSPECTED produces a warning event, not a
    confirmation.
  * After a trigger the detector is "disarmed" until the evidence drops below
    threshold again, so one fall produces one event (deduplication), and a
    cooldown follows every recovery/reset.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from .features import WINDOW_FEATURES, body_points, window_features

NORMAL = "NORMAL"
POSSIBLE_FALL = "POSSIBLE_FALL"
FALL_SUSPECTED = "FALL_SUSPECTED"
FALL_CONFIRMED = "FALL_CONFIRMED"
RECOVERY_OR_RESET = "RECOVERY_OR_RESET"
STATES = [NORMAL, POSSIBLE_FALL, FALL_SUSPECTED, FALL_CONFIRMED, RECOVERY_OR_RESET]



@dataclass
class FallParams:
    window_s: float = 1.5
    model_threshold: float | None = None   # None = use the threshold saved with the model
    descent_speed_thresh: float = 0.9
    descent_drop_thresh: float = 0.35
    torso_angle_low_deg: float = 55.0
    possible_timeout_s: float = 2.5
    suspect_min_down_s: float = 0.4
    confirm_down_s: float = 1.5
    recovery_up_s: float = 1.5
    cooldown_s: float = 10.0
    lost_track_grace_s: float = 3.0
    max_timer_step_s: float = 0.25   # a gap between observations adds at most this much to timers
    min_window_obs: int = 4
    min_window_coverage: float = 0.5

    @classmethod
    def from_config(cls, fall_cfg: dict[str, Any]) -> "FallParams":
        names = cls.__dataclass_fields__.keys()
        kw = {k: fall_cfg[k] for k in names if k in fall_cfg}
        if isinstance(kw.get("model_threshold"), str):   # "auto"
            kw["model_threshold"] = None
        return cls(**kw)


class TrainedFallModel:
    """Fall-window classifier saved by training/train_fall_classifier.py."""

    def __init__(self, path: str | Path):
        import joblib
        path = Path(path)
        if not path.exists():
            raise FileNotFoundError(path)
        bundle = joblib.load(path)
        if bundle.get("feature_names") != WINDOW_FEATURES:
            raise ValueError(f"{path} uses different window features; retrain the fall classifier")
        self.model = bundle["model"]
        self.window_s = float(bundle.get("window_s", 1.5))
        self.meta = bundle.get("meta", {})
        self.recommended_threshold = float(bundle.get("threshold", 0.5))

    def score(self, wf: dict[str, float]) -> float:
        x = np.array([[wf[k] for k in WINDOW_FEATURES]], dtype=np.float64)
        return float(self.model.predict_proba(x)[0, 1])


def rule_motion_score(wf: dict[str, float], p: FallParams) -> float:
    """Rule baseline for 'a fall-like movement happened in this window'. Returns 0..1."""
    def ok(v):
        return v is not None and not np.isnan(v)

    speed = max([v for v in (wf["max_hip_speed"], wf["max_top_speed"], wf["max_head_speed"]) if ok(v)],
                default=float("nan"))
    drop = max([v for v in (wf["hip_drop"], wf["top_drop"], wf["head_drop"]) if ok(v)], default=float("nan"))
    if not ok(speed) or not ok(drop):
        return 0.0
    fast = speed >= p.descent_speed_thresh
    big_drop = drop >= p.descent_drop_thresh
    rotated = ok(wf["torso_angle_delta"]) and wf["torso_angle_delta"] >= 40 and \
        ok(wf["torso_angle_end"]) and wf["torso_angle_end"] >= p.torso_angle_low_deg
    if fast and (big_drop or rotated):
        return 1.0
    if rotated and speed >= 0.6 * p.descent_speed_thresh:
        return 0.75
    return 0.0


@dataclass
class TrackFallState:
    track_id: int
    state: str = NORMAL
    state_since: float = 0.0
    last_t: float | None = None
    down_time: float = 0.0
    up_time: float = 0.0
    armed: bool = True
    cooldown_until: float = -1.0
    ref_hip_y: float | None = None
    ref_height: float | None = None
    last_upright_posture: str = "UNKNOWN"
    trigger_posture: str = "UNKNOWN"
    trigger_score: float = 0.0
    trigger_t: float = 0.0
    last_score: float = 0.0
    last_posture: str = "UNKNOWN"
    lost_since: float | None = None
    created: float = 0.0
    last_down: bool | None = None
    last_box: np.ndarray | None = None
    inherited_from: list = field(default_factory=list)
    info: dict = field(default_factory=dict)


class FallDetector:
    """Runs one state machine per tracked person."""

    def __init__(self, params: FallParams, model: TrainedFallModel | None = None,
                 kpt_thr: float = 0.35):
        self.p = params
        self.model = model
        self.kpt_thr = kpt_thr
        self.tracks: dict[int, TrackFallState] = {}
        # fall states of tracks the tracker removed, kept briefly so the same person
        # can be re-linked if the tracker gives them a new id (common right after a fall)
        self.orphans: dict[int, TrackFallState] = {}

    @property
    def mode(self) -> str:
        return "model" if self.model else "rules"

    # ------------------------------------------------------------------ helpers
    def _event(self, st: TrackFallState, t: float, etype: str, prev: str, reason: str) -> dict:
        return {
            "type": etype, "t": float(t), "track_id": st.track_id,
            "previous_state": prev, "state": st.state,
            "previous_posture": st.trigger_posture if etype != "POSSIBLE_FALL" else st.last_upright_posture,
            "current_posture": st.last_posture,
            "score": round(float(st.trigger_score), 4), "evidence_mode": self.mode,
            "reason": reason,
        }

    def _set_state(self, st: TrackFallState, new: str, t: float) -> str:
        prev = st.state
        st.state = new
        st.state_since = t
        return prev

    def _is_down(self, st: TrackFallState, posture: str, frame_feats: dict[str, float],
                 box: np.ndarray, kpts: np.ndarray) -> bool | None:
        """True = person is down, False = clearly upright, None = cannot tell.

        "Cannot tell" (e.g. too few keypoints) freezes the timers: missing
        evidence never counts as recovery and never counts as being down.
        """
        if posture == "LYING":
            return True
        torso = frame_feats.get("torso_angle", float("nan"))
        if not np.isnan(torso) and torso >= self.p.torso_angle_low_deg:
            return True
        if st.ref_hip_y is not None and st.ref_height:
            hip = body_points(kpts, self.kpt_thr)["hip"][1]
            y = hip if not np.isnan(hip) else float(box[1] + 0.5 * (box[3] - box[1]))
            if (y - st.ref_hip_y) / st.ref_height >= self.p.descent_drop_thresh:
                return True
        if posture in ("STANDING", "WALKING", "SITTING", "BENDING", "KNEELING") and not np.isnan(torso):
            return False
        return None

    def motion_evidence(self, history: list) -> tuple[float | None, dict[str, float] | None]:
        """Score the last window of a track's history (list of Observation)."""
        if not history:
            return None, None
        t_end = history[-1].t
        obs = [o for o in history if t_end - o.t <= self.p.window_s + 1e-9]
        if len(obs) < self.p.min_window_obs:
            return None, None
        wf = window_features([o.t for o in obs], [o.box for o in obs], [o.keypoints for o in obs],
                             self.p.window_s, self.kpt_thr, self.p.torso_angle_low_deg)
        if wf["coverage"] < self.p.min_window_coverage:
            return None, wf
        score = self.model.score(wf) if self.model else rule_motion_score(wf, self.p)
        return score, wf

    def threshold(self) -> float:
        """Decision threshold for the motion evidence score."""
        if self.model is None:
            return 0.5          # rule score is 0, 0.75 or 1
        if self.p.model_threshold is None:
            return float(getattr(self.model, "recommended_threshold", 0.5))
        return float(self.p.model_threshold)

    # -------------------------------------------------------------- main update
    def update(self, track_id: int, t: float, history: list, posture: str,
               frame_feats: dict[str, float],
               evidence: tuple[float | None, dict | None] | None = None) -> tuple[TrackFallState, list[dict]]:
        """Process one observation of a track.

        ``history`` is the track's list of Observation (latest last).
        ``evidence`` optionally supplies a precomputed (score, window_features)
        pair; offline evaluation uses this to avoid recomputing features.
        """
        st = self.tracks.get(track_id)
        if st is None:
            st = self.tracks[track_id] = TrackFallState(track_id, state_since=t, created=t)
        events: list[dict] = []
        st.lost_since = None
        dt = 0.0 if st.last_t is None else min(max(t - st.last_t, 0.0), self.p.max_timer_step_s)
        st.last_t = t
        st.last_posture = posture
        last = history[-1]
        score, wf = evidence if evidence is not None else self.motion_evidence(history)
        st.last_score = score if score is not None else 0.0
        thr = self.threshold()
        triggered = score is not None and score >= thr
        if not triggered and score is not None:
            st.armed = True          # evidence went quiet -> allow the next trigger

        st.last_box = np.asarray(last.box, dtype=np.float64).copy()
        down = self._is_down(st, posture, frame_feats, last.box, last.keypoints)
        st.last_down = down
        if down is True:
            st.down_time += dt
            st.up_time = 0.0
        elif down is False:
            # accumulated down time is kept: a brief upright flicker is common during falls
            st.up_time += dt

        if st.state == RECOVERY_OR_RESET:
            self._set_state(st, NORMAL, t)

        if st.state == NORMAL:
            if posture in ("STANDING", "SITTING", "BENDING", "KNEELING"):
                st.last_upright_posture = posture
            if triggered and st.armed and t >= st.cooldown_until:
                st.armed = False
                st.trigger_score = score
                st.trigger_t = t
                st.trigger_posture = st.last_upright_posture
                # reference = where the hips were at the start of the window
                obs = [o for o in history if t - o.t <= self.p.window_s + 1e-9]
                ref_hips = [body_points(o.keypoints, self.kpt_thr)["hip"][1] for o in obs[: max(1, len(obs) // 4)]]
                ref_hips = [h for h in ref_hips if not np.isnan(h)]
                b0 = obs[0].box
                st.ref_hip_y = float(np.mean(ref_hips)) if ref_hips else float(b0[1] + 0.5 * (b0[3] - b0[1]))
                st.ref_height = float(max(max(o.box[3] - o.box[1], o.box[2] - o.box[0]) for o in obs))
                st.down_time = dt if down is True else 0.0
                st.up_time = 0.0
                prev = self._set_state(st, POSSIBLE_FALL, t)
                events.append(self._event(st, t, "POSSIBLE_FALL", prev,
                                          f"fall-like motion (score {score:.2f} >= {thr:.2f}, {self.mode})"))
                # fall through: the same frame may already show the person down

        if st.state == POSSIBLE_FALL:
            if st.down_time >= self.p.suspect_min_down_s:
                prev = self._set_state(st, FALL_SUSPECTED, t)
                events.append(self._event(st, t, "FALL_SUSPECTED", prev,
                                          f"down for {st.down_time:.1f}s after fall-like motion"))
            elif t - st.trigger_t > self.p.possible_timeout_s:
                prev = self._set_state(st, NORMAL, t)
                st.ref_hip_y = st.ref_height = None
                events.append(self._event(st, t, "FALL_DISMISSED", prev,
                                          "person did not stay down after the motion"))
        elif st.state == FALL_SUSPECTED:
            if st.up_time >= self.p.recovery_up_s:
                prev = self._set_state(st, RECOVERY_OR_RESET, t)
                st.cooldown_until = t + self.p.cooldown_s
                st.ref_hip_y = st.ref_height = None
                events.append(self._event(st, t, "RECOVERED", prev,
                                          f"upright for {st.up_time:.1f}s before confirmation"))
            elif st.down_time >= self.p.suspect_min_down_s + self.p.confirm_down_s:
                prev = self._set_state(st, FALL_CONFIRMED, t)
                events.append(self._event(st, t, "FALL_CONFIRMED", prev,
                                          f"person down for {st.down_time:.1f}s after a fall-like motion"))
        elif st.state == FALL_CONFIRMED:
            if st.up_time >= self.p.recovery_up_s:
                prev = self._set_state(st, RECOVERY_OR_RESET, t)
                st.cooldown_until = t + self.p.cooldown_s
                st.ref_hip_y = st.ref_height = None
                events.append(self._event(st, t, "RECOVERED", prev,
                                          f"person upright again for {st.up_time:.1f}s"))
        st.info = {"score": st.last_score, "down_time": st.down_time, "up_time": st.up_time,
                   "window": wf}
        return st, events

    def track_lost(self, track_id: int, t: float) -> list[dict]:
        """Called when the tracker removes a track."""
        st = self.tracks.pop(track_id, None)
        if st is None:
            return []
        events = []
        if st.state == FALL_SUSPECTED:
            events.append(self._event(st, t, "TRACK_LOST_DURING_FALL", st.state,
                                      "person left view while a fall was suspected; check on them"))
        if st.state in (POSSIBLE_FALL, FALL_SUSPECTED, FALL_CONFIRMED):
            st.lost_since = t
            self.orphans[track_id] = st
        return events

    def _near(self, a: np.ndarray | None, b: np.ndarray, ref: float | None) -> bool:
        if a is None:
            return False
        ca = np.array([(a[0] + a[2]) / 2, (a[1] + a[3]) / 2])
        cb = np.array([(b[0] + b[2]) / 2, (b[1] + b[3]) / 2])
        size = max(ref or 0.0, a[2] - a[0], a[3] - a[1], b[2] - b[0], b[3] - b[1], 1.0)
        return float(np.linalg.norm(ca - cb)) <= 0.75 * size

    def resolve_handover(self, t: float, visible: dict[int, np.ndarray]) -> list[dict]:
        """Re-link a fall in progress to a new track id of the same person.

        After a fall the detector often loses the person for a moment or
        splits them into two boxes, and the tracker then assigns a NEW id. If a
        young track in NORMAL state appears close to where a track with a fall
        in progress was last seen (and that track is not visible now), the
        fall state moves to the new id instead of being forgotten.
        ``visible`` maps the track ids seen in this frame to their boxes.
        """
        events: list[dict] = []
        for oid in [k for k, o in self.orphans.items() if t - (o.lost_since or t) > self.p.lost_track_grace_s]:
            del self.orphans[oid]
        candidates = {**self.orphans}
        for tid, st in self.tracks.items():
            if tid not in visible and st.state in (POSSIBLE_FALL, FALL_SUSPECTED, FALL_CONFIRMED) \
                    and st.last_t is not None and t - st.last_t <= self.p.lost_track_grace_s:
                candidates[tid] = st
        if not candidates:
            return events
        for new_id, box in visible.items():
            new = self.tracks.get(new_id)
            if new is None or new.state != NORMAL or t - new.created > self.p.lost_track_grace_s \
                    or new_id in candidates or new.last_down is False:
                continue   # an upright newcomer is more likely someone else (e.g. a helper)
            best = None
            for oid, old in candidates.items():
                if self._near(old.last_box, np.asarray(box, dtype=np.float64), old.ref_height):
                    if best is None or (old.last_t or 0) > (candidates[best].last_t or 0):
                        best = oid
            if best is None:
                continue
            old = candidates.pop(best)
            self.orphans.pop(best, None)
            self.tracks.pop(best, None)
            old.inherited_from.append(best)
            old.track_id = new_id
            old.last_t = new.last_t
            old.created = new.created
            old.lost_since = None
            self.tracks[new_id] = old
            events.append({"type": "TRACK_HANDOVER", "t": float(t), "track_id": new_id,
                           "previous_state": old.state, "state": old.state,
                           "previous_posture": old.trigger_posture, "current_posture": new.last_posture,
                           "score": round(float(old.trigger_score), 4), "evidence_mode": self.mode,
                           "reason": f"fall in progress moved from person #{best} to #{new_id} (tracker id change)"})
        return events

    def stream_interrupted(self) -> None:
        """Camera dropped: discard unconfirmed motion evidence, keep later states."""
        for st in self.tracks.values():
            if st.state == POSSIBLE_FALL:
                st.state = NORMAL
                st.ref_hip_y = st.ref_height = None
            st.last_t = None

    def reset(self, track_id: int | None = None, t: float = 0.0) -> None:
        """Operator reset (after acknowledging an alert)."""
        if track_id is None:
            self.orphans.clear()
        targets = [track_id] if track_id is not None else list(self.tracks)
        for tid in targets:
            st = self.tracks.get(tid)
            if st:
                st.state = NORMAL
                st.state_since = t
                st.cooldown_until = t + self.p.cooldown_s
                st.down_time = st.up_time = 0.0
                st.ref_hip_y = st.ref_height = None
                st.armed = False
