"""Shared dataset logic for training and evaluation.

* loading the video index, labelled intervals and cached pose files
* building per-frame posture samples and per-window fall samples
* replaying cached pose sequences through the SAME PostureClassifier and
  FallDetector state machine used live, to measure event-level performance
* matching predicted alarms to labelled fall events
"""
from __future__ import annotations

import csv
import hashlib
import json
from collections import defaultdict
from dataclasses import dataclass, field

import joblib
import numpy as np

from common import EVENTS_CSV, POSE_FEATURES, SPLITS, TEMPORAL, VIDEO_INDEX
from src.analysis.fall_detector import FallDetector, FallParams, rule_motion_score
from src.analysis.features import (FRAME_FEATURES, WINDOW_FEATURES, frame_features,
                                   visible_keypoints, window_features)
from src.analysis.posture import PostureClassifier
from src.tracking.tracker import Observation

POSTURE_FROM_INTERVAL = {"STANDING": "STANDING", "WALKING": "STANDING", "SITTING": "SITTING",
                         "LYING": "LYING", "BENDING": "BENDING", "KNEELING": "KNEELING"}
EXCLUDE_POSTURE_IF = {"ACT_EXERCISING", "FALL", "FALL_LIKE_EVENT", "OCCLUDED", "UNKNOWN", "RECOVERY"}
FALL_LABELS = {"FALL", "FALL_LIKE_EVENT"}

# fall-window labelling (seconds relative to the labelled fall start)
POS_START, POS_END = 0.4, 2.0      # window end in [start+0.4, start+2.0]  -> positive
IGNORE_BEFORE = 0.3                # window end in [start-0.3, start+0.4)  -> ignored (boundary)
BOUNDARY_MARGIN = 0.15             # posture: skip frames this close to an interval edge


# ----------------------------------------------------------------- loading
def load_index() -> list[dict]:
    with open(VIDEO_INDEX, newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    for r in rows:
        r["duration_s"] = float(r["duration_s"])
    return rows


def load_events() -> dict[str, list[dict]]:
    out: dict[str, list[dict]] = defaultdict(list)
    with open(EVENTS_CSV, newline="", encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            r["start_s"], r["end_s"] = float(r["start_s"]), float(r["end_s"])
            out[r["video_id"]].append(r)
    return out


def load_splits() -> dict:
    with open(SPLITS / "splits.json", encoding="utf-8") as fh:
        return json.load(fh)


def load_pose(video_id: str) -> dict:
    with np.load(POSE_FEATURES / f"{video_id}.npz") as z:
        return {k: z[k] for k in z.files}


# ------------------------------------------------------- per-video caching
@dataclass
class Entry:
    t: float
    box: np.ndarray
    kpts: np.ndarray
    feats: dict
    n_vis: int
    wf: dict | None          # window features ending at t (None if too few observations)
    wf_valid: bool


@dataclass
class VideoCache:
    video_id: str
    frame_times: np.ndarray
    tracks: dict[int, list[Entry]] = field(default_factory=dict)
    primary: int | None = None


def _params_key(p: FallParams, kpt_thr: float, history_s: float) -> str:
    raw = json.dumps([p.window_s, p.torso_angle_low_deg, p.min_window_obs, kpt_thr, history_s,
                      FRAME_FEATURES, WINDOW_FEATURES])
    return hashlib.md5(raw.encode()).hexdigest()[:10]


def build_video_cache(video_id: str, p: FallParams, kpt_thr: float, history_s: float) -> VideoCache:
    pose = load_pose(video_id)
    vc = VideoCache(video_id, pose["frame_times"])
    tids = pose["track_id"]
    if len(tids) == 0:
        return vc
    counts = {int(t): int((tids == t).sum()) for t in np.unique(tids)}
    vc.primary = max(counts, key=counts.get)
    for tid in counts:
        idx = np.where(tids == tid)[0]
        idx = idx[np.argsort(pose["t"][idx], kind="stable")]
        hist: list[Observation] = []
        entries = []
        for i in idx:
            t = float(pose["t"][i])
            box, k = pose["box"][i], pose["kpts"][i]
            hist.append(Observation(t, box, k, float(pose["score"][i]), (0, 0)))
            while hist and t - hist[0].t > history_s:
                hist.pop(0)
            win = [o for o in hist if t - o.t <= p.window_s + 1e-9]
            wf, valid = None, False
            if len(win) >= p.min_window_obs:
                wf = window_features([o.t for o in win], [o.box for o in win], [o.keypoints for o in win],
                                     p.window_s, kpt_thr, p.torso_angle_low_deg)
                valid = wf["coverage"] >= p.min_window_coverage
            entries.append(Entry(t, box, k, frame_features(k, box, kpt_thr), visible_keypoints(k, kpt_thr),
                                 wf, valid))
        vc.tracks[tid] = entries
    return vc


def build_all_caches(video_ids: list[str], p: FallParams, kpt_thr: float, history_s: float,
                     log=None) -> dict[str, VideoCache]:
    """Build (or load from data/processed/temporal_sequences) feature caches for all videos."""
    TEMPORAL.mkdir(parents=True, exist_ok=True)
    path = TEMPORAL / f"cache_{_params_key(p, kpt_thr, history_s)}.joblib"
    cache: dict[str, VideoCache] = {}
    if path.exists():
        try:
            cache = joblib.load(path)
        except Exception:
            cache = {}
    missing = [v for v in video_ids if v not in cache and (POSE_FEATURES / f"{v}.npz").exists()]
    for k, vid in enumerate(missing, 1):
        cache[vid] = build_video_cache(vid, p, kpt_thr, history_s)
        if log and k % 20 == 0:
            log.info("  feature cache: %d/%d videos", k, len(missing))
    if missing:
        joblib.dump(cache, path, compress=3)
    return {v: cache[v] for v in video_ids if v in cache}


# ---------------------------------------------------------- posture samples
def interval_label_at(events: list[dict], t: float) -> str | None:
    """Posture label for time t, or None if unlabelled/ambiguous/excluded."""
    labels = set()
    for e in events:
        if e["start_s"] - 1e-9 <= t <= e["end_s"] + 1e-9:
            if e["label"] in EXCLUDE_POSTURE_IF:
                return None
            if e["label"] in POSTURE_FROM_INTERVAL:
                if t - e["start_s"] < BOUNDARY_MARGIN or e["end_s"] - t < BOUNDARY_MARGIN:
                    # near an edge: only keep if the video starts/ends there
                    if not (e["start_s"] <= 0.01 and t - e["start_s"] < BOUNDARY_MARGIN):
                        return None
                labels.add(POSTURE_FROM_INTERVAL[e["label"]])
    if len(labels) != 1:
        return None
    return labels.pop()


def posture_samples(caches: dict[str, VideoCache], events: dict[str, list[dict]], index: dict[str, dict],
                    min_visible: int):
    """Frame-level samples from the primary (labelled) person of each video."""
    X, y, groups, vids = [], [], [], []
    for vid, vc in caches.items():
        if vc.primary is None:
            continue
        for e in vc.tracks[vc.primary]:
            lab = interval_label_at(events.get(vid, []), e.t)
            if lab is None or e.n_vis < min_visible:
                continue
            X.append([e.feats[k] for k in FRAME_FEATURES])
            y.append(lab)
            groups.append(index[vid]["subject"])
            vids.append(vid)
    return np.array(X, dtype=np.float64), np.array(y), np.array(groups), np.array(vids)


# ------------------------------------------------------------- fall samples
def fall_window_label(events: list[dict], t: float) -> int | None:
    """1 = window ending at t contains a fall onset, 0 = no fall, None = ignore."""
    for e in events:
        if e["label"] not in FALL_LABELS:
            continue
        s, end = e["start_s"], e["end_s"]
        if s + POS_START <= t <= min(end, s + POS_END):
            return 1
        if s - IGNORE_BEFORE <= t <= end:
            return None
    return 0


def fall_samples(caches: dict[str, VideoCache], events: dict[str, list[dict]], index: dict[str, dict]):
    """Window samples from ALL tracks (non-primary tracks give extra negatives)."""
    X, y, groups, vids = [], [], [], []
    for vid, vc in caches.items():
        for tid, entries in vc.tracks.items():
            for e in entries:
                if not e.wf_valid:
                    continue
                lab = fall_window_label(events.get(vid, []), e.t) if tid == vc.primary else 0
                if lab is None:
                    continue
                X.append([e.wf[k] for k in WINDOW_FEATURES])
                y.append(lab)
                groups.append(index[vid]["subject"])
                vids.append(vid)
    return np.array(X, dtype=np.float64), np.array(y, dtype=int), np.array(groups), np.array(vids)


# ------------------------------------------------------------------ replay
def replay_video(vc: VideoCache, posture: PostureClassifier, fall: FallDetector,
                 fall_score_fn=None, posture_proba_fn=None, posture_classes=None,
                 min_hits: int = 2, max_missed_s: float = 1.5) -> dict:
    """Run cached detections through the live decision logic.

    fall_score_fn(X) -> probabilities for valid windows (None = rule baseline).
    posture_proba_fn(X) -> class probabilities (None = rules or posture.model).
    Returns alarms and per-frame posture predictions for the primary track.
    """
    # batch model predictions per track
    fall_scores: dict[int, np.ndarray] = {}
    post_out: dict[int, list] = {}
    for tid, entries in vc.tracks.items():
        sc = np.full(len(entries), np.nan)
        valid = [i for i, e in enumerate(entries) if e.wf_valid]
        if valid:
            if fall_score_fn is not None:
                Xw = np.array([[entries[i].wf[k] for k in WINDOW_FEATURES] for i in valid])
                sc[valid] = fall_score_fn(Xw)
            else:
                sc[valid] = [rule_motion_score(entries[i].wf, fall.p) for i in valid]
        fall_scores[tid] = sc
        if posture_proba_fn is not None and entries:
            Xf = np.array([[e.feats[k] for k in FRAME_FEATURES] for e in entries])
            pr = posture_proba_fn(Xf)
            post_out[tid] = [(posture_classes[j], float(pr[i, j])) for i, j in enumerate(pr.argmax(1))]
    if posture_classes is not None:
        posture.offline_classes = list(posture_classes)

    by_time: dict[float, list[tuple[int, int]]] = defaultdict(list)
    for tid, entries in vc.tracks.items():
        for i, e in enumerate(entries):
            by_time[e.t].append((tid, i))
    alarms, primary_postures = [], []
    hits: dict[int, int] = defaultdict(int)
    last_seen: dict[int, float] = {}
    hist: dict[int, list] = defaultdict(list)
    for t in vc.frame_times:
        t = float(t)
        visible: dict[int, np.ndarray] = {}
        for tid in [k for k, ls in last_seen.items() if t - ls > max_missed_s]:
            for ev in fall.track_lost(tid, t):
                alarms.append(ev)
            posture.forget(tid)
            del last_seen[tid]
        for tid, i in by_time.get(t, []):
            e = vc.tracks[tid][i]
            hits[tid] += 1
            last_seen[tid] = t
            hist[tid].append(Observation(t, e.box, e.kpts, 0.0, (0, 0)))
            res = posture.classify_features(e.feats, e.n_vis,
                                            model_output=post_out[tid][i] if tid in post_out else None)
            res = posture.smooth(tid, res)
            if tid == vc.primary:
                primary_postures.append((t, res.label))
            if hits[tid] < min_hits:
                continue
            s = fall_scores[tid][i]
            evidence = (None if np.isnan(s) else float(s), e.wf) if e.wf is not None else (None, None)
            if e.wf is not None and not e.wf_valid:
                evidence = (None, e.wf)
            _, evs = fall.update(tid, t, hist[tid][-50:], res.label, e.feats, evidence=evidence)
            alarms += evs
            visible[tid] = e.box
        alarms += fall.resolve_handover(t, visible)
    return {"alarms": alarms, "primary_postures": primary_postures}


# ---------------------------------------------------------- event matching
def match_events(alarms: list[dict], gt: list[tuple[float, float]], alarm_type: str,
                 tol_before: float = 0.5, tol_after: float = 1.0) -> dict:
    """Match alarms of one type to ground-truth fall intervals of one video."""
    times = sorted(a["t"] for a in alarms if a["type"] == alarm_type)
    matched = [None] * len(gt)
    fp, dup = 0, 0
    for ta in times:
        hit = None
        for g, (s, e) in enumerate(gt):
            if s - tol_before <= ta <= e + tol_after:
                hit = g
                break
        if hit is None:
            fp += 1
        elif matched[hit] is None:
            matched[hit] = ta
        else:
            dup += 1
    tp = sum(m is not None for m in matched)
    delays = [m - s for m, (s, _) in zip(matched, gt) if m is not None]
    return {"tp": tp, "fn": len(gt) - tp, "fp": fp, "duplicates": dup, "delays": delays}


def summarise_matches(results: list[dict], negative_seconds: float) -> dict:
    tp = sum(r["tp"] for r in results)
    fn = sum(r["fn"] for r in results)
    fp = sum(r["fp"] for r in results)
    delays = [d for r in results for d in r["delays"]]
    prec = tp / (tp + fp) if tp + fp else float("nan")
    rec = tp / (tp + fn) if tp + fn else float("nan")
    f1 = 2 * prec * rec / (prec + rec) if tp and (prec + rec) else 0.0
    hours = negative_seconds / 3600.0
    return {
        "true_positives": tp, "false_negatives (missed)": fn, "false_positives": fp,
        "duplicates": sum(r["duplicates"] for r in results),
        "precision": round(prec, 4), "recall": round(rec, 4), "f1": round(f1, 4),
        "false_alarms_per_hour": round(fp / hours, 2) if hours > 0 else None,
        "negative_hours_observed": round(hours, 4),
        "delay_mean_s": round(float(np.mean(delays)), 3) if delays else None,
        "delay_median_s": round(float(np.median(delays)), 3) if delays else None,
        "delay_max_s": round(float(np.max(delays)), 3) if delays else None,
    }


def gt_falls(events: list[dict]) -> list[tuple[float, float]]:
    return sorted((e["start_s"], e["end_s"]) for e in events if e["label"] in FALL_LABELS)
