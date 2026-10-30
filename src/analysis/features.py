"""Pose feature extraction.

Two kinds of features are produced:

1. *Frame features* describe the body shape in ONE frame (used by the posture
   classifier). Every length is divided by a body-size reference so the values
   do not depend on camera resolution or how far away the person is.
2. *Window features* describe how the body moved over the last ``window_s``
   seconds (used by the fall classifier and the rule baseline).

Missing keypoints (score < threshold) become NaN instead of fake zeros; the
trained models handle NaN natively and the rules check for it explicitly.
"""
from __future__ import annotations

import math
from typing import Sequence

import numpy as np

from ..detection.yolo_onnx import KP

# ---------------------------------------------------------------- frame level
FRAME_FEATURES = [
    "torso_angle",        # shoulder-mid -> hip-mid tilt from vertical (deg, 0 = upright, 90 = horizontal)
    "body_angle",         # shoulder-mid -> ankle/knee-mid tilt from vertical (deg)
    "aspect_ratio",       # box width / box height
    "hip_angle",          # mean shoulder-hip-knee angle (deg, ~180 straight, ~90 seated)
    "knee_angle",         # mean hip-knee-ankle angle (deg)
    "dy_shoulder_hip",    # (hip_y - shoulder_y) / size  (>0: shoulders above hips)
    "dy_hip_knee",        # (knee_y - hip_y) / size
    "dy_hip_ankle",       # (ankle_y - hip_y) / size
    "dy_head_hip",        # (hip_y - head_y) / size
    "dx_shoulder_hip",    # |shoulder_x - hip_x| / size
    "torso_len",          # torso length / size
    "thigh_ratio",        # thigh length / torso length (small when thighs point at camera)
    "shin_ratio",         # shin length / torso length
    "shoulder_width_ratio",  # shoulder width / torso length
    "hip_rel_y",          # hip y position inside the box (0 = top, 1 = bottom)
    "head_rel_y",         # head y position inside the box
    "visible_frac",       # fraction of the 17 keypoints above the score threshold
    "mean_kpt_score",     # mean keypoint score
]


def _pt(kpts: np.ndarray, name: str, thr: float) -> np.ndarray:
    k = kpts[KP[name]]
    return k[:2].astype(np.float64) if k[2] >= thr else np.array([np.nan, np.nan])


def _mid(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Midpoint of two points; if one is missing use the other."""
    if np.isnan(a).any() and np.isnan(b).any():
        return np.array([np.nan, np.nan])
    if np.isnan(a).any():
        return b
    if np.isnan(b).any():
        return a
    return (a + b) / 2.0


def _tilt_from_vertical(top: np.ndarray, bottom: np.ndarray) -> float:
    """Angle (deg) between the vector bottom->top and 'straight up' in image coords."""
    if np.isnan(top).any() or np.isnan(bottom).any():
        return float("nan")
    v = top - bottom
    n = math.hypot(v[0], v[1])
    if n < 1e-6:
        return float("nan")
    # image y grows downwards, so "up" is (0, -1)
    cos = max(-1.0, min(1.0, -v[1] / n))
    return math.degrees(math.acos(cos))


def _joint_angle(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
    """Angle ABC in degrees."""
    if np.isnan(a).any() or np.isnan(b).any() or np.isnan(c).any():
        return float("nan")
    v1, v2 = a - b, c - b
    n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
    if n1 < 1e-6 or n2 < 1e-6:
        return float("nan")
    cos = float(np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0))
    return math.degrees(math.acos(cos))


def _dist(a: np.ndarray, b: np.ndarray) -> float:
    if np.isnan(a).any() or np.isnan(b).any():
        return float("nan")
    return float(np.linalg.norm(a - b))


def _nanmean(values: Sequence[float]) -> float:
    vals = [v for v in values if not math.isnan(v)]
    return float(sum(vals) / len(vals)) if vals else float("nan")


def body_points(kpts: np.ndarray, thr: float) -> dict[str, np.ndarray]:
    ls, rs = _pt(kpts, "left_shoulder", thr), _pt(kpts, "right_shoulder", thr)
    lh, rh = _pt(kpts, "left_hip", thr), _pt(kpts, "right_hip", thr)
    lk, rk = _pt(kpts, "left_knee", thr), _pt(kpts, "right_knee", thr)
    la, ra = _pt(kpts, "left_ankle", thr), _pt(kpts, "right_ankle", thr)
    head_cands = [_pt(kpts, n, thr) for n in ("nose", "left_eye", "right_eye", "left_ear", "right_ear")]
    head_valid = [h for h in head_cands if not np.isnan(h).any()]
    head = np.mean(head_valid, axis=0) if head_valid else np.array([np.nan, np.nan])
    return {
        "ls": ls, "rs": rs, "lh": lh, "rh": rh, "lk": lk, "rk": rk, "la": la, "ra": ra,
        "shoulder": _mid(ls, rs), "hip": _mid(lh, rh), "knee": _mid(lk, rk),
        "ankle": _mid(la, ra), "head": head,
    }


def frame_features(kpts: np.ndarray, box: np.ndarray, kpt_thr: float = 0.35) -> dict[str, float]:
    """Compute the FRAME_FEATURES for one person in one frame."""
    x1, y1, x2, y2 = [float(v) for v in box]
    bw, bh = max(x2 - x1, 1.0), max(y2 - y1, 1.0)
    size = max(bw, bh)  # orientation-independent body size reference
    p = body_points(kpts, kpt_thr)
    sh, hp, kn, an, hd = p["shoulder"], p["hip"], p["knee"], p["ankle"], p["head"]
    lower = an if not np.isnan(an).any() else kn
    torso = _dist(sh, hp)

    def ratio(v: float) -> float:
        return v / torso if (not math.isnan(v) and not math.isnan(torso) and torso > 1e-6) else float("nan")

    def dy(a: np.ndarray, b: np.ndarray) -> float:  # (b_y - a_y) / size
        return float((b[1] - a[1]) / size) if not (np.isnan(a).any() or np.isnan(b).any()) else float("nan")

    thighs = _nanmean([_dist(p["lh"], p["lk"]), _dist(p["rh"], p["rk"])])
    shins = _nanmean([_dist(p["lk"], p["la"]), _dist(p["rk"], p["ra"])])
    scores = kpts[:, 2]
    return {
        "torso_angle": _tilt_from_vertical(sh, hp),
        "body_angle": _tilt_from_vertical(sh, lower),
        "aspect_ratio": bw / bh,
        "hip_angle": _nanmean([_joint_angle(p["ls"], p["lh"], p["lk"]),
                               _joint_angle(p["rs"], p["rh"], p["rk"])]),
        "knee_angle": _nanmean([_joint_angle(p["lh"], p["lk"], p["la"]),
                                _joint_angle(p["rh"], p["rk"], p["ra"])]),
        "dy_shoulder_hip": dy(sh, hp),
        "dy_hip_knee": dy(hp, kn),
        "dy_hip_ankle": dy(hp, an),
        "dy_head_hip": dy(hd, hp),
        "dx_shoulder_hip": float(abs(sh[0] - hp[0]) / size) if not (np.isnan(sh).any() or np.isnan(hp).any()) else float("nan"),
        "torso_len": torso / size if not math.isnan(torso) else float("nan"),
        "thigh_ratio": ratio(thighs),
        "shin_ratio": ratio(shins),
        "shoulder_width_ratio": ratio(_dist(p["ls"], p["rs"])),
        "hip_rel_y": float((hp[1] - y1) / bh) if not np.isnan(hp).any() else float("nan"),
        "head_rel_y": float((hd[1] - y1) / bh) if not np.isnan(hd).any() else float("nan"),
        "visible_frac": float((scores >= kpt_thr).sum()) / len(scores),
        "mean_kpt_score": float(scores.mean()),
    }


def frame_feature_vector(kpts: np.ndarray, box: np.ndarray, kpt_thr: float = 0.35) -> np.ndarray:
    f = frame_features(kpts, box, kpt_thr)
    return np.array([f[k] for k in FRAME_FEATURES], dtype=np.float64)


def visible_keypoints(kpts: np.ndarray, thr: float) -> int:
    return int((kpts[:, 2] >= thr).sum())


# --------------------------------------------------------------- window level
WINDOW_FEATURES = [
    "n_obs", "coverage",                 # observations in window, fraction of window covered
    "hip_drop", "top_drop", "head_drop",  # end - start vertical position / reference height
    "max_hip_speed", "max_top_speed", "max_head_speed",  # max downward speed (ref heights / s)
    "max_up_speed",                      # max upward speed of box top (getting up / sitting up)
    "torso_angle_start", "torso_angle_end", "torso_angle_delta", "torso_angle_max",
    "aspect_start", "aspect_end", "aspect_delta", "aspect_max",
    "height_ratio_end",                  # box height at end / reference height
    "down_frac",                         # fraction of observations with torso tilted beyond 55 deg
    "hip_angle_end", "knee_angle_end", "dy_head_hip_end", "body_angle_end",
    "visible_frac_mean",
    # added after the first error analysis (push-ups / sitting on the floor were confused with falls)
    "peak_top_drop",                     # largest drop of the box top inside the window (ref heights)
    "min_height_ratio",                  # smallest box height / reference height
    "max_center_speed",                  # fastest 2-D movement of the box centre (ref heights / s)
    "start_motion", "end_motion",        # keypoint movement per second in the first / last 0.5 s
    "wrist_hip_dy_end",                  # (wrist_y - hip_y) / ref at the end (hands on floor in push-ups)
]
BASE_WINDOW_FEATURES = WINDOW_FEATURES[:24]   # the original set, kept for comparison experiments

_SPEED_SPAN_S = 0.4   # vertical speed measured over ~0.4 s spans (robust to jitter and fps)


def _span_speeds(t: np.ndarray, y: np.ndarray, span: float = _SPEED_SPAN_S) -> tuple[float, float]:
    """Max downward and max upward speed of y (positive = moving down the image)."""
    valid = ~np.isnan(y)
    t, y = t[valid], y[valid]
    if len(t) < 2:
        return float("nan"), float("nan")
    down, up = -np.inf, -np.inf
    j = 0
    for i in range(len(t)):
        if j <= i:
            j = i + 1
        while j < len(t) - 1 and t[j] - t[i] < span:
            j += 1
        if j >= len(t):
            break
        dt = t[j] - t[i]
        if dt < span * 0.5:   # not enough time between samples for a stable speed
            continue
        v = (y[j] - y[i]) / dt
        down, up = max(down, v), max(up, -v)
    if down == -np.inf:
        return float("nan"), float("nan")
    return float(down), float(up)


def _first_last(vals: np.ndarray) -> tuple[float, float]:
    v = vals[~np.isnan(vals)]
    if len(v) == 0:
        return float("nan"), float("nan")
    k = max(1, len(v) // 5)   # average a few samples at each end to reduce jitter
    return float(np.mean(v[:k])), float(np.mean(v[-k:]))


def window_features(times: Sequence[float], boxes: Sequence[np.ndarray], kpts_seq: Sequence[np.ndarray],
                    window_s: float, kpt_thr: float = 0.35, down_angle: float = 55.0) -> dict[str, float]:
    """Temporal features for the observations of ONE track inside the window.

    ``times`` must be increasing and should already be restricted to the window
    (callers pass the last ``window_s`` seconds of a track history).
    """
    n = len(times)
    out = {k: float("nan") for k in WINDOW_FEATURES}
    out["n_obs"] = float(n)
    out["coverage"] = 0.0
    if n == 0:
        return out
    t = np.asarray(times, dtype=np.float64)
    out["coverage"] = float(min(1.0, (t[-1] - t[0]) / window_s)) if n > 1 else 0.0
    b = np.asarray(boxes, dtype=np.float64).reshape(-1, 4)
    heights = b[:, 3] - b[:, 1]
    widths = b[:, 2] - b[:, 0]
    ref = float(np.max(np.maximum(heights, widths)))   # standing height ~ largest box side
    ref = max(ref, 1.0)

    fr = [frame_features(k, bx, kpt_thr) for k, bx in zip(kpts_seq, b)]
    hip_y = np.array([np.nan] * n)
    head_y = np.array([np.nan] * n)
    for i, k in enumerate(kpts_seq):
        p = body_points(k, kpt_thr)
        hip_y[i] = p["hip"][1]
        head_y[i] = p["head"][1]
    top_y = b[:, 1]
    torso = np.array([f["torso_angle"] for f in fr])
    aspect = np.array([f["aspect_ratio"] for f in fr])

    for name, ys in (("hip", hip_y), ("top", top_y), ("head", head_y)):
        s, e = _first_last(ys / ref)
        out[f"{name}_drop"] = e - s if not (math.isnan(s) or math.isnan(e)) else float("nan")
        down, up = _span_speeds(t, ys / ref)
        out[f"max_{name}_speed"] = down
        if name == "top":
            out["max_up_speed"] = up
    s, e = _first_last(torso)
    out["torso_angle_start"], out["torso_angle_end"] = s, e
    out["torso_angle_delta"] = e - s if not (math.isnan(s) or math.isnan(e)) else float("nan")
    out["torso_angle_max"] = float(np.nanmax(torso)) if not np.isnan(torso).all() else float("nan")
    s, e = _first_last(aspect)
    out["aspect_start"], out["aspect_end"], out["aspect_delta"] = s, e, e - s
    out["aspect_max"] = float(np.max(aspect))
    out["height_ratio_end"] = float(heights[-1] / ref)
    valid_t = torso[~np.isnan(torso)]
    out["down_frac"] = float((valid_t >= down_angle).mean()) if len(valid_t) else float("nan")
    last = fr[-1]
    out["hip_angle_end"] = last["hip_angle"]
    out["knee_angle_end"] = last["knee_angle"]
    out["dy_head_hip_end"] = last["dy_head_hip"]
    out["body_angle_end"] = last["body_angle"]
    out["visible_frac_mean"] = float(np.mean([f["visible_frac"] for f in fr]))

    k0 = max(1, n // 5)
    out["peak_top_drop"] = float(np.max(top_y - np.mean(top_y[:k0])) / ref)
    out["min_height_ratio"] = float(np.min(heights) / ref)
    cx = (b[:, 0] + b[:, 2]) / 2.0 / ref
    cy = (b[:, 1] + b[:, 3]) / 2.0 / ref
    sx = _span_speeds(t, cx)
    sy = _span_speeds(t, cy)
    if not any(math.isnan(v) for v in (*sx, *sy)):
        out["max_center_speed"] = float(math.hypot(max(sx), max(sy)))
    out["start_motion"] = _motion_rate(t, kpts_seq, kpt_thr, ref, t[0], t[0] + 0.5)
    out["end_motion"] = _motion_rate(t, kpts_seq, kpt_thr, ref, t[-1] - 0.5, t[-1])
    pts = body_points(kpts_seq[-1], kpt_thr)
    wr = [kpts_seq[-1][i] for i in (9, 10) if kpts_seq[-1][i][2] >= kpt_thr]
    if wr and not np.isnan(pts["hip"]).any():
        out["wrist_hip_dy_end"] = float((np.mean([w[1] for w in wr]) - pts["hip"][1]) / ref)
    return out


def _motion_rate(t: np.ndarray, kpts_seq, thr: float, ref: float, t0: float, t1: float) -> float:
    """Mean keypoint speed (reference heights per second) between t0 and t1."""
    idx = [i for i in range(len(t)) if t0 - 1e-9 <= t[i] <= t1 + 1e-9]
    speeds = []
    for a, c in zip(idx[:-1], idx[1:]):
        dt = t[c] - t[a]
        if dt <= 0:
            continue
        ka, kc = kpts_seq[a], kpts_seq[c]
        both = (ka[:, 2] >= thr) & (kc[:, 2] >= thr)
        if both.sum() >= 4:
            d = np.linalg.norm(kc[both, :2] - ka[both, :2], axis=1).mean()
            speeds.append(d / ref / dt)
    return float(np.mean(speeds)) if speeds else float("nan")


def window_feature_vector(*args, **kwargs) -> np.ndarray:
    f = window_features(*args, **kwargs)
    return np.array([f[k] for k in WINDOW_FEATURES], dtype=np.float64)
