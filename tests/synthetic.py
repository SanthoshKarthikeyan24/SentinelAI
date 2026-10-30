"""Synthetic skeletons for unit tests (no camera or model needed)."""
from __future__ import annotations

import math

import numpy as np

# Upright body in "body units" (height 1.0), origin at hip centre, y down.
_UPRIGHT = {
    0: (0.0, -0.55), 1: (-0.02, -0.57), 2: (0.02, -0.57), 3: (-0.05, -0.56), 4: (0.05, -0.56),
    5: (-0.12, -0.38), 6: (0.12, -0.38), 7: (-0.16, -0.18), 8: (0.16, -0.18),
    9: (-0.17, 0.0), 10: (0.17, 0.0), 11: (-0.08, 0.0), 12: (0.08, 0.0),
    13: (-0.09, 0.24), 14: (0.09, 0.24), 15: (-0.09, 0.46), 16: (0.09, 0.46),
}
# Seated, facing camera: thighs foreshortened, knees bent
_SITTING = {**_UPRIGHT, 13: (-0.10, 0.06), 14: (0.10, 0.06), 15: (-0.10, 0.28), 16: (0.10, 0.28)}


def skeleton(cx: float, hip_y: float, height: float, angle_deg: float = 0.0, pose: str = "upright",
             score: float = 0.9, missing: tuple[int, ...] = ()) -> tuple[np.ndarray, np.ndarray]:
    """Return (keypoints (17,3), box) for a body rotated by angle_deg around the hips."""
    base = _SITTING if pose == "sitting" else _UPRIGHT
    a = math.radians(angle_deg)
    ca, sa = math.cos(a), math.sin(a)
    k = np.zeros((17, 3), dtype=np.float32)
    for i, (x, y) in base.items():
        rx, ry = x * ca - y * sa, x * sa + y * ca
        k[i] = (cx + rx * height, hip_y + ry * height, score)
    for i in missing:
        k[i, 2] = 0.05
    vis = k[k[:, 2] > 0.3]
    pad = 0.04 * height
    box = np.array([vis[:, 0].min() - pad, vis[:, 1].min() - pad, vis[:, 0].max() + pad,
                    vis[:, 1].max() + pad], dtype=np.float32)
    return k, box


def fall_sequence(fps: float = 15.0, stand_s: float = 2.0, fall_s: float = 0.6, down_s: float = 4.0,
                  height: float = 300.0, start_t: float = 0.0):
    """Standing -> fast fall sideways -> lying on the floor. Yields (t, kpts, box)."""
    out = []
    n_stand, n_fall, n_down = int(stand_s * fps), int(fall_s * fps), int(down_s * fps)
    floor_hip = 400 + 0.45 * height
    for i in range(n_stand):
        out.append((start_t + i / fps, *skeleton(320, 400, height, 0)))
    for i in range(n_fall):
        f = (i + 1) / n_fall
        out.append((start_t + (n_stand + i) / fps,
                    *skeleton(320 + 60 * f, 400 + (floor_hip - 400) * f, height, 88 * f)))
    for i in range(n_down):
        out.append((start_t + (n_stand + n_fall + i) / fps, *skeleton(380, floor_hip, height, 88)))
    return out


def lie_down_slowly(fps: float = 15.0, height: float = 300.0):
    """Standing -> sit (slow) -> lie down on a bed (slow): an ordinary activity."""
    out = []
    t = 0.0
    for _ in range(int(2 * fps)):
        out.append((t, *skeleton(320, 400, height, 0))); t += 1 / fps
    for i in range(int(2.5 * fps)):          # sit down over 2.5 s
        f = (i + 1) / (2.5 * fps)
        out.append((t, *skeleton(320, 400 + 0.12 * height * f, height, 0, "sitting" if f > 0.5 else "upright")))
        t += 1 / fps
    for i in range(int(3.0 * fps)):          # lean back to lying over 3 s
        f = (i + 1) / (3.0 * fps)
        out.append((t, *skeleton(320, 400 + 0.12 * height, height, -85 * f, "sitting")))
        t += 1 / fps
    for _ in range(int(4 * fps)):
        out.append((t, *skeleton(320, 400 + 0.12 * height, height, -85, "sitting"))); t += 1 / fps
    return out
