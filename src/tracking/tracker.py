"""Multi-person tracker (IoU + centre-distance association, Hungarian matching).

Each person gets a persistent ``track_id`` and their own history of boxes and
keypoints, so posture/fall analysis never mixes two people's movements.

Why not only IoU? During a fall the box changes quickly from tall/narrow to
short/wide, so IoU between consecutive frames can drop. We therefore allow a
fallback match on centre distance (normalised by box size), with a higher
cost so IoU matches are always preferred.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import linear_sum_assignment

from ..detection.yolo_onnx import PersonDetection

_INVALID = 1e6


@dataclass
class Observation:
    t: float                  # timestamp in seconds
    box: np.ndarray           # x1, y1, x2, y2
    keypoints: np.ndarray     # (17, 3)
    score: float
    frame_size: tuple[int, int]  # (width, height)


@dataclass
class Track:
    track_id: int
    box: np.ndarray
    keypoints: np.ndarray
    score: float
    first_seen: float
    last_seen: float
    hits: int = 1
    missed_frames: int = 0
    confirmed: bool = False
    history: deque = field(default_factory=deque)

    @property
    def area(self) -> float:
        return float(max(0.0, self.box[2] - self.box[0]) * max(0.0, self.box[3] - self.box[1]))

    def age(self, now: float) -> float:
        return now - self.first_seen


def iou_matrix(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Pairwise IoU between boxes a (N,4) and b (M,4)."""
    if len(a) == 0 or len(b) == 0:
        return np.zeros((len(a), len(b)), dtype=np.float32)
    x1 = np.maximum(a[:, None, 0], b[None, :, 0])
    y1 = np.maximum(a[:, None, 1], b[None, :, 1])
    x2 = np.minimum(a[:, None, 2], b[None, :, 2])
    y2 = np.minimum(a[:, None, 3], b[None, :, 3])
    inter = np.clip(x2 - x1, 0, None) * np.clip(y2 - y1, 0, None)
    area_a = (a[:, 2] - a[:, 0]) * (a[:, 3] - a[:, 1])
    area_b = (b[:, 2] - b[:, 0]) * (b[:, 3] - b[:, 1])
    union = area_a[:, None] + area_b[None, :] - inter
    return np.where(union > 0, inter / np.maximum(union, 1e-9), 0.0).astype(np.float32)


class PersonTracker:
    def __init__(self, iou_match_threshold: float = 0.25, max_center_distance: float = 0.6,
                 min_hits: int = 2, max_missed_s: float = 1.5, history_s: float = 6.0):
        self.iou_thr = float(iou_match_threshold)
        self.max_center_distance = float(max_center_distance)
        self.min_hits = int(min_hits)
        self.max_missed_s = float(max_missed_s)
        self.history_s = float(history_s)
        self.tracks: dict[int, Track] = {}
        self._next_id = 1
        self.removed_ids: list[int] = []   # ids removed during the last update

    def reset(self) -> None:
        self.tracks.clear()
        self.removed_ids = []

    def _cost_matrix(self, tracks: list[Track], dets: list[PersonDetection]) -> np.ndarray:
        tb = np.array([t.box for t in tracks], dtype=np.float32).reshape(-1, 4)
        db = np.array([d.box for d in dets], dtype=np.float32).reshape(-1, 4)
        ious = iou_matrix(tb, db)
        cost = np.full(ious.shape, _INVALID, dtype=np.float64)
        tc = np.column_stack([(tb[:, 0] + tb[:, 2]) / 2, (tb[:, 1] + tb[:, 3]) / 2])
        dc = np.column_stack([(db[:, 0] + db[:, 2]) / 2, (db[:, 1] + db[:, 3]) / 2])
        tdiag = np.hypot(tb[:, 2] - tb[:, 0], tb[:, 3] - tb[:, 1])
        ddiag = np.hypot(db[:, 2] - db[:, 0], db[:, 3] - db[:, 1])
        for i in range(len(tracks)):
            for j in range(len(dets)):
                if ious[i, j] >= self.iou_thr:
                    cost[i, j] = 1.0 - ious[i, j]
                else:
                    diag = max(tdiag[i], ddiag[j], 1.0)
                    dist = float(np.hypot(*(tc[i] - dc[j]))) / diag
                    if dist <= self.max_center_distance:
                        cost[i, j] = 1.0 + dist
        return cost

    def update(self, detections: list[PersonDetection], t: float,
               frame_size: tuple[int, int]) -> list[Track]:
        """Associate detections with tracks. Sets ``det.track_id``.

        Returns the list of currently *visible* tracks (matched this frame).
        """
        self.removed_ids = []
        tracks = list(self.tracks.values())
        matched_tracks: set[int] = set()
        matched_dets: set[int] = set()
        if tracks and detections:
            cost = self._cost_matrix(tracks, detections)
            rows, cols = linear_sum_assignment(cost)
            for r, c in zip(rows, cols):
                if cost[r, c] >= _INVALID:
                    continue
                self._apply(tracks[r], detections[c], t, frame_size)
                matched_tracks.add(tracks[r].track_id)
                matched_dets.add(c)
        for j, det in enumerate(detections):
            if j in matched_dets:
                continue
            tr = Track(track_id=self._next_id, box=det.box.copy(), keypoints=det.keypoints.copy(),
                       score=det.score, first_seen=t, last_seen=t,
                       confirmed=self.min_hits <= 1)
            tr.history.append(Observation(t, det.box.copy(), det.keypoints.copy(), det.score, frame_size))
            det.track_id = tr.track_id
            self.tracks[tr.track_id] = tr
            matched_tracks.add(tr.track_id)
            self._next_id += 1
        for tr in tracks:
            if tr.track_id not in matched_tracks:
                tr.missed_frames += 1
                if t - tr.last_seen > self.max_missed_s:
                    self.removed_ids.append(tr.track_id)
                    del self.tracks[tr.track_id]
        return [self.tracks[i] for i in sorted(matched_tracks) if i in self.tracks]

    def _apply(self, tr: Track, det: PersonDetection, t: float, frame_size) -> None:
        tr.box = det.box.copy()
        tr.keypoints = det.keypoints.copy()
        tr.score = det.score
        tr.last_seen = t
        tr.hits += 1
        tr.missed_frames = 0
        if tr.hits >= self.min_hits:
            tr.confirmed = True
        tr.history.append(Observation(t, det.box.copy(), det.keypoints.copy(), det.score, frame_size))
        while tr.history and t - tr.history[0].t > self.history_s:
            tr.history.popleft()
        det.track_id = tr.track_id

    def primary_track(self) -> Track | None:
        """Largest confirmed visible track — used later by person-following."""
        visible = [t for t in self.tracks.values() if t.confirmed and t.missed_frames == 0]
        return max(visible, key=lambda t: t.area) if visible else None
