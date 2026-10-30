"""Posture classification: a documented rule-based baseline and a trained model.

Labels: STANDING, SITTING, BENDING, KNEELING, LYING, UNKNOWN.

The trained model (Random Forest on FRAME_FEATURES) can only predict the
classes it was trained on. With the public GMDCSA-24 data these are STANDING,
SITTING and LYING. BENDING/KNEELING come from the rule baseline until you
collect and label your own examples (see docs/TRAINING.md).
"""
from __future__ import annotations

import math
from collections import Counter, deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .features import FRAME_FEATURES, frame_features, visible_keypoints

POSTURES = ["STANDING", "SITTING", "BENDING", "KNEELING", "LYING", "UNKNOWN"]


@dataclass
class PostureResult:
    label: str
    confidence: float          # rule: heuristic strength, model: class probability
    source: str                # "rules", "model" or "insufficient_keypoints"
    reliability: float         # fraction of keypoints visible (0..1)


def _nan(v: float) -> bool:
    return v is None or (isinstance(v, float) and math.isnan(v))


def rule_based_posture(f: dict[str, float]) -> tuple[str, float]:
    """Hand-written baseline. Thresholds are in degrees or body-size ratios.

    Order matters: lying is checked first (most safety relevant), then the
    upright postures.
    """
    torso, body = f["torso_angle"], f["body_angle"]
    hip, knee = f["hip_angle"], f["knee_angle"]
    aspect = f["aspect_ratio"]
    thigh = f["thigh_ratio"]

    # LYING: torso (or whole body) close to horizontal, or a very wide box with no torso info
    if (not _nan(torso) and torso >= 60) or (not _nan(body) and body >= 65):
        return "LYING", 0.8
    if _nan(torso) and aspect >= 1.6:
        return "LYING", 0.5
    if _nan(torso):
        return "UNKNOWN", 0.0

    # BENDING: torso clearly tilted forward but legs mostly straight
    if 35 <= torso < 60 and (_nan(knee) or knee >= 140):
        return "BENDING", 0.6

    # KNEELING: knees strongly bent, shins close to horizontal (ankle near knee height)
    if not _nan(knee) and knee < 120 and not _nan(f["dy_hip_ankle"]) and not _nan(f["dy_hip_knee"]):
        if f["dy_hip_ankle"] - f["dy_hip_knee"] < 0.06 and torso < 35:
            return "KNEELING", 0.5

    # SITTING: hips bent, or thighs strongly foreshortened (pointing at camera)
    if torso < 35 and ((not _nan(hip) and hip < 130) or (not _nan(thigh) and thigh < 0.45)):
        return "SITTING", 0.6

    # STANDING: upright torso with straight legs
    if torso < 30 and (_nan(hip) or hip >= 150) and (_nan(knee) or knee >= 150):
        return "STANDING", 0.7
    if torso < 30:
        return "STANDING", 0.4
    return "UNKNOWN", 0.0


class TrainedPostureModel:
    """Wraps a scikit-learn model saved by training/train_posture_classifier.py."""

    def __init__(self, path: str | Path):
        import joblib
        path = Path(path)
        if not path.exists():
            raise FileNotFoundError(path)
        bundle = joblib.load(path)
        if bundle.get("feature_names") != FRAME_FEATURES:
            raise ValueError(
                f"{path} was trained with different features. Re-run training/train_posture_classifier.py")
        self.model = bundle["model"]
        self.classes: list[str] = list(bundle["classes"])
        self.meta = bundle.get("meta", {})

    def predict(self, features: dict[str, float]) -> tuple[str, float, np.ndarray]:
        x = np.array([[features[k] for k in FRAME_FEATURES]], dtype=np.float64)
        proba = self.model.predict_proba(x)[0]
        i = int(np.argmax(proba))
        return self.classes[i], float(proba[i]), proba


class PostureClassifier:
    """Per-frame posture with UNKNOWN handling and per-track smoothing."""

    def __init__(self, model_path: str | Path | None = None, use_model: bool = True,
                 kpt_thr: float = 0.35, min_visible: int = 7, unknown_threshold: float = 0.5,
                 smoothing_window: int = 5):
        self.kpt_thr = kpt_thr
        self.min_visible = int(min_visible)
        self.unknown_threshold = float(unknown_threshold)
        self.smoothing_window = int(smoothing_window)
        self.model: TrainedPostureModel | None = None
        self.model_error: str | None = None
        if use_model and model_path:
            try:
                self.model = TrainedPostureModel(model_path)
            except Exception as exc:  # fall back to rules, but say why
                self.model_error = f"{type(exc).__name__}: {exc}"
        self._history: dict[int, deque] = {}
        self.offline_classes: list[str] = []   # classes of a batch model used offline

    @property
    def mode(self) -> str:
        return "model" if self.model else "rules"

    def classify_frame(self, kpts: np.ndarray, box: np.ndarray) -> tuple[PostureResult, dict[str, float]]:
        feats = frame_features(kpts, box, self.kpt_thr)
        return self.classify_features(feats, visible_keypoints(kpts, self.kpt_thr)), feats

    def classify_features(self, feats: dict[str, float], n_vis: int, n_total: int = 17,
                          model_output: tuple[str, float] | None = None) -> PostureResult:
        """Classify from precomputed frame features.

        ``model_output`` lets offline evaluation pass a batch-predicted
        (label, probability) so the same decision logic is applied without
        calling the model once per frame.
        """
        reliability = n_vis / float(n_total)
        if n_vis < self.min_visible:
            return PostureResult("UNKNOWN", 0.0, "insufficient_keypoints", reliability)
        rule_label, rule_conf = rule_based_posture(feats)
        if self.model is None and model_output is None:
            return PostureResult(rule_label, rule_conf, "rules", reliability)
        if model_output is not None:
            label, conf = model_output
        else:
            label, conf, _ = self.model.predict(feats)
        # The trained model knows only some classes; let the rules add BENDING/KNEELING
        # when the model is unsure and the rules are confident about a class the model lacks.
        known_classes = self.model.classes if self.model is not None else self.offline_classes
        if rule_label not in known_classes and rule_label != "UNKNOWN" and conf < 0.7 \
                and rule_conf >= 0.6:
            return PostureResult(rule_label, rule_conf, "rules", reliability)
        if conf < self.unknown_threshold:
            return PostureResult("UNKNOWN", conf, "model", reliability)
        return PostureResult(label, conf, "model", reliability)

    def classify(self, track_id: int, kpts: np.ndarray, box: np.ndarray) -> tuple[PostureResult, dict[str, float]]:
        """Frame classification followed by a majority vote over recent frames."""
        res, feats = self.classify_frame(kpts, box)
        return self.smooth(track_id, res), feats

    def smooth(self, track_id: int, res: PostureResult) -> PostureResult:
        hist = self._history.setdefault(track_id, deque(maxlen=self.smoothing_window))
        hist.append(res.label)
        known = [l for l in hist if l != "UNKNOWN"]
        if res.label != "UNKNOWN" and known:
            voted, _ = Counter(known).most_common(1)[0]
            if voted != res.label:
                res = PostureResult(voted, res.confidence, res.source + "+smoothed", res.reliability)
        return res

    def forget(self, track_id: int) -> None:
        self._history.pop(track_id, None)
