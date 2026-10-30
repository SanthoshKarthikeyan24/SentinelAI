import math

import numpy as np

from src.analysis.features import FRAME_FEATURES, WINDOW_FEATURES, frame_features, window_features
from src.analysis.posture import PostureClassifier, rule_based_posture
from synthetic import fall_sequence, skeleton


def test_upright_torso_angle_near_zero():
    k, b = skeleton(320, 400, 300, 0)
    f = frame_features(k, b)
    assert f["torso_angle"] < 5
    assert f["hip_angle"] > 170 and f["knee_angle"] > 170


def test_lying_torso_angle_near_ninety():
    k, b = skeleton(320, 400, 300, 90)
    f = frame_features(k, b)
    assert 80 < f["torso_angle"] < 100
    assert f["aspect_ratio"] > 1.5


def test_features_are_scale_invariant():
    k1, b1 = skeleton(320, 400, 300, 30)
    k2, b2 = k1.copy(), b1 * 2
    k2[:, :2] *= 2
    f1, f2 = frame_features(k1, b1), frame_features(k2, b2)
    for name in FRAME_FEATURES:
        a, c = f1[name], f2[name]
        assert (math.isnan(a) and math.isnan(c)) or abs(a - c) < 1e-6, name


def test_missing_keypoints_become_nan_not_zero():
    k, b = skeleton(320, 400, 300, 0, missing=(5, 6))   # both shoulders missing
    f = frame_features(k, b)
    assert math.isnan(f["torso_angle"])
    assert math.isnan(f["dy_shoulder_hip"])
    assert f["visible_frac"] == 15 / 17


def test_rule_baseline_postures():
    stand = frame_features(*skeleton(320, 400, 300, 0))
    lie = frame_features(*skeleton(320, 400, 300, 85))
    sit = frame_features(*skeleton(320, 400, 300, 0, "sitting"))
    bend = frame_features(*skeleton(320, 400, 300, 0))
    bend["torso_angle"] = 45.0
    assert rule_based_posture(stand)[0] == "STANDING"
    assert rule_based_posture(lie)[0] == "LYING"
    assert rule_based_posture(sit)[0] == "SITTING"
    assert rule_based_posture(bend)[0] == "BENDING"


def test_unknown_when_too_few_keypoints():
    k, b = skeleton(320, 400, 300, 0, missing=tuple(range(0, 12)))
    pc = PostureClassifier(None, use_model=False, min_visible=7)
    res, _ = pc.classify_frame(k, b)
    assert res.label == "UNKNOWN" and res.source == "insufficient_keypoints"


def test_smoothing_majority_vote():
    pc = PostureClassifier(None, use_model=False, smoothing_window=5)
    for _ in range(4):
        pc.classify(1, *skeleton(320, 400, 300, 0))
    res, _ = pc.classify(1, *skeleton(320, 400, 300, 0, "sitting"))
    assert res.label == "STANDING"            # one odd frame is smoothed away
    pc.forget(1)
    res, _ = pc.classify(1, *skeleton(320, 400, 300, 0, "sitting"))
    assert res.label == "SITTING"


def test_window_features_detect_fast_drop():
    seq = fall_sequence()
    # window ending just after the fall motion
    end = 2.0 + 0.6 + 0.2
    win = [s for s in seq if end - 1.5 <= s[0] <= end]
    wf = window_features([s[0] for s in win], [s[2] for s in win], [s[1] for s in win], 1.5)
    assert set(wf) == set(WINDOW_FEATURES)
    assert wf["max_hip_speed"] > 0.5
    assert wf["torso_angle_delta"] > 60
    # a still standing window has no speed
    win = [s for s in seq if s[0] <= 1.5]
    wf2 = window_features([s[0] for s in win], [s[2] for s in win], [s[1] for s in win], 1.5)
    assert abs(wf2["max_hip_speed"]) < 0.05


def test_window_features_empty_input():
    wf = window_features([], [], [], 1.5)
    assert wf["n_obs"] == 0 and wf["coverage"] == 0.0
    assert np.isnan(wf["hip_drop"])
