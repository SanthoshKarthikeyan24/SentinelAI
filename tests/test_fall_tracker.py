import numpy as np

from src.analysis.fall_detector import (FALL_CONFIRMED, FALL_SUSPECTED, NORMAL, POSSIBLE_FALL,
                                        FallDetector, FallParams)
from src.analysis.posture import PostureClassifier
from src.detection.yolo_onnx import PersonDetection
from src.tracking.tracker import PersonTracker, iou_matrix
from synthetic import fall_sequence, lie_down_slowly, skeleton


def run(seq, fd=None, track_id=1, params=None):
    """Feed a synthetic sequence through tracker -> posture rules -> fall detector."""
    fd = fd or FallDetector(params or FallParams(), model=None)
    tr = PersonTracker(min_hits=2)
    pc = PostureClassifier(None, use_model=False)
    events, states = [], []
    for t, k, b in seq:
        tracks = tr.update([PersonDetection(b, 0.9, k)], t, (1280, 720))
        for trk in tracks:
            res, feats = pc.classify(trk.track_id, trk.keypoints, trk.box)
            if trk.confirmed:
                st, evs = fd.update(trk.track_id, t, list(trk.history), res.label, feats)
                events += evs
                states.append(st.state)
    return events, states, fd


def test_fast_fall_is_confirmed_once():
    events, states, _ = run(fall_sequence())
    types = [e["type"] for e in events]
    assert types.count("POSSIBLE_FALL") == 1
    assert types.count("FALL_SUSPECTED") == 1
    assert types.count("FALL_CONFIRMED") == 1
    assert types.index("POSSIBLE_FALL") < types.index("FALL_SUSPECTED") < types.index("FALL_CONFIRMED")
    confirmed = [e for e in events if e["type"] == "FALL_CONFIRMED"][0]
    assert 2.0 <= confirmed["t"] <= 2.6 + 0.4 + 1.5 + 0.8


def test_slow_lying_down_is_not_confirmed():
    events, _, _ = run(lie_down_slowly())
    assert "FALL_CONFIRMED" not in [e["type"] for e in events]


def test_recovery_after_confirmed_fall():
    seq = fall_sequence(down_s=3.0)
    t_end = seq[-1][0]
    seq += [(t_end + (i + 1) / 15, *skeleton(380, 400, 300, 0)) for i in range(40)]  # stands up
    events, states, _ = run(seq)
    types = [e["type"] for e in events]
    assert "FALL_CONFIRMED" in types and "RECOVERED" in types
    assert states[-1] == NORMAL


def test_cooldown_prevents_immediate_retrigger():
    p = FallParams(cooldown_s=30)
    seq = fall_sequence(down_s=3.0)
    t = seq[-1][0]
    seq += [(t + (i + 1) / 15, *skeleton(380, 400, 300, 0)) for i in range(40)]
    seq += fall_sequence(start_t=seq[-1][0] + 1 / 15)
    events, _, _ = run(seq, params=p)
    assert [e["type"] for e in events].count("FALL_CONFIRMED") == 1


def test_track_lost_during_suspected_fall_warns_but_does_not_confirm():
    fd = FallDetector(FallParams(confirm_down_s=10.0), model=None)
    seq = fall_sequence(down_s=1.0)
    events, states, fd = run(seq, fd=fd)
    assert states[-1] == FALL_SUSPECTED
    lost = fd.track_lost(1, seq[-1][0] + 2)
    assert lost and lost[0]["type"] == "TRACK_LOST_DURING_FALL"
    assert "FALL_CONFIRMED" not in [e["type"] for e in events]


def test_stream_interruption_discards_possible_fall():
    fd = FallDetector(FallParams(), model=None)
    from src.analysis.fall_detector import TrackFallState
    fd.tracks[1] = TrackFallState(1, state=POSSIBLE_FALL)
    fd.tracks[2] = TrackFallState(2, state=FALL_CONFIRMED)
    fd.stream_interrupted()
    assert fd.tracks[1].state == NORMAL
    assert fd.tracks[2].state == FALL_CONFIRMED   # a confirmed emergency is never auto-cleared


def test_operator_reset():
    events, states, fd = run(fall_sequence())
    assert states[-1] == FALL_CONFIRMED
    fd.reset(None, 100.0)
    assert fd.tracks[1].state == NORMAL


def test_iou_matrix():
    a = np.array([[0, 0, 10, 10]], dtype=np.float32)
    b = np.array([[0, 0, 10, 10], [5, 5, 15, 15], [20, 20, 30, 30]], dtype=np.float32)
    m = iou_matrix(a, b)
    assert m.shape == (1, 3)
    assert abs(m[0, 0] - 1) < 1e-6 and abs(m[0, 1] - 25 / 175) < 1e-6 and m[0, 2] == 0


def _det(x):
    k, b = skeleton(x, 400, 300, 0)
    return PersonDetection(b, 0.9, k)


def test_tracker_keeps_ids_for_two_people():
    tr = PersonTracker(min_hits=1)
    ids = []
    for i in range(20):
        tracks = tr.update([_det(200 + i * 3), _det(900 - i * 3)], i / 15, (1280, 720))
        ids.append(sorted((round(float(t.box[0])), t.track_id) for t in tracks))
    first = [tid for _, tid in ids[0]]
    last = [tid for _, tid in ids[-1]]
    assert first == last and len(set(first)) == 2


def test_tracker_survives_brief_miss_and_removes_stale_tracks():
    tr = PersonTracker(min_hits=1, max_missed_s=0.5)
    tr.update([_det(300)], 0.0, (1280, 720))
    tr.update([], 0.2, (1280, 720))                     # missed briefly
    tracks = tr.update([_det(305)], 0.3, (1280, 720))
    assert tracks[0].track_id == 1
    tr.update([], 0.5, (1280, 720))
    tr.update([], 1.0, (1280, 720))                     # 0.7 s missing > 0.5 s
    assert 1 in tr.removed_ids and 1 not in tr.tracks


def test_tracker_follows_fast_shape_change_during_fall():
    tr = PersonTracker(min_hits=1)
    ids = set()
    for t, k, b in fall_sequence():
        for trk in tr.update([PersonDetection(b, 0.9, k)], t, (1280, 720)):
            ids.add(trk.track_id)
    assert ids == {1}


def _feed(fd, pc, tid, seq, hist):
    out = []
    for t, k, b in seq:
        from src.tracking.tracker import Observation
        hist.setdefault(tid, []).append(Observation(t, b, k, 0.9, (1280, 720)))
        res, feats = pc.classify(tid, k, b)
        _, evs = fd.update(tid, t, hist[tid], res.label, feats)
        out += evs
        out += fd.resolve_handover(t, {tid: b})
    return out


def test_fall_state_moves_to_new_track_id_after_id_switch():
    fd = FallDetector(FallParams(), model=None)
    pc = PostureClassifier(None, use_model=False)
    seq = fall_sequence(down_s=0.6)              # fall, then tracker loses track 1
    hist = {}
    ev1 = _feed(fd, pc, 1, seq, hist)
    assert fd.tracks[1].state == FALL_SUSPECTED
    t0 = seq[-1][0]
    lying = [(t0 + 0.3 + i / 15, *skeleton(380, 535, 300, 88)) for i in range(40)]
    ev2 = _feed(fd, pc, 2, lying, hist)          # same person, new id, still lying
    types = [e["type"] for e in ev1 + ev2]
    assert "TRACK_HANDOVER" in types
    assert "FALL_CONFIRMED" in types
    assert fd.tracks[2].state == FALL_CONFIRMED and 1 not in fd.tracks


def test_upright_newcomer_does_not_inherit_fall():
    fd = FallDetector(FallParams(), model=None)
    pc = PostureClassifier(None, use_model=False)
    seq = fall_sequence(down_s=0.6)
    hist = {}
    _feed(fd, pc, 1, seq, hist)
    t0 = seq[-1][0]
    helper = [(t0 + 0.3 + i / 15, *skeleton(420, 400, 300, 0)) for i in range(30)]   # standing person
    ev = _feed(fd, pc, 2, helper, hist)
    assert "TRACK_HANDOVER" not in [e["type"] for e in ev]
    assert fd.tracks[2].state == NORMAL
