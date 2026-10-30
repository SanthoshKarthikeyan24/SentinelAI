from prepare_dataset import parse_gmdcsa_classes
from dataset_lib import fall_window_label, interval_label_at, match_events, summarise_matches


def test_parse_normal_and_malformed_annotations():
    r = parse_gmdcsa_classes("Falling (SW)[3.4 to 6]; Sitting[0 to 3.4]")
    assert [(x[0], x[2], x[3]) for x in r] == [("FALL", 3.4, 6.0), ("SITTING", 0.0, 3.4)]
    r = parse_gmdcsa_classes("Falling (BW)[2.3 6]; Sitting[0 to 2]")       # missing 'to'
    assert r[0][:4] == ("FALL", "Falling (BW)", 2.3, 6.0) and "without 'to'" in r[0][4]
    r = parse_gmdcsa_classes("Standing[2.4 to 3.6; 6.6 to 7.9; Walking[3.6 to 5.2]")  # missing ']'
    assert ("WALKING", 3.6, 5.2) in [(x[0], x[2], x[3]) for x in r]
    assert parse_gmdcsa_classes("Falling (FW); Sitting[0 to 1.7]")[0][0] == "SITTING"  # no fall interval
    assert parse_gmdcsa_classes("Dancing[0 to 3]")[0][0].startswith("UNRECOGNISED")


EV = [{"label": "SITTING", "start_s": 0.0, "end_s": 3.0},
      {"label": "FALL", "start_s": 3.4, "end_s": 6.0},
      {"label": "ACT_EXERCISING", "start_s": 7.0, "end_s": 9.0},
      {"label": "STANDING", "start_s": 7.0, "end_s": 9.0}]


def test_posture_labels_from_intervals():
    assert interval_label_at(EV, 1.0) == "SITTING"
    assert interval_label_at(EV, 2.95) is None         # too close to the interval edge
    assert interval_label_at(EV, 3.2) is None          # unlabelled gap
    assert interval_label_at(EV, 4.0) is None          # inside a fall
    assert interval_label_at(EV, 8.0) is None          # exercising excluded


def test_fall_window_labels():
    assert fall_window_label(EV, 1.0) == 0
    assert fall_window_label(EV, 3.2) is None          # boundary
    assert fall_window_label(EV, 4.0) == 1
    assert fall_window_label(EV, 5.5) is None          # already down: ignored
    assert fall_window_label(EV, 8.0) == 0


def test_event_matching():
    alarms = [{"type": "FALL_SUSPECTED", "t": 4.0}, {"type": "FALL_SUSPECTED", "t": 4.5},
              {"type": "FALL_SUSPECTED", "t": 20.0}, {"type": "FALL_CONFIRMED", "t": 5.0}]
    m = match_events(alarms, [(3.4, 6.0), (30.0, 33.0)], "FALL_SUSPECTED")
    assert m["tp"] == 1 and m["fn"] == 1 and m["fp"] == 1 and m["duplicates"] == 1
    assert abs(m["delays"][0] - 0.6) < 1e-9
    s = summarise_matches([m], 3600)
    assert s["precision"] == 0.5 and s["recall"] == 0.5 and s["false_alarms_per_hour"] == 1.0
