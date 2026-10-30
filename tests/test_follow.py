"""Person-following: controller geometry, bounded acceleration, camera obstacle detection,
the follow state machine, and the FollowManager driving a simulated robot."""
import time
from types import SimpleNamespace

import numpy as np
import pytest

from robot.follow import (FollowController, FollowManager, FollowParams, ObstacleDetector, ObstacleParams,
                          RampLimiter, target_quality)
from robot.robot_logic import RobotLink
from robot.sim_robot import SimRobotServer
from robot.wifi_client import RobotClient
from src.detection.yolo_onnx import ObjectDetection
from src.pipeline import PersonState
from src.safety.robot_state_machine import FollowInputs, FollowState, FollowStateMachine

W, H = 1280, 720
S = FollowState


def person(tid, cx, height_ratio, state="NORMAL", posture="STANDING", score=0.9, confirmed=True, rel=1.0):
    h = height_ratio * H
    w = h * 0.4
    box = np.array([cx - w / 2, H - h - 10, cx + w / 2, H - 10], dtype=np.float32)
    return PersonState(tid, box, np.zeros((17, 3), np.float32), score, posture, 0.9, "model", rel, state, 0.0,
                       confirmed)


def obj(name, x1, y1, x2, y2):
    return ObjectDetection(np.array([x1, y1, x2, y2], dtype=np.float32), 0.8, 0, name)


# ------------------------------------------------------------ controller
def compute(p, **kw):
    return FollowController(FollowParams(**kw)).compute(W, H, p)


def test_centred_far_person_drives_straight():
    d = compute(person(1, W / 2, 0.3))
    assert d.target_id == 1 and d.left == d.right > 0 and d.left <= 35


def test_person_on_right_turns_right_and_far_off_centre_spins():
    d = compute(person(1, W * 0.7, 0.3))
    assert d.left > d.right
    d = compute(person(1, W * 0.95, 0.3))
    assert d.left == -d.right and d.left > 0
    d = compute(person(1, W * 0.05, 0.3))
    assert d.left == -d.right and d.left < 0


def test_too_close_and_near_follow_distance_mean_no_forward_and_never_reverse():
    d = compute(person(1, W / 2, 0.85))
    assert (d.left, d.right) == (0, 0) and "too close" in d.reason
    d = compute(person(1, W / 2, 0.54))
    assert (d.left, d.right) == (0, 0) and "estimate" in d.reason     # never claims a measured distance
    for hr in np.linspace(0.05, 0.99, 30):
        d = compute(person(1, W / 2, float(hr)))
        assert d.left + d.right >= 0


def test_minimum_speed_and_cap():
    assert compute(person(1, W / 2, 0.48), min_speed_pct=18, max_speed_pct=30).left == 18
    assert compute(person(1, W / 2, 0.05), min_speed_pct=18, max_speed_pct=30).left == 30


def test_obstacle_flag_removes_forward_but_allows_turning():
    d = FollowController(FollowParams()).compute(W, H, person(1, W * 0.75, 0.3), obstacle=True)
    assert d.left == -d.right and d.left > 0


def test_target_quality():
    p = FollowParams()
    assert target_quality(person(1, W / 2, 0.3), p)[0]
    assert not target_quality(person(1, W / 2, 0.3, score=0.4), p)[0]
    assert not target_quality(person(1, W / 2, 0.3, confirmed=False), p)[0]
    assert not target_quality(person(1, W / 2, 0.3, rel=0.1), p)[0]


# ------------------------------------------------------------------ ramp
def test_ramp_limits_acceleration_but_never_delays_stopping():
    r = RampLimiter(60)
    assert r.apply(0.0, 30, 30) == (6, 6)            # first step: 0.1 s assumed
    assert r.apply(0.1, 30, 30) == (12, 12)
    assert r.apply(0.2, 30, 30) == (18, 18)
    assert r.apply(0.3, 5, 5) == (5, 5)              # slowing down: immediate
    assert r.apply(0.4, 0, 0) == (0, 0)              # stopping: immediate
    assert r.apply(0.5, 20, -20) == (6, -6)
    assert r.apply(0.6, -20, 20) == (0, 0)           # reversing passes through zero
    assert r.apply(5.0, 30, 30)[0] <= 30             # long gap capped at 0.5 s
    assert r.apply(5.1, 30, 30)[0] <= 30


# ------------------------------------------------------------- obstacles
def test_obstacle_in_corridor_is_reported_repeated_then_cleared():
    det = ObstacleDetector(ObstacleParams(clear_frames=3, report_interval_s=0.5))
    chair = obj("chair", W * 0.4, H * 0.6, W * 0.6, H * 0.95)
    tgt = [person(1, W / 2, 0.3)]
    ob = det.update(0.0, W, H, [chair], tgt, 1)
    assert ob.seen and ob.send_report and "chair" in ob.reason
    assert not det.update(0.2, W, H, [chair], tgt, 1).send_report
    assert det.update(0.6, W, H, [chair], tgt, 1).send_report
    clears = [det.update(0.7 + i * 0.1, W, H, [], tgt, 1) for i in range(3)]
    assert [o.send_clear for o in clears] == [False, False, True]
    assert not det.update(1.2, W, H, [], tgt, 1).seen


def test_objects_outside_corridor_far_or_tiny_are_ignored():
    det = ObstacleDetector(ObstacleParams())
    side = obj("chair", 0, H * 0.6, W * 0.2, H * 0.95)
    far = obj("couch", W * 0.4, H * 0.3, W * 0.6, H * 0.5)
    tiny = obj("cup", W * 0.49, H * 0.9, W * 0.51, H * 0.92)
    assert not det.update(0.0, W, H, [side, far, tiny], [person(1, W / 2, 0.3)], 1).seen


def test_other_person_in_path_is_an_obstacle_but_target_and_its_duplicates_are_not():
    det = ObstacleDetector(ObstacleParams())
    target = person(1, W / 2, 0.75)
    dup = obj("dog", target.box[0] + 5, target.box[1] + 40, target.box[2] - 5, target.box[3] - 2)
    assert not det.update(0.0, W, H, [dup], [target], 1).seen
    other = person(2, W / 2, 0.75)
    other.box[3] = H * 0.98
    ob = ObstacleDetector(ObstacleParams()).update(0.0, W, H, [], [person(1, W / 2, 0.3), other], 1)
    assert ob.seen and "person" in ob.reason


def test_follow_params_validation():
    for bad in ({"max_speed_pct": 0}, {"deadband": 0.5, "spin_threshold": 0.4}, {"command_ms": 5000},
                {"obstacle": {"corridor_width": 2}}, {"min_detection_score": 0.3}, {"max_accel_pct_per_s": 0}):
        with pytest.raises(ValueError):
            FollowParams.from_config(bad)
    p = FollowParams.from_config({"enabled": True, "max_speed_pct": 30, "obstacle": {"clear_frames": 2}})
    assert p.max_speed_pct == 30 and p.obstacle.clear_frames == 2


# --------------------------------------------------------- state machine
GOOD = FollowInputs(target_visible=True, target_confident=True)
GONE = FollowInputs()


def sm(**kw):
    m = FollowStateMachine(lost_grace_s=0.7, uncertain_max_s=3.0, obstacle_clear_s=1.0, **kw)
    assert m.select_target(7, 0.0)[0]
    return m


def test_select_then_follow_then_uncertain_then_back():
    m = sm()
    assert m.state == S.FOLLOW_READY and m.robot_follow_mode and not m.motion_allowed
    assert m.step(0.1, GOOD) == S.FOLLOWING and m.motion_allowed
    assert m.step(0.2, GONE) == S.TARGET_UNCERTAIN and not m.motion_allowed and m.robot_follow_mode
    assert m.step(0.3, GOOD) == S.FOLLOWING
    weak = FollowInputs(target_visible=True, target_confident=False)
    assert m.step(0.4, weak) == S.TARGET_UNCERTAIN


def test_lost_after_grace_and_never_switches_target():
    m = sm()
    m.step(0.1, GOOD)
    m.step(0.2, GONE)
    assert m.step(0.7, GONE) == S.TARGET_UNCERTAIN
    assert m.step(0.9, GONE) == S.TARGET_LOST and not m.robot_follow_mode
    assert m.step(1.0, GOOD) == S.TARGET_LOST          # stays lost: the operator must select again
    assert m.target_id == 7
    assert m.select_target(9, 2.0)[0] and m.target_id == 9


def test_uncertain_too_long_becomes_lost():
    m = sm()
    m.step(0.1, GOOD)
    weak = FollowInputs(target_visible=True, target_confident=False)
    m.step(0.2, weak)
    assert m.step(3.0, weak) == S.TARGET_UNCERTAIN
    assert m.step(3.3, weak) == S.TARGET_LOST


def test_obstacle_stop_then_operator_resume():
    m = sm()
    m.step(0.1, GOOD)
    obs = FollowInputs(target_visible=True, target_confident=True, obstacle=True)
    assert m.step(0.2, obs) == S.OBSTACLE_STOP and not m.motion_allowed
    assert m.step(0.5, GOOD) == S.OBSTACLE_STOP          # must be clear for 1 s
    assert m.step(1.6, GOOD) == S.FOLLOW_READY
    assert m.step(1.7, GOOD) == S.FOLLOW_READY           # waits for the operator
    assert m.resume(1.8)[0]
    assert m.step(1.9, GOOD) == S.FOLLOWING


def test_obstacle_auto_resume_option():
    m = sm(obstacle_auto_resume=True)
    m.step(0.1, GOOD)
    m.step(0.2, FollowInputs(target_visible=True, target_confident=True, obstacle=True))
    m.step(0.3, GOOD)
    m.step(1.4, GOOD)
    assert m.step(1.5, GOOD) == S.FOLLOWING


def test_emergency_comm_and_fall_end_following():
    m = sm()
    m.step(0.1, GOOD)
    assert m.step(0.2, FollowInputs(target_visible=True, target_confident=True, emergency=True)) == S.EMERGENCY_STOP
    assert m.step(0.3, GOOD) == S.FOLLOW_DISABLED and m.target_id is None   # cleared: select again
    m = sm()
    m.step(0.1, GOOD)
    assert m.step(0.2, FollowInputs(target_visible=True, target_confident=True, comm_ok=False)) == S.FOLLOW_DISABLED
    m = sm()
    m.step(0.1, GOOD)
    assert m.step(0.2, FollowInputs(target_visible=True, target_confident=True, target_down=True)) == S.FOLLOW_DISABLED
    m = sm()
    m.step(0.1, GOOD)
    assert m.step(0.2, FollowInputs(target_visible=True, target_confident=True, fall_response=True)) == S.FOLLOW_DISABLED
    assert "fall" in m.reason
    m = FollowStateMachine()
    m.step(0.0, FollowInputs(emergency=True))
    m.state = S.EMERGENCY_STOP
    assert not m.select_target(1, 0.1)[0]


def test_transitions_are_recorded():
    m = sm()
    m.step(0.1, GOOD)
    m.disable(0.2)
    assert [t[2] for t in m.transitions] == ["FOLLOW_READY", "FOLLOWING", "FOLLOW_DISABLED"]
    assert m.transition_count == 3


# --------------------------------------------------- with a simulated robot
def _wait(cond, timeout=2.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return
        time.sleep(0.01)
    raise AssertionError("condition not reached in time")


@pytest.fixture
def rig():
    sim = SimRobotServer(port=0, motors_enabled=True).start()
    client = RobotClient("127.0.0.1", sim.port)
    events = []
    logger = SimpleNamespace(log=lambda et, **kw: events.append((et, kw)))
    link = RobotLink(client, max_speed_pct=40, event_logger=logger).start()
    assert client.wait_connected(3) and client.wait_status(3)
    pipe = SimpleNamespace(object_every_n=10)
    mgr = FollowManager(link, FollowParams(max_accel_pct_per_s=1000), pipeline=pipe, event_logger=logger)
    yield sim, client, link, mgr, pipe, events
    link.close()
    sim.stop()


def frame(persons, objects=()):
    return SimpleNamespace(persons=list(persons), objects=list(objects))


def feed(mgr, persons, objects=(), n=1, dt=0.05, age=0.0):
    for _ in range(n):
        mgr.on_frame(frame(persons, objects), (W, H), frame_age_s=age)
        time.sleep(dt)


def test_operator_selects_target_and_robot_follows_only_that_person(rig):
    sim, client, link, mgr, pipe, events = rig
    p1, p2 = person(1, W * 0.7, 0.3), person(2, W * 0.3, 0.3)
    assert not mgr.select(5, [p1, p2])[0]                       # not visible
    assert not mgr.select(2, [person(2, W / 2, 0.3, posture="LYING")])[0]
    ok, msg = mgr.select(1, [p1, p2])
    assert ok, msg
    assert sim.core.follow and sim.core.follow_target == 1 and pipe.object_every_n == 2
    feed(mgr, [p1, p2], n=3)
    _wait(lambda: sim.core.state == "MOVING")
    assert sim.core.left > sim.core.right > 0 and mgr.sm.state == S.FOLLOWING
    # the target disappears, the other person stays: never switch
    feed(mgr, [p2], n=2)
    assert mgr.sm.state == S.TARGET_UNCERTAIN and mgr.sm.target_id == 1
    _wait(lambda: sim.core.left == 0 and sim.core.right == 0)
    time.sleep(0.75)
    feed(mgr, [p2], n=2)
    assert mgr.sm.state == S.TARGET_LOST
    _wait(lambda: not sim.core.follow)                          # robot left follow mode
    assert any(et == "FOLLOW_STATE" and "TARGET_LOST" in kw["reason"] for et, kw in events)


def test_obstacle_stops_following_until_operator_resumes(rig):
    sim, client, link, mgr, pipe, events = rig
    p = person(1, W / 2, 0.3)
    assert mgr.select(1, [p])[0]
    feed(mgr, [p], n=2)
    _wait(lambda: sim.core.state == "MOVING")
    chair = obj("chair", W * 0.4, H * 0.6, W * 0.6, H * 0.95)
    feed(mgr, [p], [chair], n=2)
    assert mgr.sm.state == S.OBSTACLE_STOP
    _wait(lambda: sim.core.obstacle_active and sim.core.left == 0)
    for _ in range(30):                                           # path clear for > 1 s
        feed(mgr, [p], dt=0.05)
        if mgr.sm.state == S.FOLLOW_READY:
            break
    assert mgr.sm.state == S.FOLLOW_READY
    _wait(lambda: not sim.core.obstacle_active)
    feed(mgr, [p], n=3)
    assert sim.core.left == 0 and mgr.sm.state == S.FOLLOW_READY  # no automatic resume
    assert mgr.resume()[0]
    feed(mgr, [p], n=2)
    _wait(lambda: sim.core.state == "MOVING")


def test_stale_frames_never_drive(rig):
    sim, client, link, mgr, pipe, events = rig
    p = person(1, W / 2, 0.3)
    assert mgr.select(1, [p])[0]
    feed(mgr, [p], n=3, age=2.0)                                # 2 s old pictures
    time.sleep(0.2)
    assert sim.core.state != "MOVING"
    assert link.manager.stats["dropped_stale"] >= 1


def test_fall_alert_and_emergency_end_following(rig):
    sim, client, link, mgr, pipe, events = rig
    p = person(1, W / 2, 0.3)
    assert mgr.select(1, [p])[0]
    feed(mgr, [p], n=2)
    _wait(lambda: sim.core.state == "MOVING")
    link.on_alert(SimpleNamespace(severity="warning", message="suspected", alert_id="w1", track_id=1))
    assert mgr.sm.state == S.FOLLOW_DISABLED and "fall" in mgr.sm.reason
    _wait(lambda: not sim.core.follow and sim.core.left == 0)
    assert pipe.object_every_n == 10
    link.set_fall_flags(suspected=True, confirmed=False)
    ok, msg = mgr.select(1, [p])
    assert not ok and "fall" in msg
    link.set_fall_flags(False, False)
    assert mgr.select(1, [p])[0]
    link.on_alert(SimpleNamespace(severity="emergency", message="fall", alert_id="a1", track_id=1))
    _wait(lambda: sim.core.fall_alert and not sim.core.follow and sim.core.left == 0)   # FALL_DETECTED
    assert not mgr.active
    ok, msg = mgr.select(1, [p])
    assert not ok and "fall" in msg.lower()
    link.emergency("operator")                                 # the operator E-STOP is still a latch
    _wait(lambda: sim.core.state == "EMERGENCY")
    client.send("status_request")
    client.wait_status(1)
    ok, msg = mgr.select(1, [p])
    assert not ok and "emergency" in msg.lower()


def test_target_falling_ends_following(rig):
    sim, client, link, mgr, pipe, events = rig
    assert mgr.select(1, [person(1, W / 2, 0.3)])[0]
    feed(mgr, [person(1, W / 2, 0.3)], n=2)
    feed(mgr, [person(1, W / 2, 0.3, state="POSSIBLE_FALL")])
    assert mgr.sm.state == S.FOLLOW_DISABLED
    _wait(lambda: not sim.core.follow)


def test_manual_is_refused_while_following(rig):
    sim, client, link, mgr, pipe, events = rig
    assert mgr.select(1, [person(1, W / 2, 0.3)])[0]
    d = link.manual("forward")
    assert not d.sent and "following" in d.reason
    link.stop()
    assert not mgr.active and link.manual("forward").sent
