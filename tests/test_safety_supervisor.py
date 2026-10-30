"""Central safety supervisor: priority order, combined permissions, latches."""
import itertools

import pytest

from src.safety.safety_supervisor import Level, SafetyInputs, SafetySupervisor

HEALTHY = dict(robot_connected=True, status_fresh=True)


def ev(**kw):
    return SafetySupervisor().evaluate(SafetyInputs(**{**HEALTHY, **kw}))


def test_idle_allows_everything():
    d = ev()
    assert d.level == Level.IDLE and d.allow_follow and d.allow_manual and d.allow_forward


@pytest.mark.parametrize("inputs,level", [
    (dict(robot_emergency=True), Level.FIRMWARE_EMERGENCY),
    (dict(laptop_emergency=True), Level.EMERGENCY_STOP),
    (dict(robot_connected=False), Level.COMM_FAULT),
    (dict(status_fresh=False), Level.COMM_FAULT),
    (dict(watchdog_ok=False), Level.COMM_FAULT),
    (dict(comm_fault_reported=True), Level.COMM_FAULT),
    (dict(robot_obstacle=True), Level.OBSTACLE),
    (dict(camera_obstacle=True), Level.OBSTACLE),
    (dict(fall_suspected=True), Level.FALL_RESPONSE),
    (dict(fall_confirmed=True), Level.FALL_RESPONSE),
    (dict(follow_state="TARGET_LOST"), Level.TARGET_LOST),
    (dict(follow_state="TARGET_UNCERTAIN"), Level.TARGET_LOST),
    (dict(follow_state="FOLLOWING"), Level.FOLLOWING),
    (dict(manual_active=True), Level.MANUAL),
])
def test_each_condition_maps_to_its_level(inputs, level):
    assert ev(**inputs).level == level


def test_priority_order_is_respected_for_every_pair():
    """With any two conditions active, the higher-priority one is reported."""
    conds = [(dict(robot_emergency=True), 1), (dict(laptop_emergency=True), 2), (dict(robot_connected=False), 3),
             (dict(robot_obstacle=True), 4), (dict(fall_suspected=True), 5), (dict(follow_state="TARGET_LOST"), 6),
             (dict(follow_state="FOLLOWING"), 7), (dict(manual_active=True), 8)]
    for (a, la), (b, lb) in itertools.combinations(conds, 2):
        if "follow_state" in a and "follow_state" in b:
            continue
        d = ev(**a, **b)
        assert d.level == min(la, lb), (a, b, d)
        assert len(d.conditions) >= 2


def test_hard_stops_block_all_motion():
    for kw in (dict(robot_emergency=True), dict(laptop_emergency=True), dict(robot_connected=False),
               dict(status_fresh=False), dict(watchdog_ok=False)):
        d = ev(**kw)
        assert not (d.allow_follow or d.allow_manual or d.allow_forward), kw


def test_permissions_combine_all_active_conditions():
    d = ev(robot_obstacle=True)
    assert not d.allow_follow and d.allow_manual and not d.allow_forward     # reverse/turn still allowed
    d = ev(fall_suspected=True)
    assert not d.allow_follow and not d.allow_manual
    d = ev(robot_obstacle=True, fall_suspected=True)                         # top = obstacle, but fall blocks manual
    assert d.level == Level.OBSTACLE and not d.allow_manual
    d = ev(follow_state="FOLLOWING")
    assert d.allow_follow and not d.allow_manual                             # manual off while following


def test_check_motion():
    s = SafetySupervisor()
    inp = SafetyInputs(**HEALTHY, camera_obstacle=True)
    assert "forward" in s.check_motion(inp, "manual", 20, 20)
    assert s.check_motion(inp, "manual", -20, -20) == ""
    assert s.check_motion(inp, "manual", -20, 20) == ""                       # turn in place
    assert "following blocked" in s.check_motion(inp, "follow", 10, 10)
    assert s.check_motion(inp, "follow", 0, 0) == ""                          # stopping is always allowed
    inp = SafetyInputs(**HEALTHY, follow_state="FOLLOW_READY")
    assert "disable following" in s.check_motion(inp, "manual", 10, 10)


def test_comm_fault_latch_needs_operator_on_a_healthy_link():
    s = SafetySupervisor()
    assert s.evaluate(SafetyInputs(**HEALTHY, comm_fault_reported=True)).level == Level.COMM_FAULT
    # the link looks fine again, the robot no longer reports anything: still latched
    assert s.evaluate(SafetyInputs(**HEALTHY)).level == Level.COMM_FAULT
    ok, msg = s.acknowledge_comm_fault(SafetyInputs(robot_connected=False))
    assert not ok and s.comm_fault_latched
    ok, _ = s.acknowledge_comm_fault(SafetyInputs(**HEALTHY))
    assert ok and s.evaluate(SafetyInputs(**HEALTHY)).level == Level.IDLE


def test_decision_serialises():
    d = ev(robot_obstacle=True, manual_active=True).to_dict()
    assert d["level"] == 4 and d["level_name"] == "OBSTACLE" and len(d["conditions"]) == 2
