"""Protocol v3: laptop-side command building/validation, encoding and parsing (robot/commands.py),
and agreement between the laptop's validation and the robot's (robot/sim_robot.py, which
tests/test_core_parity.py proves identical to the firmware)."""
import json
import random

import pytest

from robot.commands import (MAX_LINE, PROTOCOL_VERSION, SCHEMA, CommandError, ProtocolError, encode, make_command,
                            parse_line)
from robot.sim_robot import SimRobotCore, parse_json_object

VALID = {
    "ping": {}, "status_request": {}, "stop": {}, "mute": {}, "obstacle_clear": {},
    "motor": dict(mode="manual", left=-100, right=100, duration_ms=2000),
    "follow_mode": dict(enabled=True, target_id=65535),
    "follow": dict(target_id=0, left=30, right=-30, duration_ms=50, confidence=0.5),
    "led": dict(state="blink", interval_ms=100),
    "buzzer": dict(pattern="emergency"),
    "emergency_stop": dict(reason="fall_confirmed"),
    "clear_emergency": dict(operator_confirmed=True),
    "obstacle": dict(source="camera", distance_cm=999),
    "command": dict(command="LEFT", duration_ms=500),
}


def test_every_type_has_a_valid_example():
    assert set(VALID) == set(SCHEMA)
    for t, f in VALID.items():
        assert make_command(t, **f).type == t


@pytest.mark.parametrize("msg_type,fields", [
    ("motor", dict(mode="manual", left=101, right=0, duration_ms=300)),
    ("motor", dict(mode="manual", left=10, right=0, duration_ms=49)),
    ("motor", dict(mode="manual", left=10, right=0, duration_ms=2001)),
    ("motor", dict(mode="auto", left=10, right=0, duration_ms=300)),
    ("motor", dict(mode="manual", left=1.5, right=0, duration_ms=300)),
    ("motor", dict(mode="manual", left=True, right=0, duration_ms=300)),       # bool is not a number
    ("motor", dict(mode="manual", left=10, right=0)),                          # missing field
    ("motor", dict(mode="manual", left=10, right=0, duration_ms=300, speed=3)),  # unknown field
    ("follow", dict(target_id=1, left=1, right=1, duration_ms=300, confidence=1.01)),
    ("follow", dict(target_id=-1, left=1, right=1, duration_ms=300, confidence=0.9)),
    ("follow_mode", dict(enabled=True)),                                       # target required
    ("follow_mode", dict(enabled=1, target_id=1)),
    ("led", dict(state="red")),
    ("led", dict(state="blink", interval_ms=99)),
    ("buzzer", dict(pattern="siren")),
    ("emergency_stop", dict(reason="has space")),
    ("emergency_stop", dict(reason="x" * 25)),
    ("clear_emergency", dict(operator_confirmed=False)),
    ("obstacle", dict(source="sensor", distance_cm=10)),
    ("obstacle", dict(source="camera", distance_cm=1000)),
    ("jump", {}),
])
def test_laptop_refuses_invalid_commands(msg_type, fields):
    with pytest.raises(CommandError):
        make_command(msg_type, **fields)


def test_encode_is_compact_ordered_and_bounded():
    line = encode(17, "m-17", make_command("motor", mode="manual", left=30, right=30, duration_ms=300))
    assert line == (b'{"version":3,"type":"motor","command_id":"m-17","seq":17,"mode":"manual",'
                    b'"left":30,"right":30,"duration_ms":300}\n')
    longest = encode(999_999_999, "a" * 24, make_command("follow", target_id=65535, left=-100, right=-100,
                                                         duration_ms=2000, confidence=0.123456))
    assert len(longest) - 1 <= MAX_LINE
    with pytest.raises(CommandError):
        encode(0, "x", make_command("ping"))
    with pytest.raises(CommandError):
        encode(1, "bad id", make_command("ping"))


def test_every_laptop_command_passes_the_robot_parser_and_is_accepted():
    """The robot must accept (or refuse only for state reasons) everything the laptop can build."""
    seq = 0
    for t, f in VALID.items():
        core = SimRobotCore(motors_enabled=True)
        core.begin(0)
        core.connected(0)
        if t == "follow":
            seq += 1
            core.handle(encode(seq, "pre", make_command("follow_mode", enabled=True, target_id=0)).decode().strip(), 1)
        seq += 1
        ack = json.loads(core.handle(encode(seq, f"c{seq}", make_command(t, **f)).decode().strip(), 2))
        assert ack["ok"] is True, (t, ack)
        assert ack["command_id"] == f"c{seq}" and ack["seq"] == seq


def test_random_valid_commands_round_trip_through_the_robot_parser():
    rng = random.Random(3)
    for i in range(500):
        t = rng.choice(["motor", "follow", "led", "obstacle", "follow_mode"])
        if t == "motor":
            f = dict(mode="manual", left=rng.randint(-100, 100), right=rng.randint(-100, 100),
                     duration_ms=rng.randint(50, 2000))
        elif t == "follow":
            f = dict(target_id=rng.randint(0, 65535), left=rng.randint(-100, 100), right=rng.randint(-100, 100),
                     duration_ms=rng.randint(50, 2000), confidence=rng.random())
        elif t == "led":
            f = dict(state=rng.choice(["on", "off", "blink"]), interval_ms=rng.randint(100, 2000))
        elif t == "obstacle":
            f = dict(source="camera", distance_cm=rng.randint(0, 999))
        else:
            f = dict(enabled=True, target_id=rng.randint(0, 65535))
        line = encode(i + 1, f"r{i}", make_command(t, **f)).decode().strip()
        parsed = parse_json_object(line)
        assert parsed is not None, line
        values = {k: _value(v) for k, v in parsed.items()}
        sent = json.loads(line)
        assert set(values) == set(sent)
        for k, v in sent.items():           # decimals: the robot keeps 6 places
            assert abs(values[k] - v) < 1e-9 if isinstance(v, float) else values[k] == v, (k, line)


def _value(f):
    return {"str": f.s, "int": f.i, "num": f.d, "bool": f.b, "null": None}[f.type]


def test_parse_line_robot_messages():
    r = parse_line('{"version":3,"type":"ack","command_id":"a","seq":4,"ok":false,"error":"BAD_SEQ"}')
    assert r.kind == "ack" and not r.ok and r.error == "BAD_SEQ" and r.seq == 4 and r.command_id == "a"
    r = parse_line('{"version":3,"type":"event","event":"watchdog_stop"}')
    assert r.kind == "event" and r.event == "watchdog_stop"
    r = parse_line('{"version":3,"type":"status","state":"idle"}')
    assert r.kind == "status" and r.data["state"] == "idle"
    for bad in ("OK:1:PING", "[1,2]", '{"version":3,"type":"hello"}', '{"version":2,"type":"ack"}', ""):
        with pytest.raises(ProtocolError):
            parse_line(bad)
    assert PROTOCOL_VERSION == 3
