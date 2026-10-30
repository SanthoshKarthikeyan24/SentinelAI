"""The Python simulator must behave EXACTLY like the firmware core (protocol v3).

The same timed scripts (a hand-written scenario + thousands of random steps,
including malformed JSON) are replayed through firmware/host_test/core_replay.cpp
(the real robot_core.cpp) and through robot/sim_robot.SimRobotCore; every ack,
status, event and motor/LED/buzzer output must be byte-identical.
"""
import json
import random
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

from robot.sim_robot import SimRobotCore

FW = Path(__file__).resolve().parent.parent / "firmware"


@pytest.fixture(scope="module")
def replay_binary(tmp_path_factory):
    if shutil.which("g++") is None or sys.platform.startswith("win"):
        pytest.skip("g++ not available")
    exe = tmp_path_factory.mktemp("replay") / "core_replay"
    subprocess.run(["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror", f"-I{FW / 'sentinel_robot'}",
                    str(FW / "host_test" / "core_replay.cpp"), str(FW / "sentinel_robot" / "robot_core.cpp"),
                    "-o", str(exe)], check=True, capture_output=True, text=True)
    return exe


def python_replay(script: str, motors: bool, sensor: bool) -> str:
    out = []
    core = SimRobotCore(motors_enabled=motors, sensor_enabled=sensor,
                        on_output=lambda k, a, b: out.append(f"{k} {int(a)} {int(b)}"))
    begun = False
    for line in script.splitlines():
        if not line:
            continue
        t_txt, _, rest = line.partition(" ")
        t = int(t_txt)
        if not begun:
            core.begin(t)
            begun = True
        if rest == "CONNECT":
            out.append("H " + core.connected(t))
        elif rest == "DISCONNECT":
            core.disconnected(t)
        elif rest == "TICK":
            core.tick(t)
        elif rest == "OVERFLOW":
            out.append("R " + core.overflow())
        elif rest.startswith("DIST "):
            core.on_distance(int(rest[5:]), t)
        elif rest.startswith("LINE "):
            out.append("R " + core.handle(rest[5:], t))
        out.extend("E " + m for m in core.pop_messages())
    return "\n".join(out) + ("\n" if out else "")


def cpp_replay(exe, script: str, motors: bool, sensor: bool) -> str:
    r = subprocess.run([str(exe), "1" if motors else "0", "1" if sensor else "0"], input=script,
                       capture_output=True, text=True, timeout=60)
    assert r.returncode == 0
    return r.stdout


def first_difference(py: str, cpp: str) -> str:
    pl, cl = py.splitlines(), cpp.splitlines()
    i = next((i for i, (a, b) in enumerate(zip(pl, cl)) if a != b), min(len(pl), len(cl)))
    return f"first difference at output line {i}: python={pl[i:i + 3]} cpp={cl[i:i + 3]}"


def J(seq, typ, cid=None, **f):
    msg = {"version": 3, "type": typ, "command_id": cid or f"c{seq}", "seq": seq}
    msg.update(f)
    return json.dumps(msg, separators=(",", ":"))


SCENARIO_STEPS = [
    (0, "CONNECT"),
    (5, J(1, "ping")),
    (10, J(2, "follow_mode", enabled=True, target_id=7)),
    (130, "TICK"),
    (200, J(3, "follow", target_id=7, left=30, right=20, duration_ms=300, confidence=0.9)),
    (210, J(4, "follow", target_id=8, left=30, right=20, duration_ms=300, confidence=0.9)),
    (220, J(5, "follow", target_id=7, left=30, right=20, duration_ms=300, confidence=0.2)),
    (230, J(6, "motor", mode="manual", left=30, right=30, duration_ms=300)),
    (250, "TICK"),
    (260, "DIST 10"),
    (300, J(7, "obstacle", source="camera", distance_cm=20)),
    (320, J(8, "follow", target_id=7, left=30, right=30, duration_ms=300, confidence=0.9)),
    (330, J(9, "follow_mode", enabled=False)),
    (335, J(10, "follow", target_id=7, left=10, right=10, duration_ms=300, confidence=0.9)),
    (340, J(11, "motor", mode="manual", left=30, right=30, duration_ms=300)),
    (350, J(12, "motor", mode="manual", left=-30, right=-30, duration_ms=300)),
    (700, "TICK"),
    (900, "DIST 50"),
    (1000, "TICK"),
    (1810, "TICK"),
    (1820, J(13, "obstacle_clear")),
    (1900, J(14, "led", state="blink", interval_ms=250)),
    (2400, "TICK"),
    (2600, J(15, "buzzer", pattern="warning")),
    (2700, "TICK"),
    (2900, "TICK"),
    (3000, J(16, "buzzer", pattern="emergency")),
    (3400, "TICK"),
    (3500, J(17, "status_request")),
    (3600, J(18, "mute")),
    (3700, J(19, "emergency_stop", reason="fall_confirmed")),
    (3710, J(20, "motor", mode="manual", left=30, right=30, duration_ms=300)),
    (3720, J(21, "follow_mode", enabled=True, target_id=1)),
    (3730, J(22, "emergency_stop", reason="operator")),
    (3740, J(23, "clear_emergency", operator_confirmed=False)),
    (3750, "DISCONNECT"),
    (3760, "CONNECT"),
    (3770, J(1, "mute")),
    (3790, J(2, "clear_emergency", operator_confirmed=True)),
    (3800, J(3, "status_request")),
    (3810, J(4, "motor", mode="manual", left=90, right=-90, duration_ms=2000)),
    (4400, "TICK"),
    (4410, "OVERFLOW"),
    (4420, J(4, "ping")),
    (4430, "hello"),
    (4440, J(6, "motor", mode="manual", left=101, right=0, duration_ms=300)),
    (4450, J(7, "jump")),
    (4460, J(8, "ping", extra=1)),
    (4470, '{"version":2,"type":"ping","command_id":"a","seq":9}'),
    (4480, '{"version":3,"type":"ping","command_id":"bad id!","seq":10}'),
    (4490, '{"version":3,"type":"ping","command_id":"x","seq":11,"seq":12}'),
    (4500, '{"version":3,"type":"motor","command_id":"x","seq":13,"mode":"manual","left":1e2,"right":0,'
           '"duration_ms":300}'),
    (4510, '{ "version" : 3 , "type" : "stop" , "command_id" : "sp" , "seq" : 14 }'),
    (4520, J(15, "motor", mode="manual", left=20, right=20, duration_ms=500)),
    (5100, "TICK"),
    (5110, J(16, "stop")),
    (5120, J(17, "led", state="on")),
    (5130, '{"version":3,"type":"ping","command_id":"long","seq":18,"pad":"' + "x" * 160 + '"}'),
    (5200, "TICK"),
    (6300, "TICK"),
    (6400, J(19, "emergency_stop", reason="fall_confirmed")),
    (40000, "TICK"),
    (66500, "TICK"),
    (66600, J(20, "status_request")),
    (66700, J(21, "clear_emergency", operator_confirmed=True)),
    # named commands (simulated movement, LED, buzzer, status, emergency)
    (66710, J(22, "command", command="LEFT")),
    (66720, J(23, "command", command="STATUS")),
    (66730, J(24, "command", command="LED_BLINK")),
    (66740, J(25, "command", command="BUZZER_ON")),
    (66800, "TICK"),
    (67000, J(26, "command", command="BUZZER_OFF")),
    (67010, J(27, "command", command="FORWARD", duration_ms=300)),
    (67100, "TICK"),
    (67400, "TICK"),
    (67410, J(28, "command", command="RIGHT")),
    (68100, "TICK"),
    (68110, J(29, "command", command="BACKWARD")),
    (68120, J(30, "command", command="STOP")),
    (68130, J(31, "command", command="JUMP")),
    (68140, J(32, "command", command="STOP", duration_ms=300)),
    (68150, J(33, "command", command="LEFT", duration_ms=5000)),
    (68160, J(34, "command", command="LED_ON")),
    (68170, J(35, "command", command="BUZZER_SHORT_BEEP")),
    (68180, J(36, "command", command="BUZZER_WARNING")),
    (68190, J(37, "obstacle", source="camera", distance_cm=10)),
    (68200, J(38, "command", command="FORWARD")),
    (68210, J(39, "command", command="BACKWARD")),
    (68220, J(40, "command", command="LED_OFF")),
    (68230, J(41, "command", command="EMERGENCY_STOP")),
    (68240, J(42, "command", command="LEFT")),
    (68250, J(43, "command", command="BUZZER_OFF")),
    (68260, J(44, "status_request")),
    (68270, J(45, "clear_emergency", operator_confirmed=True)),
    (68280, J(46, "follow_mode", enabled=True, target_id=2)),
    (68290, J(47, "command", command="RIGHT")),
    (68300, J(48, "command", command="STOP")),
    (68310, J(49, "command", command="LEFT")),
    (68320, J(50, "command", command="STOP")),
    # AI events: model running light, fall alert (blink + buzzer pattern), repeats, mute, emergency priority
    (68400, J(51, "command", command="MODEL_STARTED")),
    (68410, J(52, "command", command="MODEL_STARTED")),
    (68420, J(53, "command", command="LED_BLINK")),
    (68430, J(54, "command", command="MODEL_STARTED")),
    (68500, "TICK"),
    (68600, J(55, "command", command="FALL_DETECTED")),
    (68700, "TICK"), (68850, "TICK"), (69000, "TICK"), (69100, "TICK"), (69250, "TICK"),
    (69300, J(56, "command", command="FALL_DETECTED")),
    (69310, J(57, "command", command="LEFT")),
    (69320, J(58, "motor", mode="manual", left=10, right=10, duration_ms=300)),
    (69330, J(59, "follow_mode", enabled=True, target_id=3)),
    (69340, J(60, "command", command="BUZZER_SHORT_BEEP")),
    (69350, J(61, "status_request")),
    (69400, "TICK"),
    (69500, J(62, "command", command="BUZZER_OFF")),
    (69600, "TICK"),
    (69700, J(63, "command", command="FALL_CLEARED")),
    (69710, J(64, "command", command="FALL_CLEARED")),
    (69720, J(65, "status_request")),
    (69800, J(66, "command", command="FALL_DETECTED")),
    (69810, J(67, "command", command="EMERGENCY_STOP")),
    (69900, "TICK"),
    (69910, J(68, "status_request")),
    (69920, J(69, "clear_emergency", operator_confirmed=True)),
    (69930, J(70, "status_request")),
    (70000, J(71, "mute")),
    (70010, J(72, "command", command="FALL_CLEARED")),
    (70020, J(73, "command", command="FALL_DETECTED")),
    (131000, "TICK"),
    (131010, J(74, "command", command="MODEL_STOPPED")),
    (131020, J(75, "status_request")),
    (131030, J(76, "command", command="MODEL_STARTED")),
    (131040, "DISCONNECT"),
    (131050, "CONNECT"),
    (131060, J(1, "status_request")),
]
SCENARIO = "".join(f"{t} {s if s.isupper() or s.startswith('DIST') else 'LINE ' + s}\n" for t, s in SCENARIO_STEPS)


@pytest.mark.parametrize("motors,sensor", [(True, True), (False, False), (True, False)])
def test_scenario_identical(replay_binary, motors, sensor):
    py = python_replay(SCENARIO, motors, sensor)
    cpp = cpp_replay(replay_binary, SCENARIO, motors, sensor)
    assert py == cpp, first_difference(py, cpp)


def test_scenario_covers_the_protocol():
    """The scenario must exercise every message type and the main refusal codes."""
    py = python_replay(SCENARIO, True, True)
    import robot.commands as c
    for typ in c.SCHEMA:
        assert f'"type":"{typ}"' in SCENARIO, typ
    for code in ("BAD_JSON", "BAD_VERSION", "BAD_FIELD", "UNKNOWN_FIELD", "UNKNOWN_TYPE", "BAD_SEQ",
                 "EMERGENCY_ACTIVE", "OBSTACLE_AHEAD", "FOLLOW_ACTIVE", "FOLLOW_NOT_ENABLED", "TARGET_MISMATCH",
                 "LOW_CONFIDENCE", "NEEDS_CONFIRM", "TOO_LONG", "UNKNOWN_COMMAND", "FALL_ALERT_ACTIVE"):
        assert f'"error":"{code}"' in py, code
    for event in ("hello", "watchdog_stop", "move_done", "obstacle", "obstacle_clear", "buzzer_timeout"):
        assert f'"event":"{event}"' in py, event


# ---- random scripts -------------------------------------------------------
RAW_LINES = [
    "", "{", "}", "{}", "[]", "null", "hello", "PING", '{"version":3}', '{"version":3,"type":"ping"}',
    '{"version":03,"type":"ping","command_id":"a","seq":1}',
    '{"version":3.0,"type":"ping","command_id":"a","seq":1}',
    '{"version":3,"type":"ping","command_id":"a\\"b","seq":1}',
    '{"version":3,"type":"ping","command_id":"a","seq":-1}',
    '{"version":3,"type":"ping","command_id":"","seq":1}',
    '{"version":3,"type":"ping","command_id":"a","seq":1,}',
    '{"version":3,"type":"ping","command_id":"a","seq":1}x',
    '{"version":3,"type":"ping","command_id":"a","seq":{"n":1}}',
    '{"version":3,"type":"ping","command_id":"a","seq":true}',
    '{"version":3,"type":"ping","":1,"command_id":"a","seq":1}',
    '{"version":3,"type":"ping","command_id":"aaaaaaaaaaaaaaaaaaaaaaaaa","seq":1}',
    '{"version":3,"type":"ping","command_id":"a","seq":1234567890}',
    '\t{"version":3,"type":"ping","command_id":"t","seq":5}\t',
]
CHOICES = {
    "left": [0, 1, 30, -30, 60, 61, 100, -100, 101, -101, 2.5, "x", True],
    "right": [0, 20, -20, 59, 100, -100, -101, None],
    "duration_ms": [49, 50, 300, 1999, 2000, 2001, 0, -5, 1.0],
    "target_id": [0, 1, 7, 65535, 65536, -1, "7"],
    "confidence": [0, 0.49, 0.5, 0.499999, 0.75, 1, 1.0000001, 1.5, -0.1, "hi", 0.1234567],
    "enabled": [True, False, 1, "yes"],
    "state": ["on", "off", "blink", "BLINK", 1],
    "interval_ms": [99, 100, 250, 2000, 2001],
    "pattern": ["off", "short_beep", "warning", "emergency", "siren"],
    "reason": ["fall_confirmed", "operator", "x.y-z", "bad reason", "", "a" * 25],
    "operator_confirmed": [True, False, 1],
    "source": ["camera", "sensor"],
    "distance_cm": [0, 5, 35, 999, 1000, -1],
    "mode": ["manual", "auto"],
    "command": ["FORWARD", "BACKWARD", "LEFT", "RIGHT", "STOP", "LED_ON", "LED_OFF", "LED_BLINK", "BUZZER_ON",
                "BUZZER_OFF", "BUZZER_SHORT_BEEP", "BUZZER_WARNING", "STATUS", "EMERGENCY_STOP", "JUMP", "left", 5,
                "MODEL_STARTED", "MODEL_STOPPED", "FALL_DETECTED", "FALL_CLEARED", "FALL_CLEARED", "MODEL_STARTED"],
}
TYPE_FIELDS = {
    "ping": [], "status_request": [], "stop": [], "mute": [], "obstacle_clear": [],
    "motor": ["mode", "left", "right", "duration_ms"],
    "follow_mode": ["enabled", "target_id"],
    "follow": ["target_id", "left", "right", "duration_ms", "confidence"],
    "led": ["state", "interval_ms"],
    "buzzer": ["pattern"],
    "emergency_stop": ["reason"],
    "clear_emergency": ["operator_confirmed"],
    "obstacle": ["source", "distance_cm"],
    "command": ["command", "duration_ms"],
}
GOOD = {"left": 30, "right": 20, "duration_ms": 300, "target_id": 7, "confidence": 0.9, "enabled": True,
        "state": "blink", "interval_ms": 300, "pattern": "short_beep", "reason": "fall_confirmed",
        "operator_confirmed": True, "source": "camera", "distance_cm": 30, "mode": "manual",
        "command": "LEFT"}
WEIGHTS = {"motor": 6, "follow": 6, "follow_mode": 3, "stop": 2, "ping": 1, "status_request": 1, "mute": 1,
           "obstacle_clear": 2, "led": 2, "buzzer": 2, "emergency_stop": 1, "clear_emergency": 2, "obstacle": 2,
           "command": 5}


def random_command(rng: random.Random, seq) -> str:
    typ = rng.choices(list(WEIGHTS), weights=list(WEIGHTS.values()))[0]
    msg = {"version": 3, "type": typ, "command_id": f"r{rng.randrange(1000)}", "seq": seq}
    for name in TYPE_FIELDS[typ]:
        if typ == "command" and name == "duration_ms" and rng.random() < 0.7:
            continue                                   # usually no duration for named commands
        if rng.random() < 0.06:
            continue                                   # missing field
        v = rng.choice(CHOICES[name]) if rng.random() < 0.3 else GOOD[name]
        if name == "target_id" and typ == "follow" and rng.random() < 0.6:
            v = 7
        if v is not None:
            msg[name] = v
    if rng.random() < 0.03:
        msg["extra"] = 1
    if rng.random() < 0.03:
        msg["version"] = rng.choice([2, 4, "3"])
    if rng.random() < 0.03:
        msg["type"] = rng.choice(["jump", "PING", ""])
    items = list(msg.items())
    if rng.random() < 0.1:
        rng.shuffle(items)                             # key order must not matter
    sep = (", ", ": ") if rng.random() < 0.1 else (",", ":")
    return json.dumps(dict(items), separators=sep)


def random_script(rng: random.Random, steps: int) -> str:
    t, seq, lines = 0, 0, ["0 CONNECT"]
    for _ in range(steps):
        t += rng.choice([1, 5, 20, 50, 100, 130, 250, 400, 600, 1100, 1600])
        k = rng.random()
        if k < 0.33:
            lines.append(f"{t} TICK")
        elif k < 0.42:
            lines.append(f"{t} DIST {rng.choice([-1, 5, 20, 24, 25, 30, 35, 36, 80, 300])}")
        elif k < 0.45:
            lines.append(f"{t} DISCONNECT")
            lines.append(f"{t} CONNECT")
            seq = 0
        elif k < 0.47:
            lines.append(f"{t} OVERFLOW")
        elif k < 0.51:
            lines.append(f"{t} LINE {rng.choice(RAW_LINES)}")
        else:
            if rng.random() < 0.92:
                seq += 1
                s = seq
            else:
                s = rng.choice([0, max(seq, 1), 1234567890, -3])
            lines.append(f"{t} LINE {random_command(rng, s)}")
    return "\n".join(lines) + "\n"


@pytest.mark.parametrize("seed", range(16))
def test_random_scripts_identical(replay_binary, seed):
    rng = random.Random(seed)
    script = random_script(rng, 700)
    motors, sensor = bool(seed % 2), seed % 3 != 0
    py = python_replay(script, motors, sensor)
    cpp = cpp_replay(replay_binary, script, motors, sensor)
    assert py == cpp, first_difference(py, cpp)


def test_random_scripts_reach_interesting_states():
    """Guard against a generator that only ever produces rejected commands."""
    py = "".join(python_replay(random_script(random.Random(seed), 700), True, True) for seed in range(4))
    assert py.count('"ok":true') > 200
    for state in ("idle", "moving", "following", "emergency"):
        assert f'"state":"{state}"' in py, state
    for event in ("watchdog_stop", "move_done", "obstacle_clear"):
        assert f'"event":"{event}"' in py, event
    assert "M 30 20" in py
