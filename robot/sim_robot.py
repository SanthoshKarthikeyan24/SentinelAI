"""Pure-Python robot simulator (works on Windows) speaking protocol v3 (JSON lines).

SimRobotCore is a line-by-line Python mirror of firmware/sentinel_robot/robot_core.cpp,
including its strict JSON parser, field checks, watchdog, movement time-outs, follow mode,
obstacle hold, sensor logic, LED/buzzer patterns, emergency latch and periodic status.
tests/test_core_parity.py replays identical scripts through both and requires identical
output, so testing against this simulator tests the firmware logic.

Usage:
    python -m robot.sim_robot                         # 127.0.0.1:3333
    python -m robot.sim_robot --port 3334 --host 0.0.0.0 --motors --sensor
With --sensor, type a distance in cm (e.g. 20) + Enter to simulate the sensor; -1 = nothing.
"""
from __future__ import annotations

import argparse
import logging
import socket
import threading
import time
from collections import deque

from .commands import PROTOCOL_VERSION

log = logging.getLogger("sentinel.sim_robot")
MASK = 0xFFFFFFFF            # millis() is a 32-bit unsigned counter on the ESP32
MAX_LINE = 200
MAX_FIELDS, MAX_KEY, MAX_STR = 12, 23, 32
OUT_QUEUE = 6
LED_NAMES = ("off", "on", "blink")
BUZ_NAMES = ("off", "short_beep", "warning", "emergency", "on")
SIM_NAMES = {"FORWARD": "forward", "BACKWARD": "backward", "LEFT": "left", "RIGHT": "right"}
NAMED = ("FORWARD", "BACKWARD", "LEFT", "RIGHT", "STOP", "LED_ON", "LED_OFF", "LED_BLINK", "BUZZER_ON",
         "BUZZER_OFF", "BUZZER_SHORT_BEEP", "BUZZER_WARNING", "STATUS", "EMERGENCY_STOP", "MODEL_STARTED",
         "MODEL_STOPPED", "FALL_DETECTED", "FALL_CLEARED")

# same order as kTypes in robot_core.cpp
TYPES = {
    "ping": ([], []),
    "status_request": ([], []),
    "motor": (["mode", "left", "right", "duration_ms"], []),
    "stop": ([], []),
    "follow_mode": (["enabled"], ["target_id"]),
    "follow": (["target_id", "left", "right", "duration_ms", "confidence"], []),
    "led": (["state"], ["interval_ms"]),
    "buzzer": (["pattern"], []),
    "mute": ([], []),
    "emergency_stop": (["reason"], []),
    "clear_emergency": (["operator_confirmed"], []),
    "obstacle": (["source", "distance_cm"], []),
    "obstacle_clear": ([], []),
    "command": (["command"], ["duration_ms"]),
}
COMMON = ("version", "type", "command_id", "seq")


def valid_id(s: str) -> bool:
    if not 1 <= len(s) <= 24:
        return False
    return all(c.isascii() and (c.isalnum() or c in "_-.") for c in s)


def _printable(c: str) -> bool:
    return 0x20 <= ord(c) <= 0x7E and c != "\\"


class JField:
    __slots__ = ("type", "s", "i", "d", "b")

    def __init__(self):
        self.type, self.s, self.i, self.d, self.b = "", "", 0, 0.0, False


def parse_json_object(text: str) -> dict | None:
    """Mirror of parseJsonObject(): returns {key: JField} or None."""
    p, n = 0, len(text)

    def ch(k=0):
        return text[p + k] if p + k < n else ""

    def skip():
        nonlocal p
        while ch() in (" ", "\t") and ch() != "":
            p += 1

    out: dict = {}
    skip()
    if ch() != "{":
        return None
    p += 1
    skip()
    if ch() == "}":
        p += 1
        skip()
        return out if p == n else None
    while True:
        if len(out) >= MAX_FIELDS:
            return None
        if ch() != '"':
            return None
        p += 1
        key = ""
        while ch() not in ("", '"'):
            if not _printable(ch()) or len(key) >= MAX_KEY:
                return None
            key += ch()
            p += 1
        if ch() != '"' or not key:
            return None
        p += 1
        if key in out:
            return None
        skip()
        if ch() != ":":
            return None
        p += 1
        skip()
        f = JField()
        if ch() == '"':
            p += 1
            s = ""
            while ch() not in ("", '"'):
                if not _printable(ch()) or len(s) >= MAX_STR:
                    return None
                s += ch()
                p += 1
            if ch() != '"':
                return None
            p += 1
            f.type, f.s = "str", s
        elif text.startswith("true", p):
            f.type, f.b = "bool", True
            p += 4
        elif text.startswith("false", p):
            f.type = "bool"
            p += 5
        elif text.startswith("null", p):
            f.type = "null"
            p += 4
        elif ch() == "-" or ch().isdigit():
            neg = False
            if ch() == "-":
                neg = True
                p += 1
            if not (ch() and "0" <= ch() <= "9"):
                return None
            if ch() == "0" and ch(1) and "0" <= ch(1) <= "9":
                return None
            ip, nd = 0, 0
            while ch() and "0" <= ch() <= "9":
                if nd >= 9:
                    return None
                ip = ip * 10 + int(ch())
                p += 1
                nd += 1
            if ch() == ".":
                p += 1
                if not (ch() and "0" <= ch() <= "9"):
                    return None
                fr, fd, pw = 0, 0, 1.0
                while ch() and "0" <= ch() <= "9":
                    if fd >= 6:
                        return None
                    fr = fr * 10 + int(ch())
                    pw *= 10.0
                    p += 1
                    fd += 1
                v = float(ip) + float(fr) / pw
                f.type, f.d = "num", (-v if neg else v)
            else:
                f.type = "int"
                f.i = -ip if neg else ip
                f.d = float(f.i)
        else:
            return None
        out[key] = f
        skip()
        if ch() == ",":
            p += 1
            skip()
            continue
        if ch() == "}":
            p += 1
            skip()
            return out if p == n else None
        return None


def _int_in(f, lo, hi) -> bool:
    return f is not None and f.type == "int" and lo <= f.i <= hi


def _str_one_of(f, opts) -> bool:
    return f is not None and f.type == "str" and f.s in opts


def _clamp(v: int, lim: int) -> int:
    return lim if v > lim else (-lim if v < -lim else v)


def _b(v: bool) -> str:
    return "true" if v else "false"


class SimRobotCore:
    """Python mirror of RobotCore (robot_core.cpp)."""

    def __init__(self, watchdog_ms=500, min_move_ms=50, max_move_ms=2000, max_speed=60, motors_enabled=False,
                 obstacle_hold_ms=1500, sensor_enabled=False, obstacle_stop_cm=25, obstacle_clear_cm=35,
                 status_interval_ms=1000, emergency_buzz_max_ms=60000, follow_min_confidence=0.5,
                 sim_move_default_ms=2000, led_installed=True, buzzer_installed=True, fall_blink_ms=250,
                 on_output=None):
        self.watchdog_ms, self.min_move_ms, self.max_move_ms = watchdog_ms, min_move_ms, max_move_ms
        self.max_speed = max(0, min(100, max_speed))
        self.motors_enabled = motors_enabled
        self.obstacle_hold_ms = obstacle_hold_ms
        self.sensor_enabled = sensor_enabled
        self.stop_cm = obstacle_stop_cm
        self.clear_cm = max(obstacle_clear_cm, obstacle_stop_cm)
        self.status_interval_ms = status_interval_ms
        self.emergency_buzz_max_ms = emergency_buzz_max_ms
        self.follow_min_confidence = follow_min_confidence
        self.sim_move_default_ms = sim_move_default_ms
        self.led_installed, self.buzzer_installed = led_installed, buzzer_installed
        self.sim_move = "none"         # simulated named movement (never reaches the motor outputs)
        self.sim_start = self.sim_dur = 0
        self.fall_blink_ms = fall_blink_ms
        self.model_running = False     # AI state reported by the laptop
        self.fall_alert = self.fall_muted = self.fall_buzz_reported = False
        self.fall_start = 0
        self.on_output = on_output
        self.state = "IDLE"            # IDLE / MOVING / EMERGENCY
        self.left = self.right = 0
        self.follow = False
        self.follow_target = -1
        self.connected_flag = False
        self.comm_fault = False
        self.last_valid = self.move_start = self.move_dur = 0
        self.last_seq = 0
        self.watchdog_trips = 0
        self.rejected = 0
        self.boot = 0
        self.last_status = 0
        self.last_command_id = ""
        self.emergency_reason = ""
        self.emergency_start = 0
        self.muted = False
        self.emergency_buzz_reported = False
        self.local_obstacle = self.remote_obstacle = False
        self.remote_set = 0
        self.distance_cm = -1
        self.led_mode = 0              # 0 off, 1 on, 2 blink
        self.led_interval = 500
        self.buzzer_mode = 0           # 0 off, 1 short, 2 warning, 3 emergency
        self.buzzer_start = 0
        self.buzzer = self.led = False
        self._outputs_init = False
        self._queue: deque = deque()
        self.hw_log: list[tuple] = []

    # ----------------------------------------------------------- outputs
    def _out_motors(self, l, r):
        self.hw_log.append(("M", l, r))
        if self.on_output:
            self.on_output("M", l, r)

    def _out_alarm(self, b, led):
        self.hw_log.append(("A", b, led))
        if self.on_output:
            self.on_output("A", b, led)

    @property
    def obstacle_active(self) -> bool:
        return self.local_obstacle or self.remote_obstacle

    def state_name(self) -> str:
        if self.state == "EMERGENCY":
            return "emergency"
        if self.state == "MOVING":
            return "moving"
        return "following" if self.follow else "idle"

    def _push(self, text: str) -> None:
        if len(self._queue) == OUT_QUEUE:
            self._queue.popleft()
        self._queue.append(text)

    def pop_messages(self) -> list[str]:
        out = list(self._queue)
        self._queue.clear()
        return out

    def _stop(self):
        self.left = self.right = 0
        self._out_motors(0, 0)
        if self.state == "MOVING":
            self.state = "IDLE"

    def _drive(self, l, r, now, dur) -> bool:
        if self.state == "EMERGENCY":
            self._stop()
            return False
        if l == 0 and r == 0:
            self._stop()
            return True
        self.left, self.right = l, r
        self.move_start, self.move_dur = now, dur
        self.state = "MOVING"
        self._out_motors(l if self.motors_enabled else 0, r if self.motors_enabled else 0)
        return True

    def _enter_emergency(self, reason: str, now: int):
        self._stop()
        self.sim_move = "none"
        self.follow = False
        self.follow_target = -1
        if self.state != "EMERGENCY":
            self.state = "EMERGENCY"
            self.emergency_reason = reason
            self.emergency_start = now
            self.muted = False
            self.emergency_buzz_reported = False
        self._update_indicators(now)

    def _start_buzzer(self, mode: int, now: int):
        self.buzzer_mode, self.buzzer_start = mode, now

    def _fall_buzzer_active(self, now: int) -> bool:
        return self.fall_alert and not self.fall_muted and ((now - self.fall_start) & MASK) < self.emergency_buzz_max_ms

    def _buzzer_sounding(self, now: int) -> bool:
        if self.state == "EMERGENCY":
            t = (now - self.emergency_start) & MASK
            return (not self.muted) and t < self.emergency_buzz_max_ms and (t % 500) < 300
        if self.fall_alert:
            if not self._fall_buzzer_active(now):
                return False
            c = ((now - self.fall_start) & MASK) % 1200
            return c < 200 or 400 <= c < 600
        t = (now - self.buzzer_start) & MASK
        if self.buzzer_mode == 1:
            return t < 150
        if self.buzzer_mode == 2:
            return t < 900 and (t % 300) < 150
        if self.buzzer_mode == 3:
            return t < self.emergency_buzz_max_ms and (t % 500) < 300
        if self.buzzer_mode == 4:
            return t < self.emergency_buzz_max_ms
        return False

    def _update_indicators(self, now: int) -> None:
        now &= MASK
        if self.state == "EMERGENCY":
            led = True
        elif self.fall_alert:
            led = (((now - self.fall_start) & MASK) // self.fall_blink_ms) % 2 == 0
        elif self.obstacle_active:
            led = (now // 100) % 2 == 0
        elif self.led_mode == 1:
            led = True
        elif self.led_mode == 2:
            led = (now // self.led_interval) % 2 == 0
        elif self.model_running:
            led = True
        else:
            led = self.follow and (now // 500) % 2 == 0
        if self.state != "EMERGENCY":
            t = (now - self.buzzer_start) & MASK
            if (self.buzzer_mode == 1 and t >= 150) or (self.buzzer_mode == 2 and t >= 900):
                self.buzzer_mode = 0
            elif self.buzzer_mode in (3, 4) and t >= self.emergency_buzz_max_ms:
                self.buzzer_mode = 0
                self._push('{"version":3,"type":"event","event":"buzzer_timeout"}')
        elif (not self.muted and not self.emergency_buzz_reported
              and ((now - self.emergency_start) & MASK) >= self.emergency_buzz_max_ms):
            self.emergency_buzz_reported = True
            self._push('{"version":3,"type":"event","event":"buzzer_timeout"}')
        if (self.fall_alert and not self.fall_muted and not self.fall_buzz_reported
                and ((now - self.fall_start) & MASK) >= self.emergency_buzz_max_ms):
            self.fall_buzz_reported = True
            self._push('{"version":3,"type":"event","event":"buzzer_timeout","alert":"fall"}')
        buzzer = self._buzzer_sounding(now)
        if not self._outputs_init or buzzer != self.buzzer or led != self.led:
            self._outputs_init = True
            self.buzzer, self.led = buzzer, led
            self._out_alarm(buzzer, led)

    # ---------------------------------------------------------------- API
    def begin(self, now):
        self.boot = self.last_valid = self.last_status = now
        self.state = "IDLE"
        self.follow = False
        self.follow_target = -1
        self.model_running = False
        self.fall_alert = False
        self._stop()
        self._outputs_init = False
        self._update_indicators(now)

    def connected(self, now) -> str:
        self.connected_flag = True
        self.last_seq = 0
        self.last_valid = self.last_status = now
        return (f'{{"version":3,"type":"event","event":"hello","protocol":{PROTOCOL_VERSION},'
                f'"state":"{self.state_name()}","motors_enabled":{_b(self.motors_enabled)},'
                f'"sensor_installed":{_b(self.sensor_enabled)},"led_installed":{_b(self.led_installed)},'
                f'"buzzer_installed":{_b(self.buzzer_installed)},"comm_fault":{_b(self.comm_fault)},'
                f'"watchdog_trips":{self.watchdog_trips}}}')

    def disconnected(self, now):
        self.connected_flag = False
        if self.state == "MOVING" or self.sim_move != "none":
            self.comm_fault = True
        self.sim_move = "none"
        self.model_running = False
        self.follow = False
        self.follow_target = -1
        self._stop()

    def build_status(self, now: int) -> str:
        target = str(self.follow_target) if self.follow_target >= 0 else "null"
        dist = str(self.distance_cm) if self.sensor_enabled and self.distance_cm >= 0 else "null"
        reason = f'"{self.emergency_reason}"' if self.state == "EMERGENCY" else "null"
        last_id = f'"{self.last_command_id}"' if self.last_command_id else "null"
        src = ("both" if self.local_obstacle and self.remote_obstacle else "sensor" if self.local_obstacle
               else "camera" if self.remote_obstacle else "none")
        # the pattern being played (not the on/off phase at this instant)
        buz = (("emergency" if not self.muted and ((now - self.emergency_start) & MASK) < self.emergency_buzz_max_ms
                else "off") if self.state == "EMERGENCY"
               else ("fall_alert" if self._fall_buzzer_active(now) else "off") if self.fall_alert
               else BUZ_NAMES[self.buzzer_mode])
        # the LED pattern actually shown (same priority as _update_indicators)
        led = ("on" if self.state == "EMERGENCY" else "blink_fall" if self.fall_alert
               else "blink_obstacle" if self.obstacle_active
               else LED_NAMES[self.led_mode] if self.led_mode else "on" if self.model_running
               else "blink_following" if self.follow else "off")
        return (f'{{"version":3,"type":"status","state":"{self.state_name()}",'
                f'"motors_stopped":{_b(self.left == 0 and self.right == 0)},"left":{self.left},"right":{self.right},'
                f'"motors_enabled":{_b(self.motors_enabled)},"follow_mode":{_b(self.follow)},"target_id":{target},'
                f'"emergency_active":{_b(self.state == "EMERGENCY")},"emergency_reason":{reason},'
                f'"obstacle_detected":{_b(self.obstacle_active)},"obstacle_source":"{src}",'
                f'"sensor_installed":{_b(self.sensor_enabled)},"distance_cm":{dist},'
                f'"watchdog_ok":{_b(not self.comm_fault)},"watchdog_trips":{self.watchdog_trips},'
                f'"led":"{led}","buzzer":"{buz}","simulated_movement":"{self.sim_move}",'
                f'"model_running":{_b(self.model_running)},"fall_alert":{_b(self.fall_alert)},'
                f'"last_command_id":{last_id},"uptime_s":{((now - self.boot) & MASK) // 1000}}}')

    def _push_status(self, now):
        self._push(self.build_status(now))
        self.last_status = now

    def _ack_error(self, cid, seq, code, field=None) -> str:
        self.rejected += 1
        head = f'{{"version":3,"type":"ack","command_id":"{cid}"' if cid else '{"version":3,"type":"ack","command_id":null'
        out = head + f',"seq":{seq},"ok":false,"error":"{code}"'
        if field:
            out += f',"field":"{field}"'
        return out + "}"

    def overflow(self) -> str:
        return self._ack_error(None, 0, "TOO_LONG")

    def on_distance(self, cm: int, now: int) -> None:
        if not self.sensor_enabled:
            return
        was_active = self.obstacle_active
        was_local = self.local_obstacle
        self.distance_cm = cm
        if 0 <= cm < self.stop_cm:
            self.local_obstacle = True
        elif cm < 0 or cm > self.clear_cm:
            self.local_obstacle = False
        if self.local_obstacle and not was_local:
            self._push(f'{{"version":3,"type":"event","event":"obstacle","source":"sensor","distance_cm":{cm}}}')
            if not was_active and self.state != "EMERGENCY":
                self._start_buzzer(1, now)
        if self.local_obstacle and self.state == "MOVING" and (self.follow or self.left + self.right > 0):
            self._stop()
        if self.local_obstacle and self.sim_move == "forward":
            self.sim_move = "none"
        if was_active and not self.obstacle_active:
            self._push('{"version":3,"type":"event","event":"obstacle_clear"}')
        self._update_indicators(now)

    def handle(self, line: str, now: int) -> str:
        if len(line) > MAX_LINE:
            return self.overflow()
        o = parse_json_object(line)
        if o is None:
            return self._ack_error(None, 0, "BAD_JSON")
        fid, fseq, fver, ftype = o.get("command_id"), o.get("seq"), o.get("version"), o.get("type")
        cid = fid.s if (fid is not None and fid.type == "str" and valid_id(fid.s)) else None
        eseq = fseq.i if (fseq is not None and fseq.type == "int" and 0 <= fseq.i <= 999999999) else 0
        if fver is None:
            return self._ack_error(cid, eseq, "MISSING_FIELD", "version")
        if fver.type != "int" or fver.i != PROTOCOL_VERSION:
            return self._ack_error(cid, eseq, "BAD_VERSION")
        if fid is None:
            return self._ack_error(cid, eseq, "MISSING_FIELD", "command_id")
        if cid is None:
            return self._ack_error(cid, eseq, "BAD_FIELD", "command_id")
        if fseq is None:
            return self._ack_error(cid, eseq, "MISSING_FIELD", "seq")
        if fseq.type != "int" or not 1 <= fseq.i <= 999999999:
            return self._ack_error(cid, eseq, "BAD_FIELD", "seq")
        if fseq.i <= self.last_seq:
            return self._ack_error(cid, eseq, "BAD_SEQ")
        self.last_seq = fseq.i
        if ftype is None:
            return self._ack_error(cid, eseq, "MISSING_FIELD", "type")
        if ftype.type != "str":
            return self._ack_error(cid, eseq, "BAD_FIELD", "type")
        if ftype.s not in TYPES:
            return self._ack_error(cid, eseq, "UNKNOWN_TYPE")
        required, optional = TYPES[ftype.s]
        for key in o:
            if key not in COMMON and key not in required and key not in optional:
                return self._ack_error(cid, eseq, "UNKNOWN_FIELD", key)
        for key in required:
            if key not in o:
                return self._ack_error(cid, eseq, "MISSING_FIELD", key)
        t = ftype.s
        bad = err = None
        wheels = None
        want_status = False
        named = named_result = None
        lim = self.max_speed

        if t == "ping":
            pass
        elif t == "status_request":
            want_status = True
        elif t == "motor":
            l, r, d = o["left"], o["right"], o["duration_ms"]
            if not _str_one_of(o["mode"], ("manual",)):
                bad = "mode"
            elif not _int_in(l, -100, 100):
                bad = "left"
            elif not _int_in(r, -100, 100):
                bad = "right"
            elif not _int_in(d, self.min_move_ms, self.max_move_ms):
                bad = "duration_ms"
            elif self.state == "EMERGENCY":
                err = "EMERGENCY_ACTIVE"
            elif self.fall_alert and (l.i != 0 or r.i != 0):
                err = "FALL_ALERT_ACTIVE"
            elif self.follow:
                err = "FOLLOW_ACTIVE"
            else:
                lv, rv = _clamp(l.i, lim), _clamp(r.i, lim)
                if self.obstacle_active and lv + rv > 0:
                    err = "OBSTACLE_AHEAD"
                else:
                    self.last_valid = now
                    self.sim_move = "none"
                    self._drive(lv, rv, now, d.i)
                    self.comm_fault = False
                    wheels = (lv, rv)
        elif t == "stop":
            self._stop()
            self.sim_move = "none"
            self.follow = False
            self.follow_target = -1
            self.comm_fault = False
        elif t == "follow_mode":
            en, tg = o["enabled"], o.get("target_id")
            if en.type != "bool":
                bad = "enabled"
            elif tg is not None and not _int_in(tg, 0, 65535):
                bad = "target_id"
            elif en.b and tg is None:
                return self._ack_error(cid, eseq, "MISSING_FIELD", "target_id")
            elif en.b:
                if self.state == "EMERGENCY":
                    err = "EMERGENCY_ACTIVE"
                elif self.fall_alert:
                    err = "FALL_ALERT_ACTIVE"
                else:
                    if not self.follow:
                        self._start_buzzer(1, now)
                    self.sim_move = "none"
                    self.follow = True
                    self.follow_target = tg.i
                    self.comm_fault = False
            else:
                self._stop()
                self.follow = False
                self.follow_target = -1
                self.comm_fault = False
        elif t == "follow":
            tg, l, r, d, c = o["target_id"], o["left"], o["right"], o["duration_ms"], o["confidence"]
            if not _int_in(tg, 0, 65535):
                bad = "target_id"
            elif not _int_in(l, -100, 100):
                bad = "left"
            elif not _int_in(r, -100, 100):
                bad = "right"
            elif not _int_in(d, self.min_move_ms, self.max_move_ms):
                bad = "duration_ms"
            elif c.type not in ("int", "num") or c.d < 0.0 or c.d > 1.0:
                bad = "confidence"
            elif self.state == "EMERGENCY":
                err = "EMERGENCY_ACTIVE"
            elif self.fall_alert:
                err = "FALL_ALERT_ACTIVE"
            elif not self.follow:
                err = "FOLLOW_NOT_ENABLED"
            elif tg.i != self.follow_target:
                err = "TARGET_MISMATCH"
            elif c.d < self.follow_min_confidence:
                self._stop()
                err = "LOW_CONFIDENCE"
            else:
                lv, rv = _clamp(l.i, lim), _clamp(r.i, lim)
                if self.obstacle_active and (lv != 0 or rv != 0):
                    err = "OBSTACLE_AHEAD"
                else:
                    self.last_valid = now
                    self._drive(lv, rv, now, d.i)
                    self.comm_fault = False
                    wheels = (lv, rv)
        elif t == "led":
            st, iv = o["state"], o.get("interval_ms")
            if not _str_one_of(st, ("on", "off", "blink")):
                bad = "state"
            elif iv is not None and not _int_in(iv, 100, 2000):
                bad = "interval_ms"
            else:
                self.led_mode = {"on": 1, "blink": 2}.get(st.s, 0)
                self.led_interval = iv.i if iv is not None else 500
        elif t == "buzzer":
            pt = o["pattern"]
            if not _str_one_of(pt, BUZ_NAMES):
                bad = "pattern"
            else:
                self._start_buzzer(BUZ_NAMES.index(pt.s), now)
        elif t == "mute":
            if self.state == "EMERGENCY":
                self.muted = True
            if self.fall_alert:
                self.fall_muted = True
            self.buzzer_mode = 0
        elif t == "emergency_stop":
            rs = o["reason"]
            if rs.type != "str" or not valid_id(rs.s):
                bad = "reason"
            else:
                self._enter_emergency(rs.s, now)
        elif t == "clear_emergency":
            oc = o["operator_confirmed"]
            if oc.type != "bool":
                bad = "operator_confirmed"
            elif not oc.b:
                err = "NEEDS_CONFIRM"
            else:
                self._stop()
                self.sim_move = "none"
                self.state = "IDLE"
                self.follow = False
                self.follow_target = -1
                self.muted = False
                self.buzzer_mode = 0
                self.led_mode = 0
                self.emergency_reason = ""
        elif t == "obstacle":
            dc = o["distance_cm"]
            if not _str_one_of(o["source"], ("camera",)):
                bad = "source"
            elif not _int_in(dc, 0, 999):
                bad = "distance_cm"
            else:
                was_active = self.obstacle_active
                self.remote_obstacle = True
                self.remote_set = now
                if not was_active and self.state != "EMERGENCY":
                    self._start_buzzer(1, now)
                if self.state == "MOVING" and (self.follow or self.left + self.right > 0):
                    self._stop()
                if self.sim_move == "forward":
                    self.sim_move = "none"
        elif t == "obstacle_clear":
            was_active = self.obstacle_active
            self.remote_obstacle = False
            if was_active and not self.obstacle_active:
                self._push('{"version":3,"type":"event","event":"obstacle_clear"}')
        elif t == "command":
            nm = o["command"]
            if nm.type != "str":
                bad = "command"
            elif nm.s not in NAMED:
                err = "UNKNOWN_COMMAND"
            else:
                named = nm.s
                err, bad, want_status, named_result = self._named(nm.s, o.get("duration_ms"), now)

        if bad:
            return self._ack_error(cid, eseq, "BAD_FIELD", bad)
        if err:
            return self._ack_error(cid, eseq, err)
        self.last_valid = now
        self.last_command_id = cid
        self._update_indicators(now)
        if wheels is not None:
            reply = (f'{{"version":3,"type":"ack","command_id":"{cid}","seq":{eseq},"ok":true,'
                     f'"left":{wheels[0]},"right":{wheels[1]}}}')
        elif named is not None:
            reply = (f'{{"version":3,"type":"ack","command_id":"{cid}","seq":{eseq},"ok":true,'
                     f'"command":"{named}","result":"{named_result}"}}')
        else:
            reply = f'{{"version":3,"type":"ack","command_id":"{cid}","seq":{eseq},"ok":true}}'
        if want_status:
            self._push_status(now)
        return reply

    def _named(self, name: str, dur, now: int):
        """Mirror of RobotCore::handleNamed. Returns (err, bad, want_status, result)."""
        mv = SIM_NAMES.get(name)
        if dur is not None and (mv is None or not _int_in(dur, self.min_move_ms, self.max_move_ms)):
            return None, "duration_ms", False, ""
        if mv is not None:
            if self.state == "EMERGENCY":
                return "EMERGENCY_ACTIVE", None, False, ""
            if self.fall_alert:
                return "FALL_ALERT_ACTIVE", None, False, ""
            if self.follow:
                return "FOLLOW_ACTIVE", None, False, ""
            if mv == "forward" and self.obstacle_active:
                return "OBSTACLE_AHEAD", None, False, ""
            self._stop()
            self.sim_move, self.sim_start = mv, now
            self.sim_dur = dur.i if dur is not None else self.sim_move_default_ms
            self.comm_fault = False
            return None, None, False, f"SIMULATED_MOVEMENT_RECEIVED: {name}"
        if name == "STOP":
            self._stop()
            self.sim_move = "none"
            self.follow = False
            self.follow_target = -1
            self.comm_fault = False
            return None, None, False, "STOPPED"
        if name.startswith("LED_"):
            self.led_mode = {"LED_ON": 1, "LED_BLINK": 2}.get(name, 0)
            self.led_interval = 500
            return None, None, False, name
        if name.startswith("BUZZER_"):
            m = {"ON": 4, "SHORT_BEEP": 1, "WARNING": 2}.get(name[7:], 0)
            if m == 0 and self.state == "EMERGENCY":
                self.muted = True
            if m == 0 and self.fall_alert:
                self.fall_muted = True
            self._start_buzzer(m, now)
            return None, None, False, name
        if name == "STATUS":
            return None, None, True, "STATUS_SENT"
        if name == "MODEL_STARTED":
            self.model_running = True
            self.led_mode = 0
            return None, None, False, "MODEL_RUNNING"
        if name == "MODEL_STOPPED":
            self.model_running = False
            self.fall_alert = self.fall_muted = False
            self.led_mode = 0
            self.buzzer_mode = 0
            return None, None, False, "MODEL_STOPPED"
        if name == "FALL_DETECTED":
            if not self.fall_alert:                   # repeated events never restart the alert
                self.fall_alert, self.fall_muted, self.fall_buzz_reported = True, False, False
                self.fall_start = now
                self._stop()
                self.sim_move = "none"
                self.follow = False
                self.follow_target = -1
            return None, None, False, "FALL_ALERT_ACTIVE"
        if name == "FALL_CLEARED":
            was = self.fall_alert
            self.fall_alert = self.fall_muted = False
            return None, None, False, "FALL_ALERT_CLEARED" if was else "NO_FALL_ALERT"
        self._enter_emergency("operator", now)          # EMERGENCY_STOP
        return None, None, False, "EMERGENCY_LATCHED"

    def tick(self, now) -> None:
        if self.remote_obstacle and ((now - self.remote_set) & MASK) >= self.obstacle_hold_ms:
            self.remote_obstacle = False
            if not self.obstacle_active:
                self._push('{"version":3,"type":"event","event":"obstacle_clear"}')
        if self.state == "MOVING":
            if ((now - self.last_valid) & MASK) > self.watchdog_ms:
                self._stop()
                self.watchdog_trips += 1
                self.comm_fault = True
                self._push('{"version":3,"type":"event","event":"watchdog_stop"}')
            elif ((now - self.move_start) & MASK) >= self.move_dur:
                self._stop()
                self._push('{"version":3,"type":"event","event":"move_done"}')
        if self.sim_move != "none":
            if ((now - self.last_valid) & MASK) > self.watchdog_ms:
                self.sim_move = "none"
                self.watchdog_trips += 1
                self.comm_fault = True
                self._push('{"version":3,"type":"event","event":"watchdog_stop","simulated":true}')
            elif ((now - self.sim_start) & MASK) >= self.sim_dur:
                self.sim_move = "none"
                self._push('{"version":3,"type":"event","event":"move_done","simulated":true}')
        self._update_indicators(now)
        if self.connected_flag and ((now - self.last_status) & MASK) >= self.status_interval_ms:
            self._push_status(now)


class SimRobotServer:
    """TCP server around SimRobotCore. One active laptop connection; newest wins."""

    def __init__(self, host="127.0.0.1", port=3333, **core_kwargs):
        self.core = SimRobotCore(**core_kwargs)
        self.host, self.port = host, port
        self._srv: socket.socket | None = None
        self._client: socket.socket | None = None
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        self.t0 = time.monotonic()
        self.received: list[str] = []       # every line the robot received (for tests)

    def now(self) -> int:
        return int((time.monotonic() - self.t0) * 1000) & MASK

    def start(self) -> "SimRobotServer":
        self._stop.clear()
        self.core.begin(self.now())
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind((self.host, self.port))
        self.port = self._srv.getsockname()[1]
        self._srv.listen(2)
        self._srv.settimeout(0.05)
        for target in (self._accept_loop, self._tick_loop):
            t = threading.Thread(target=target, daemon=True)
            t.start()
            self._threads.append(t)
        log.info("Simulated robot (protocol v%d) listening on %s:%d", PROTOCOL_VERSION, self.host, self.port)
        return self

    def stop(self) -> None:
        self._stop.set()
        with self._lock:
            if self._client:
                self._drop(self._client)
        if self._srv:
            self._srv.close()
        for t in self._threads:
            t.join(timeout=1)

    def set_distance(self, cm: int) -> None:
        """Simulate a distance-sensor reading (needs sensor_enabled=True)."""
        with self._lock:
            self.core.on_distance(int(cm), self.now())
            self._flush()

    def _send(self, sock, text):
        try:
            sock.sendall((text + "\n").encode())
        except OSError:
            pass

    def _flush(self):
        for msg in self.core.pop_messages():
            if self._client:
                self._send(self._client, msg)

    def _drop(self, sock):
        try:
            sock.close()
        except OSError:
            pass
        if sock is self._client:
            self._client = None
            self.core.disconnected(self.now())

    def _accept_loop(self):
        while not self._stop.is_set():
            try:
                conn, _ = self._srv.accept()
            except (socket.timeout, OSError):
                continue
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self._lock:
                if self._client is not None:
                    self._send(self._client, '{"version":3,"type":"event","event":"replaced"}')
                    self._drop(self._client)
                self._client = conn
                self._send(conn, self.core.connected(self.now()))
            threading.Thread(target=self._read_loop, args=(conn,), daemon=True).start()

    def _read_loop(self, conn):
        buf = b""
        overflow = False
        try:
            conn.settimeout(0.05)
        except OSError:          # already replaced/closed by a newer connection
            return
        while not self._stop.is_set():
            try:
                data = conn.recv(1024)
            except socket.timeout:
                continue
            except OSError:
                data = b""
            if not data:
                with self._lock:
                    if conn is self._client:
                        self._drop(conn)
                return
            for byte in data:
                if byte == 10:   # '\n'
                    with self._lock:
                        if conn is not self._client:
                            return
                        if overflow:
                            self._send(conn, self.core.overflow())
                        else:
                            text = buf.decode("ascii", "replace")
                            if text.endswith("\r"):
                                text = text[:-1]
                            self.received.append(text)
                            self._send(conn, self.core.handle(text, self.now()))
                        self._flush()
                    buf, overflow = b"", False
                elif overflow:
                    continue
                elif len(buf) >= MAX_LINE:
                    overflow = True
                else:
                    buf += bytes([byte])

    def _tick_loop(self):
        while not self._stop.is_set():
            with self._lock:
                self.core.tick(self.now())
                self._flush()
            time.sleep(0.005)


def main() -> None:
    ap = argparse.ArgumentParser(description="Simulated SentinelAI robot (protocol v3)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=3333)
    ap.add_argument("--motors", action="store_true", help="pretend MOTORS_ENABLED=1")
    ap.add_argument("--sensor", action="store_true", help="pretend a distance sensor is fitted")
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S")
    srv = SimRobotServer(args.host, args.port, motors_enabled=args.motors, sensor_enabled=args.sensor).start()
    print(f"Simulated robot on {args.host}:{srv.port}. Ctrl+C to stop."
          + (" Type a distance in cm + Enter to simulate the sensor." if args.sensor else ""))
    if args.sensor:
        def reader():
            while True:
                try:
                    text = input().strip()
                except EOFError:
                    return
                try:
                    srv.set_distance(int(text))
                except ValueError:
                    print("type a whole number of cm, e.g. 20 (or -1 for nothing in range)")
        threading.Thread(target=reader, daemon=True).start()
    try:
        last = None
        while True:
            time.sleep(0.2)
            c = srv.core
            st = (c.state_name(), c.left, c.right, c.follow, c.obstacle_active, c.buzzer, c.led)
            if st != last:
                print(f"state={st[0]} motors=({st[1]},{st[2]}) follow={st[3]} obstacle={st[4]} "
                      f"buzzer={st[5]} led={st[6]}")
                last = st
    except KeyboardInterrupt:
        pass
    finally:
        srv.stop()


if __name__ == "__main__":
    main()
