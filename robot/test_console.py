"""SentinelAI ESP32 TEST CONSOLE - stage 1 communication test (no motors).

    python -m robot.test_console --host 192.168.1.50      # your ESP32's IP (Serial Monitor shows it)
    python -m robot.test_console                          # uses robot.host from config/settings.yaml
    python -m robot.test_console --sim                    # practise without hardware (simulated ESP32)
    python -m robot.test_console --host 192.168.1.50 --run-all   # send every command once, then a summary
    python -m robot.test_console --host 192.168.1.50 --hw-test   # only the LED + buzzer check, then exit

Type the number (or the name, e.g. LED_ON) and press Enter. FORWARD/BACKWARD/LEFT/RIGHT/STOP are
SIMULATED: the ESP32 only records and acknowledges them - no motor exists or moves. LED and BUZZER
items switch the real LED (GPIO 22) and buzzer (GPIO 23). AI events (MODEL_STARTED, FALL_DETECTED,
FALL_CLEARED, MODEL_STOPPED) are what the SentinelAI app sends; here YOU send them as deliberate tests.
EXIT sends STOP and disconnects cleanly.
Everything shown is also written to logs/esp32_test_<date>_<time>.log - send me that file.
"""
from __future__ import annotations

import argparse
import json
import logging
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

from .esp32_client import CommandResult, ESP32Client

ROOT = Path(__file__).resolve().parent.parent
# (command, label, section) - the section header is printed when it changes
MENU = [
    ("FORWARD", "FORWARD", "SIMULATED MOVEMENT - acknowledged only, no motor moves"),
    ("BACKWARD", "BACKWARD", None), ("LEFT", "LEFT", None), ("RIGHT", "RIGHT", None),
    ("STOP", "STOP (clears the simulated movement)", None),
    ("LED_ON", "LED ON", "PHYSICAL LED TEST (red LED, GPIO 22)"),
    ("LED_OFF", "LED OFF (ends the test; LED returns to its automatic state)", None),
    ("LED_BLINK", "LED BLINK", None),
    ("BUZZER_ON", "BUZZER ON (stops by itself after 60 s)", "PHYSICAL BUZZER TEST (GPIO 23 via transistor)"),
    ("BUZZER_OFF", "BUZZER OFF (also mutes an active alarm)", None),
    ("BUZZER_SHORT_BEEP", "SHORT BEEP", None), ("BUZZER_WARNING", "WARNING PATTERN (3 beeps)", None),
    ("MODEL_STARTED", "MODEL_STARTED  -> LED continuously ON",
     "AI EVENTS - sent by YOU as deliberate tests (operate the real LED + buzzer)"),
    ("FALL_DETECTED", "FALL_DETECTED  -> LED blinks + buzzer alert  [TEST - no real fall]", None),
    ("FALL_CLEARED", "FALL_CLEARED   -> alert off, LED ON again if the model runs", None),
    ("MODEL_STOPPED", "MODEL_STOPPED  -> LED and buzzer OFF", None),
    ("STATUS", "STATUS", "OTHER"),
    ("EMERGENCY_STOP", "EMERGENCY STOP (LED ON + alarm, latched until R)", None),
    ("EXIT", "EXIT (sends STOP, disconnects)", None),
]
EXTRA = [("H", "HARDWARE TEST: LED on/blink/off + short beep + warning (no AI model needed)"),
         ("R", "RESET EMERGENCY (asks for confirmation)"),
         ("T", "send INVALID test messages (bad JSON, unknown command, wrong version, too long)"),
         ("M", "show this menu again")]
INVALID_TESTS = [
    ("invalid JSON", "this is not json", "BAD_JSON"),
    ("unknown command", '{"version":3,"type":"command","command_id":"bad-cmd","seq":{seq},"command":"JUMP"}',
     "UNKNOWN_COMMAND"),
    ("wrong protocol version", '{"version":1,"type":"command","command_id":"bad-ver","seq":{seq},"command":"STOP"}',
     "BAD_VERSION"),
    ("missing field", '{"version":3,"type":"command","command_id":"bad-miss","seq":{seq}}', "MISSING_FIELD"),
    ("malformed command_id", '{"version":3,"type":"command","command_id":"bad id!","seq":{seq},"command":"STOP"}',
     "BAD_FIELD"),
    ("invalid value", '{"version":3,"type":"command","command_id":"bad-val","seq":{seq},"command":"LEFT",'
     '"duration_ms":99999}', "BAD_FIELD"),
    ("oversized message", '{"version":3,"type":"command","command_id":"big","seq":{seq},"command":"' + "X" * 220 + '"}',
     "TOO_LONG"),
]


class Console:
    def __init__(self, esp: ESP32Client, log_path: Path):
        self.esp = esp
        self.log_path = log_path
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self._log = open(log_path, "a", encoding="utf-8")
        self._lock = threading.Lock()
        self.results: list[CommandResult] = []
        self._was_connected = False
        esp.client.add_listener(self._on_robot_message)

    # ------------------------------------------------------------- output
    def say(self, text: str, to_screen: bool = True) -> None:
        with self._lock:
            if to_screen:
                print(text, flush=True)
            self._log.write(f"{datetime.now().isoformat(timespec='milliseconds')} {text}\n")
            self._log.flush()

    def _on_robot_message(self, reply) -> None:
        if reply.kind == "event":
            ev = reply.event
            if ev == "hello":
                self.say(f"\n[ESP32] hello: protocol {reply.data.get('protocol')}, state {reply.data.get('state')}, "
                         f"motors_enabled {reply.data.get('motors_enabled')}, "
                         f"comm_fault {reply.data.get('comm_fault')}")
                if reply.data.get("comm_fault"):
                    self.say("[ESP32] it reports that the previous connection was lost during a (simulated) "
                             "movement. Nothing was resumed. Send STOP to acknowledge.")
            else:
                self.say(f"\n[ESP32 event] {reply.raw}")
        else:
            self.say(reply.raw, to_screen=False)          # periodic status: log file only

    def watch_link(self, stop: threading.Event) -> None:
        """Tell the user when the link drops or comes back (runs in the background)."""
        last_report = 0.0
        while not stop.wait(0.3):
            c = self.esp.connected
            if c != self._was_connected:
                self._was_connected = c
                self.say("\n[link] CONNECTED to the ESP32" if c else
                         f"\n[link] CONNECTION LOST ({self.esp.last_error()}) - reconnecting by itself ...")
                last_report = time.monotonic()
            elif not c and time.monotonic() - last_report > 10:
                self.say(f"[link] still not connected: {self.esp.last_error() or 'no answer'}")
                last_report = time.monotonic()

    def header(self) -> None:
        h = self.esp.hello()
        st = self.esp.client.robot if self.esp.connected else {}
        conn = "CONNECTED" if self.esp.connected else "NOT CONNECTED"
        resp = "RESPONDING" if self.esp.responding() else "NOT RESPONDING"
        motors = "DISABLED (movement is simulated)" if not h.get("motors_enabled") else "ENABLED (!)"
        self.say("\n================================\n SENTINELAI ESP32 TEST CONSOLE\n================================")
        self.say(f"Connection: {conn}   ({self.esp.client.host}:{self.esp.client.port})")
        self.say(f"ESP32: {resp}" + (f"   state {st.get('state')}, simulated movement "
                                     f"{st.get('simulated_movement')}, LED {st.get('led')}, buzzer {st.get('buzzer')}"
                                     if st else ""))
        self.say(f"Motors: {motors}" + ("   EMERGENCY LATCHED - use R to reset" if st.get("emergency_active") else ""))
        if st:
            self.say(f"AI model (as told to the ESP32): {'RUNNING' if st.get('model_running') else 'not running'}"
                     f"   fall alert: {'ACTIVE' if st.get('fall_alert') else 'none'}")

    def menu(self) -> None:
        self.header()
        for i, (_, label, section) in enumerate(MENU, 1):
            if section:
                self.say(f"  -- {section} --")
            self.say(f"{i:>2}. {label}")
        for key, label in EXTRA:
            self.say(f" {key}. {label}")

    # ------------------------------------------------------------ actions
    def run_command(self, name: str) -> CommandResult:
        self.say(f"-> {name}")
        if name == "STATUS":
            st = self.esp.get_status()
            r = CommandResult("STATUS", st is not None, result="status received" if st else None,
                              error=None if st else (self.esp.last_error() or "no status received"))
            if st:
                self.say("<- status: " + json.dumps(st, indent=1))
        elif name == "EMERGENCY_STOP":
            r = self.esp.emergency_stop()
        elif name == "FALL_DETECTED":
            self.say("   (deliberate TEST event sent by you - not a fall detected by the AI)")
            r = self.esp.send_command(name)
        else:
            r = self.esp.send_command(name)
        if name != "STATUS" or not r.ok:
            self.say(("<- ACK " if r.ok else "<- ") + str(r))
        self.results.append(r)
        return r

    def reset_emergency(self, interactive: bool = True) -> CommandResult:
        if interactive:
            ans = input("Reset the emergency? Only after checking that everything is safe. Type YES: ").strip()
            if ans != "YES":
                self.say("reset cancelled")
                return CommandResult("RESET_EMERGENCY", False, error="cancelled by operator")
        self.say("-> RESET_EMERGENCY (clear_emergency, operator_confirmed: true)")
        r = self.esp.reset_emergency(confirmed=True)
        self.say(("<- ACK " if r.ok else "<- ") + str(r))
        self.results.append(r)
        return r

    def hardware_test(self) -> bool:
        """LED and buzzer check without the AI model. Watch the LED and listen."""
        steps = [("LED_ON", 1.5, "LED should be ON"), ("LED_BLINK", 2.5, "LED should BLINK (0.5 s)"),
                 ("LED_OFF", 1.0, "LED should be OFF"), ("BUZZER_SHORT_BEEP", 1.0, "one short BEEP"),
                 ("BUZZER_WARNING", 1.5, "three short beeps"), ("BUZZER_ON", 1.0, "continuous tone ..."),
                 ("BUZZER_OFF", 0.5, "... silent again")]
        ok = True
        self.say("\n-- HARDWARE TEST (LED + buzzer, no AI model) --")
        for name, pause, expect in steps:
            r = self.run_command(name)
            ok &= r.ok
            self.say(f"   expected now: {expect}")
            time.sleep(pause)
        self.say("Hardware test commands " + ("all acknowledged" if ok else "NOT all acknowledged") +
                 " - did the LED and the buzzer do what was expected? (only your eyes/ears can confirm)")
        return ok

    def ai_event_test(self) -> bool:
        """The AI state sequence, sent as deliberate tests."""
        plan = [("MODEL_STARTED", 2.0, "LED continuously ON"),
                ("FALL_DETECTED", 3.0, "LED blinking + buzzer beep-beep pattern [TEST]"),
                ("FALL_DETECTED", 1.0, "repeat: nothing changes (no second alert)"),
                ("FALL_CLEARED", 2.0, "buzzer stops, LED continuously ON again"),
                ("MODEL_STOPPED", 1.0, "LED and buzzer OFF")]
        ok = True
        self.say("\n-- AI EVENT TEST (deliberate test events, not real falls) --")
        for name, pause, expect in plan:
            r = self.run_command(name)
            ok &= r.ok
            self.say(f"   expected now: {expect}")
            time.sleep(pause)
        return ok

    def invalid_tests(self) -> bool:
        """Send deliberately wrong messages; the ESP32 must refuse each with the right error code."""
        all_ok = True
        for label, template, expect in INVALID_TESTS:
            n_before = self.esp.client.stats["err"]
            try:
                self.esp.client.send_test_line(template)
            except Exception as exc:
                self.say(f"-> {label}: not sent ({exc})")
                all_ok = False
                continue
            end = time.monotonic() + 1.5
            reply = None
            while time.monotonic() < end and reply is None:
                time.sleep(0.02)
                if self.esp.client.stats["err"] > n_before:      # a NEW refusal arrived: take the newest
                    reply = next((l for l in reversed(self.esp.client.last_raw_lines()) if '"ok":false' in l), None)
            if reply is not None and f'"error":"{expect}"' not in reply:
                self.say(f"-> {label}: wrong answer {reply}")
                reply = None
            good = reply is not None
            all_ok &= good
            self.say(f"-> {label:<22} expected {expect:<16} " + (f"<- {reply}" if good else
                     f"<- NOT received (errors before/after: {n_before}/{self.esp.client.stats['err']})"))
        r = self.esp.send_command("STATUS")      # the link must still work afterwards
        all_ok &= r.ok
        self.say(f"link still usable afterwards: {'yes' if r.ok else 'NO - ' + str(r.error)}")
        return all_ok

    def interactive(self) -> None:
        self.menu()
        while True:
            try:
                raw = input("\nEnter command: ").strip()
            except (EOFError, KeyboardInterrupt):
                raw = "EXIT"
            if not raw:
                continue
            key = raw.upper().replace(" ", "_")
            if key.isdigit() and 1 <= int(key) <= len(MENU):
                key = MENU[int(key) - 1][0]
            self.say(f"[you] {raw}", to_screen=False)
            if key == "H":
                self.hardware_test()
                continue
            if key in ("EXIT", "QUIT", "Q"):
                break
            if key == "M":
                self.menu()
            elif key in ("R", "RESET", "RESET_EMERGENCY"):
                self.reset_emergency()
            elif key == "T":
                self.invalid_tests()
            elif key in [m[0] for m in MENU]:
                self.run_command(key)
            else:
                self.say(f"Unknown choice - type a number 1-{len(MENU)}, a command name, H, R, T or M.")

    def run_all(self) -> int:
        """Every command once, with pauses so you can watch the LED and listen to the buzzer."""
        plan = [("STATUS", 0.5), ("FORWARD", 0.5), ("BACKWARD", 0.5), ("LEFT", 0.5), ("RIGHT", 0.5), ("STOP", 0.5),
                ("LED_ON", 1.5), ("LED_BLINK", 2.5), ("LED_OFF", 1.0), ("BUZZER_SHORT_BEEP", 1.0),
                ("BUZZER_WARNING", 1.5), ("BUZZER_ON", 1.0), ("BUZZER_OFF", 0.5), ("EMERGENCY_STOP", 2.0),
                ("LEFT", 0.3), ("BUZZER_OFF", 0.5)]
        expect_fail = {14: "EMERGENCY_ACTIVE"}       # LEFT during the emergency must be refused
        ok_ai = self.ai_event_test()
        ok = True
        for i, (name, pause) in enumerate(plan):
            r = self.run_command(name)
            want_fail = expect_fail.get(i)
            good = (not r.ok and want_fail in (r.error or "")) if want_fail else r.ok
            ok &= good
            if want_fail:
                self.say(f"   (expected: refused with {want_fail}) -> {'as expected' if good else 'NOT as expected'}")
            time.sleep(pause)
        ok &= self.reset_emergency(interactive=False).ok and ok_ai
        # fall alert blocks (simulated) movement until it is cleared
        self.say("\n-- fall alert blocks movement --")
        r1 = self.run_command("FALL_DETECTED")
        r2 = self.run_command("LEFT")
        good = r1.ok and not r2.ok and "FALL_ALERT_ACTIVE" in (r2.error or "")
        self.say(f"   (expected: LEFT refused with FALL_ALERT_ACTIVE) -> {'as expected' if good else 'NOT as expected'}")
        ok &= good and self.run_command("FALL_CLEARED").ok
        self.say("\n-- invalid messages (each must be refused) --")
        ok &= self.invalid_tests()
        st = self.esp.get_status() or {}
        ok &= st.get("emergency_active") is False and st.get("simulated_movement") == "none"
        self.say("\n===== SUMMARY =====")
        for r in self.results:
            self.say(f"  {'OK  ' if r.ok else 'FAIL'} {r.command:<18} {r.result or r.error or ''}"
                     + (f"  ({r.latency_ms:.1f} ms)" if r.latency_ms is not None else ""))
        lat = sorted(r.latency_ms for r in self.results if r.ok and r.latency_ms is not None)
        if lat:
            self.say(f"  ack latency: median {lat[len(lat) // 2]:.1f} ms, max {lat[-1]:.1f} ms")
        self.say("RESULT: " + ("ALL AS EXPECTED" if ok else "SOME STEPS FAILED - see above"))
        return 0 if ok else 1

    def close(self) -> None:
        self.say("Sending STOP and disconnecting ...")
        self.esp.disconnect()
        self.say(f"Disconnected. Log saved to {self.log_path}")
        self._log.close()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default=None,
                    help="ESP32 IP address, or 'auto' to find it on the Wi-Fi (default: robot.host in settings.yaml)")
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--sim", action="store_true", help="start a simulated ESP32 on this laptop (no hardware)")
    ap.add_argument("--run-all", action="store_true", help="send every command once and print a summary")
    ap.add_argument("--hw-test", action="store_true", help="only the LED + buzzer test, then exit")
    ap.add_argument("--timeout", type=float, default=1.0, help="seconds to wait for each acknowledgement")
    args = ap.parse_args()
    logging.basicConfig(level=logging.ERROR, format="%(message)s")
    sim = None
    host, port = args.host, args.port
    if args.sim:
        from .sim_robot import SimRobotServer
        sim = SimRobotServer(port=0).start()
        host, port = "127.0.0.1", sim.port
        print(f"Simulated ESP32 running on {host}:{port}")
    if host is None or port is None:
        sys.path.insert(0, str(ROOT))
        from src.config import load_config
        cfg = load_config()
        host = host or cfg.robot.host
        port = port or cfg.robot.port
    log_path = ROOT / "logs" / f"esp32_test_{datetime.now():%Y%m%d_%H%M%S}.log"
    esp = ESP32Client(host, port, ack_timeout_s=args.timeout, reconnect_delay_s=1.0, reconnect_max_delay_s=3.0)
    con = Console(esp, log_path)
    if esp.client.auto:
        con.say(f"Searching for the ESP32 on this Wi-Fi (port {port}) ...")
    else:
        con.say(f"Connecting to the ESP32 at {host}:{port} ...")
    stop = threading.Event()
    if not esp.connect(timeout=15.0 if esp.client.auto else 6.0):
        con.say(f"Could not connect: {esp.last_error() or 'no answer'}")
        con.say("Check: ESP32 powered and Serial Monitor shows '[wifi] connected'; the IP is correct; laptop and "
                "ESP32 on the same Wi-Fi (docs/STAGE1_COMM_TEST.md section 8). Still trying in the background.")
        if args.run_all or args.hw_test:
            con.close()
            if sim:
                sim.stop()
            return 2
    threading.Thread(target=con.watch_link, args=(stop,), daemon=True).start()
    code = 0
    try:
        if args.hw_test:
            code = 0 if con.hardware_test() else 1
        elif args.run_all:
            code = con.run_all()
        else:
            con.interactive()
    finally:
        stop.set()
        con.close()
        if sim:
            sim.stop()
    return code


if __name__ == "__main__":
    sys.exit(main())
