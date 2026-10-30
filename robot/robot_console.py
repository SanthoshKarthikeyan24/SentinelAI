"""Interactive console to talk to the robot by hand (bench testing, protocol v3).

    python -m robot.robot_console --host 192.168.43.50
    python -m robot.robot_console --sim            # start a simulated robot and talk to it

Commands (type 'help'):
    ping | status | stop | mute | estop [reason] | clear     (clear = clear_emergency, asks to confirm)
    forward|backward|left|right [speed]      one press-and-hold step (expires after manual_hold_ms)
    motor <left> <right> [ms]                e.g.  motor 30 -30 400   (wheel speeds -100..100)
    follow <id> | follow off                 robot follow mode for track id (beep + LED blink)
    led on|off|blink [interval_ms]           red LED
    buzzer off|short_beep|warning|emergency  buzzer pattern
    obstacle [cm]  |  obstacle_clear         camera obstacle report / clear
    safety            show the safety supervisor's decision
    latency [n]       measure n ping round trips
    log               show the last raw lines received
    quit
Motion goes through the same safety supervisor as the AI (RobotLink): it is refused during
an emergency, a communication fault, while following, or (forward) with an obstacle.
With MOTORS_ENABLED 0 in the firmware no wheel turns - only the robot's reported state changes.
"""
from __future__ import annotations

import argparse
import json
import logging
import statistics
import sys
import time
from pathlib import Path

from .commands import CommandError
from .robot_logic import RobotLink
from .wifi_client import RobotClient, RobotNotConnected, RobotTimeout

HELP = __doc__.split("Commands (type 'help'):")[1]


def _show(reply) -> str:
    return reply.raw if reply is not None else "no ack"


def run(host: str, port: int) -> None:
    client = RobotClient(host, port)
    link = RobotLink(client).start()
    print(f"Connecting to {host}:{port} ...")
    if client.wait_connected(5) and client.wait_status(3):
        h = client.hello
        print(f"Connected. Robot state {client.robot_state}, protocol v{h.get('protocol')}, "
              f"motors {'ENABLED' if h.get('motors_enabled') else 'disabled'}, "
              f"obstacle sensor {'fitted' if h.get('sensor_installed') else 'not fitted'}"
              + (" - robot reports a COMMUNICATION FAULT from before (type 'stop')" if h.get("comm_fault") else ""))
    else:
        print(f"Not connected yet ({client.last_error}); still retrying in the background.")
    try:
        while True:
            try:
                line = input("robot> ").strip()
            except EOFError:
                break
            if not line:
                continue
            parts = line.split()
            cmd, args = parts[0].lower(), parts[1:]
            try:
                if cmd in ("quit", "exit", "q"):
                    break
                elif cmd == "help":
                    print(HELP)
                elif cmd in ("forward", "backward", "back", "left", "right"):
                    d = link.manual("backward" if cmd == "back" else cmd, int(args[0]) if args else None)
                    print(("robot: " + _show(d.ticket.wait(1.0))) if d.sent else f"NOT sent: {d.reason}")
                elif cmd == "motor":
                    if len(args) < 2:
                        print("usage: motor <left> <right> [ms]")
                        continue
                    d = link.drive(int(args[0]), int(args[1]), int(args[2]) if len(args) > 2 else None)
                    print(("robot: " + _show(d.ticket.wait(1.0))) if d.sent else f"NOT sent: {d.reason}")
                elif cmd == "follow":
                    if not args or args[0].lower() == "off":
                        print(_show(link.follow_mode(False).wait(1.0)))
                    else:
                        print(_show(link.follow_mode(True, int(args[0])).wait(1.0)))
                elif cmd == "led":
                    print(_show(link.set_led(args[0] if args else "", int(args[1]) if len(args) > 1 else None)))
                elif cmd == "buzzer":
                    print(_show(link.buzzer(args[0] if args else "")))
                elif cmd == "obstacle":
                    print(_show(client.send("obstacle", source="camera", distance_cm=int(args[0]) if args else 0)))
                elif cmd == "obstacle_clear":
                    print(_show(client.send("obstacle_clear")))
                elif cmd == "clear":
                    ans = input("Clear the robot emergency? Check on the person first. Type YES: ")
                    print(_show(link.clear_emergency(True)) if ans.strip() == "YES" else "cancelled")
                elif cmd in ("estop", "emergency", "fall"):
                    link.emergency(args[0] if args else ("fall_confirmed" if cmd == "fall" else "operator"))
                    print("emergency_stop queued with top priority (retried until acknowledged)")
                elif cmd == "status":
                    client.send("status_request")
                    client.wait_status(1.0)
                    print(json.dumps(client.robot, indent=1))
                elif cmd == "stop":
                    print(_show(link.stop("console")))
                elif cmd in ("ping", "mute"):
                    print(_show(client.send(cmd)))
                elif cmd == "safety":
                    print(json.dumps(link.safety().to_dict(), indent=1))
                elif cmd == "latency":
                    n = int(args[0]) if args else 50
                    rtts = []
                    for _ in range(n):
                        t0 = time.perf_counter()
                        client.send("ping")
                        rtts.append((time.perf_counter() - t0) * 1000)
                    rtts.sort()
                    print(f"{n} pings: mean {statistics.mean(rtts):.1f} ms, median {statistics.median(rtts):.1f} ms,"
                          f" p95 {rtts[int(0.95 * (n - 1))]:.1f} ms, max {rtts[-1]:.1f} ms")
                elif cmd == "log":
                    print("\n".join(client.last_raw_lines()[-15:]))
                elif cmd == "link":
                    print(json.dumps(link.status(), indent=1, default=str))
                else:
                    print("unknown command - type 'help'")
            except (RobotNotConnected, RobotTimeout, CommandError, ValueError) as exc:
                print(f"error: {exc}")
    finally:
        link.close()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default=None)
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--sim", action="store_true", help="start a simulated robot and connect to it")
    args = ap.parse_args()
    logging.basicConfig(level=logging.WARNING, format="%(asctime)s %(message)s", datefmt="%H:%M:%S")
    sim = None
    host, port = args.host, args.port
    if args.sim:
        from .sim_robot import SimRobotServer
        sim = SimRobotServer(port=0).start()
        host, port = "127.0.0.1", sim.port
    if host is None or port is None:
        sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
        from src.config import load_config
        cfg = load_config()
        host = host or cfg.robot.host
        port = port or cfg.robot.port
    try:
        run(host, port)
    finally:
        if sim:
            sim.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
