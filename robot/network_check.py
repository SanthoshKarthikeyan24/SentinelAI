"""Check that the laptop can reach the phone camera AND the robot at the same time.

Switching the laptop to the ESP32's own Wi-Fi can silently cut it off from
the phone camera (and the reverse). This tool reads camera frames and pings
the robot simultaneously for a few seconds and reports both.

    python -m robot.network_check --camera http://192.168.43.1:8080/video --robot 192.168.43.50
    python -m robot.network_check                      # uses camera.source and robot.host from settings.yaml
Results: evaluation/communication/network_check_<time>.json
"""
from __future__ import annotations

import argparse
import json
import socket
import statistics
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def check_camera(source, seconds: float, out: dict) -> None:
    import cv2
    t0 = time.monotonic()
    cap = cv2.VideoCapture(source)
    out["opened"] = bool(cap.isOpened())
    frames, gaps, last = 0, [], None
    while out["opened"] and time.monotonic() - t0 < seconds:
        ok, img = cap.read()
        now = time.monotonic()
        if not ok:
            out.setdefault("read_failures", 0)
            out["read_failures"] += 1
            if out["read_failures"] > 50:
                break
            continue
        frames += 1
        if last is not None:
            gaps.append(now - last)
        last = now
        out["resolution"] = f"{img.shape[1]}x{img.shape[0]}"
    cap.release()
    dur = max(time.monotonic() - t0, 1e-6)
    out["frames"] = frames
    out["fps"] = round(frames / dur, 2)
    out["longest_gap_s"] = round(max(gaps), 3) if gaps else None
    out["ok"] = frames > 0 and (max(gaps) if gaps else 0) < 2.0


def check_robot(host: str, port: int, seconds: float, out: dict) -> None:
    from .comm_test import RawLink
    try:
        link = RawLink(host, port, timeout=3)
    except OSError as exc:
        out.update(ok=False, error=f"cannot connect: {exc}")
        return
    hello, _ = link.wait_event("hello", 2.0)
    out["hello"] = hello
    rtts, sent, ok = [], 0, 0
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < seconds:
            r, ms = link.request("ping", timeout=0.5)
            sent += 1
            if r and r.get("ok") is True:
                ok += 1
                rtts.append(ms)
            time.sleep(0.1)
    except OSError as exc:
        out["error"] = str(exc)
    link.close()
    rtts.sort()
    out.update(sent=sent, acknowledged=ok,
               rtt_mean_ms=round(statistics.mean(rtts), 1) if rtts else None,
               rtt_p95_ms=round(rtts[int(0.95 * (len(rtts) - 1))], 1) if rtts else None,
               rtt_max_ms=round(rtts[-1], 1) if rtts else None,
               ok=bool(hello) and sent > 0 and ok == sent)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", default=None, help="camera URL / webcam index")
    ap.add_argument("--robot", default=None, metavar="HOST[:PORT]")
    ap.add_argument("--seconds", type=float, default=10.0)
    args = ap.parse_args()
    sys.path.insert(0, str(ROOT))
    from src.config import load_config, parse_source
    cfg = load_config()
    camera = parse_source(args.camera if args.camera is not None else cfg.camera.source)
    host, port = cfg.robot.host, cfg.robot.port
    if args.robot:
        host, _, p = args.robot.partition(":")
        port = int(p) if p else port
    print(f"Reading camera {camera} and pinging robot {host}:{port} together for {args.seconds:.0f} s ...")
    cam, rob = {}, {}
    threads = [threading.Thread(target=check_camera, args=(camera, args.seconds, cam)),
               threading.Thread(target=check_robot, args=(host, port, args.seconds, rob))]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    try:
        laptop_ip = socket.gethostbyname(socket.gethostname())
    except OSError:
        laptop_ip = "unknown"
    result = {"time": datetime.now().isoformat(timespec="seconds"), "laptop_ip": laptop_ip,
              "camera_source": str(camera), "camera": cam, "robot": {"host": host, "port": port, **rob}}
    result["both_ok"] = bool(cam.get("ok")) and bool(rob.get("ok"))
    d = ROOT / "evaluation" / "communication"
    d.mkdir(parents=True, exist_ok=True)
    path = d / f"network_check_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(f"Camera: {'OK' if cam.get('ok') else 'PROBLEM'}  {cam}")
    print(f"Robot : {'OK' if rob.get('ok') else 'PROBLEM'}  {rob}")
    print("RESULT:", "camera and robot both reachable" if result["both_ok"] else
          "NOT both reachable - see docs/NETWORK.md", f"(saved {path})")
    return 0 if result["both_ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
