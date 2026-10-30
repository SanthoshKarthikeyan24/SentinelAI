"""SentinelAI - start the live AI monitoring application (Phase 1, no robot needed).

Examples (run from the project folder with the virtual environment active):
    python run_app.py                                    # webcam 0, web dashboard
    python run_app.py --source http://192.168.1.5:8080/video
    python run_app.py --source data/raw/public_datasets/GMDCSA24/"Subject 1"/Fall/01.mp4 --realtime
    python run_app.py --mode window                      # OpenCV window instead of dashboard
    python run_app.py --mode headless --max-frames 300   # no UI, just logs (testing)
    python run_app.py --robot 192.168.43.50              # link to the ESP32 at this address
    python run_app.py --robot auto                       # find the ESP32 on the Wi-Fi by itself (default in settings)
    python run_app.py --no-robot                         # AI only, never contact the ESP32
    python run_app.py --robot-sim                        # link to a simulated robot (no hardware)

Then open http://127.0.0.1:5000 in a browser (dashboard mode).
Press Ctrl+C to stop.
"""
from __future__ import annotations

import argparse
import logging
import sys
import time
import webbrowser

from src.config import ConfigError, load_config, parse_source


def _install_stop_signals() -> None:
    """SIGTERM (and Ctrl+Break on Windows) stop the app the same safe way as Ctrl+C:
    following off, robot told to stop, link closed, log written."""
    import signal

    def _stop(signum, frame):
        raise KeyboardInterrupt
    for name in ("SIGTERM", "SIGBREAK"):
        if hasattr(signal, name):
            try:
                signal.signal(getattr(signal, name), _stop)
            except (ValueError, OSError):
                pass


def main() -> int:
    ap = argparse.ArgumentParser(description="SentinelAI live monitoring")
    ap.add_argument("--config", default=None, help="path to settings.yaml")
    ap.add_argument("--source", default=None, help="webcam index, stream URL or video file")
    ap.add_argument("--mode", choices=["dashboard", "window", "headless"], default="dashboard")
    ap.add_argument("--max-frames", type=int, default=0, help="stop after N frames (0 = run until stopped)")
    ap.add_argument("--realtime", action="store_true", help="play video files at native speed")
    ap.add_argument("--no-objects", action="store_true", help="disable object detection (faster)")
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--no-browser", action="store_true")
    ap.add_argument("--robot", default=None, metavar="HOST[:PORT]",
                    help="enable the ESP32 link to this address (overrides settings.yaml)")
    ap.add_argument("--no-robot", action="store_true",
                    help="do not contact the ESP32 even if robot.enabled is true in settings.yaml")
    ap.add_argument("--robot-sim", action="store_true",
                    help="start a simulated robot inside this program and connect to it (no hardware)")
    args = ap.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s",
                        datefmt="%H:%M:%S")
    overrides: dict = {"camera": {}, "detection": {}, "dashboard": {}}
    if args.source is not None:
        overrides["camera"]["source"] = parse_source(args.source)
    if args.realtime:
        overrides["camera"]["file_realtime"] = True
    if args.no_objects:
        overrides["detection"]["object_detection_enabled"] = False
    if args.port:
        overrides["dashboard"]["port"] = args.port
    sim = None
    if args.robot_sim:
        from robot.sim_robot import SimRobotServer
        sim = SimRobotServer(port=0).start()
        args.robot = f"127.0.0.1:{sim.port}"
        print(f"  Simulated robot running on {args.robot}")
    if args.no_robot:
        overrides["robot"] = {"enabled": False}
    elif args.robot:
        host, _, port = args.robot.partition(":")
        overrides["robot"] = {"enabled": True, "host": host}
        if port:
            overrides["robot"]["port"] = int(port)
    try:
        cfg = load_config(args.config, overrides)
    except ConfigError as exc:
        print(f"Configuration error: {exc}", file=sys.stderr)
        return 2

    from src.app_runner import AppRunner
    try:
        runner = AppRunner(cfg, max_frames=args.max_frames)
    except FileNotFoundError as exc:
        print(f"Missing file: {exc}", file=sys.stderr)
        return 2
    _install_stop_signals()
    runner.start()
    try:
        if args.mode == "dashboard":
            from src.dashboard.server import create_app
            url = f"http://{cfg.dashboard.host}:{cfg.dashboard.port}"
            print(f"\n  Dashboard: {url}   (Ctrl+C to stop)\n")
            if not args.no_browser:
                webbrowser.open(url)
            create_app(runner).run(host=cfg.dashboard.host, port=cfg.dashboard.port, threaded=True,
                                   use_reloader=False)
        elif args.mode == "window":
            import cv2
            while not runner.finished:
                frame = runner.latest_frame()
                if frame is not None:
                    cv2.imshow("SentinelAI (q = quit, a = acknowledge alerts)", frame)
                key = cv2.waitKey(30) & 0xFF
                if key == ord("q"):
                    break
                if key == ord("a"):
                    runner.acknowledge()
            cv2.destroyAllWindows()
        else:
            last = 0.0
            while not runner.finished:
                time.sleep(0.2)
                if time.time() - last > 5:
                    st = runner.status()
                    logging.info("frames=%d fps=%.1f camera=%s people=%d alerts=%d",
                                 st["frames_processed"], st["fps"], st["camera"]["status"],
                                 len(st["persons"]), len(st["alerts_active"]))
                    last = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        runner.stop()
        if sim is not None:
            sim.stop()
    if runner.error:
        print(f"Stopped with error: {runner.error}", file=sys.stderr)
        return 1
    st = runner.status()
    print(f"Processed {st['frames_processed']} frames. Events logged to {runner.logger.jsonl_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
