"""Laptop self-check: Python packages, cameras, model speed, settings, network.

Run on the laptop (double-click check_system.bat, or):
    .venv\\Scripts\\python.exe tools\\check_system.py
Writes reports/system_check.json (no images, no passwords) and prints a short summary.

Camera check: opens indexes 0-3 and reads 15 frames from each. A camera whose picture never
changes (identical frames) or is almost black is flagged - typical of a *virtual* camera
(OBS, Snap Camera, phone-link apps) rather than the real webcam.
"""
from __future__ import annotations

import importlib
import json
import platform
import socket
import sys
import time
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
OUT = ROOT / "reports" / "system_check.json"


def packages() -> dict:
    out = {}
    for name in ("numpy", "cv2", "onnxruntime", "sklearn", "scipy", "joblib", "flask", "yaml", "psutil"):
        try:
            m = importlib.import_module(name)
            out[name] = getattr(m, "__version__", "ok")
        except Exception as exc:          # report, never crash
            out[name] = f"MISSING ({type(exc).__name__})"
    return out


def cameras(max_index: int = 4) -> list[dict]:
    import cv2
    import numpy as np
    res = []
    backend = cv2.CAP_DSHOW if sys.platform.startswith("win") else cv2.CAP_ANY
    for i in range(max_index):
        r = {"index": i}
        cap = cv2.VideoCapture(i, backend)
        if not cap.isOpened():
            r["opened"] = False
            res.append(r)
            continue
        frames, t0 = [], time.time()
        while len(frames) < 15 and time.time() - t0 < 5:
            ok, f = cap.read()
            if ok and f is not None:
                frames.append(cv2.resize(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY), (160, 90)).astype(np.float32))
        cap.release()
        r["opened"] = True
        r["frames_read"] = len(frames)
        if frames:
            diffs = [float(np.abs(a - b).mean()) for a, b in zip(frames, frames[1:])]
            r["mean_brightness"] = round(float(np.mean(frames[-1])), 1)
            r["frame_change"] = round(float(np.mean(diffs)) if diffs else 0.0, 3)
            r["fps_estimate"] = round(len(frames) / max(time.time() - t0, 1e-6), 1)
            if r["mean_brightness"] < 8:
                r["verdict"] = "black picture - camera covered, privacy shutter closed, or virtual camera"
            elif r["frame_change"] < 0.05:
                r["verdict"] = "picture does not change - probably a VIRTUAL camera (OBS/phone app) or a still image"
            else:
                r["verdict"] = "live picture - likely a real camera (move your hand in front of it to be sure)"
        else:
            r["verdict"] = "opened but gave no frames"
        res.append(r)
    return res


def model_speed() -> dict:
    import numpy as np
    from src.config import load_config, resolve_path
    from src.detection.yolo_onnx import ObjectDetector, PoseDetector
    cfg = load_config()
    out = {}
    img = (np.random.default_rng(0).random((720, 1280, 3)) * 255).astype(np.uint8)
    for name, cls, path in (("pose", PoseDetector, cfg.models.pose_model), ("objects", ObjectDetector, cfg.models.object_model)):
        try:
            det = cls(resolve_path(path), imgsz=640, conf=0.4, iou=0.5, providers=list(cfg.models.providers))
            det.detect(img)
            t = []
            for _ in range(10):
                t0 = time.perf_counter()
                det.detect(img)
                t.append((time.perf_counter() - t0) * 1000)
            t.sort()
            out[name] = {"median_ms": round(t[len(t) // 2], 1), "max_ms": round(t[-1], 1),
                         "providers": getattr(det, "providers", None)}
        except Exception as exc:
            out[name] = {"error": f"{type(exc).__name__}: {exc}"}
    if "median_ms" in out.get("pose", {}):
        out["expected_fps_pose_only"] = round(1000 / out["pose"]["median_ms"], 1)
    return out


def settings() -> dict:
    import yaml
    from src.config import DEFAULT_CONFIG_PATH, load_config, validate
    raw = yaml.safe_load(DEFAULT_CONFIG_PATH.read_text(encoding="utf-8"))
    cfg = load_config()
    return {"problems": validate(raw), "camera_source": str(cfg.camera.source),
            "robot_enabled": cfg.robot.enabled, "robot_host": cfg.robot.host, "robot_port": cfg.robot.port,
            "dashboard": f"{cfg.dashboard.host}:{cfg.dashboard.port}"}


def robot_reachable(host: str, port: int) -> dict:
    t0 = time.perf_counter()
    try:
        s = socket.create_connection((host, port), timeout=2)
        s.settimeout(1.5)
        hello = s.recv(512).decode("ascii", "replace").strip()
        s.close()
        return {"reachable": True, "connect_ms": round((time.perf_counter() - t0) * 1000, 1), "hello": hello[:300]}
    except OSError as exc:
        return {"reachable": False, "error": str(exc)}


def main() -> int:
    rep = {"created": datetime.now().isoformat(timespec="seconds"), "python": sys.version.split()[0],
           "platform": platform.platform(), "machine": platform.machine(), "processor": platform.processor()}
    try:
        import psutil
        rep["cpu_cores"] = psutil.cpu_count(logical=False)
        rep["cpu_threads"] = psutil.cpu_count()
        rep["ram_gb"] = round(psutil.virtual_memory().total / 1e9, 1)
    except Exception:
        pass
    print("Checking packages ...")
    rep["packages"] = packages()
    print("Checking settings ...")
    try:
        rep["settings"] = settings()
    except Exception as exc:
        rep["settings"] = {"error": f"{type(exc).__name__}: {exc}"}
    print("Checking cameras 0-3 (a camera light may switch on briefly) ...")
    try:
        rep["cameras"] = cameras()
    except Exception as exc:
        rep["cameras"] = {"error": f"{type(exc).__name__}: {exc}"}
    print("Measuring model speed ...")
    try:
        rep["model_speed"] = model_speed()
    except Exception as exc:
        rep["model_speed"] = {"error": f"{type(exc).__name__}: {exc}"}
    st = rep.get("settings", {})
    if st.get("robot_host"):
        print(f"Trying the robot at {st['robot_host']}:{st['robot_port']} ...")
        rep["robot"] = robot_reachable(st["robot_host"], int(st["robot_port"]))
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(rep, indent=2), encoding="utf-8")
    print("\n===== SUMMARY =====")
    print("Python", rep["python"], "|", rep["platform"])
    missing = [k for k, v in rep["packages"].items() if str(v).startswith("MISSING")]
    print("Packages:", "all present" if not missing else "MISSING " + ", ".join(missing))
    for c in rep["cameras"] if isinstance(rep["cameras"], list) else []:
        print(f"Camera {c['index']}: " + (c.get("verdict", "") if c.get("opened") else "not available"))
    ms = rep.get("model_speed", {})
    if "pose" in ms:
        print("Pose model:", ms["pose"])
    if "robot" in rep:
        print("Robot:", "reachable" if rep["robot"]["reachable"] else "not reachable (" + rep["robot"]["error"] + ")")
    if st.get("problems"):
        print("Settings problems:", st["problems"])
    print(f"\nSaved {OUT}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
