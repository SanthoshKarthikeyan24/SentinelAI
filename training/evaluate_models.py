"""Collect all measured results into evaluation/SUMMARY.md.

Measures (on this computer):
  * pose-model coverage on the dataset: share of frames with a detected person,
    keypoint visibility of the labelled person, spurious extra tracks
  * inference speed of the pose and object models and of the trained classifiers
Collects (from earlier runs):
  * evaluation/posture/metrics.json  (train_posture_classifier.py)
  * evaluation/fall/metrics.json     (train_fall_classifier.py)

Usage:
    python training/evaluate_models.py
    python training/evaluate_models.py --speed-frames 50
"""
from __future__ import annotations

import argparse
import json
import platform
import time

import cv2
import numpy as np

from common import EVALUATION, POSE_FEATURES, ROOT, get_logger, now_iso, software_versions, write_json
from dataset_lib import load_index
from src.config import load_config, resolve_path

log = get_logger("evaluate")


def pose_coverage(kpt_thr: float) -> dict:
    index = load_index()
    frames = with_person = 0
    vis, extra_tracks, per_video = [], [], []
    for v in index:
        f = POSE_FEATURES / f"{v['video_id']}.npz"
        if not f.exists():
            continue
        z = np.load(f)
        ft, t, tid, k = z["frame_times"], z["t"], z["track_id"], z["kpts"]
        n_f = len(ft)
        n_p = len(np.unique(t))
        frames += n_f
        with_person += n_p
        per_video.append(n_p / max(n_f, 1))
        if len(tid):
            ids, counts = np.unique(tid, return_counts=True)
            primary = ids[np.argmax(counts)]
            vis.extend(((k[tid == primary][:, :, 2] >= kpt_thr).sum(1) / 17.0).tolist())
            extra_tracks.append(len(ids) - 1)
    return {
        "videos": len(per_video), "sampled_frames": int(frames),
        "frames_with_person": int(with_person),
        "frame_coverage": round(with_person / max(frames, 1), 4),
        "videos_with_coverage_below_80pct": int(sum(c < 0.8 for c in per_video)),
        "primary_person_mean_visible_keypoints": round(float(np.mean(vis)) * 17, 2) if vis else None,
        "primary_person_frames_with_at_least_7_keypoints": round(float(np.mean(np.array(vis) * 17 >= 7)), 4) if vis else None,
        "mean_extra_track_ids_per_video": round(float(np.mean(extra_tracks)), 2) if extra_tracks else None,
        "note": "No keypoint ground truth exists for GMDCSA-24, so this is detection coverage, not keypoint accuracy.",
    }


def speed(cfg, n: int) -> dict:
    from src.detection.yolo_onnx import ObjectDetector, PoseDetector
    vids = sorted((ROOT / "data/raw/public_datasets/GMDCSA24").glob("Subject 2/ADL/01.mp4"))
    if vids:
        cap = cv2.VideoCapture(str(vids[0]))
        frames = []
        while len(frames) < n:
            ok, f = cap.read()
            if not ok:
                break
            frames.append(f)
        cap.release()
    else:
        frames = [np.random.randint(0, 255, (720, 1280, 3), np.uint8) for _ in range(n)]
    out = {"machine": platform.processor() or platform.machine(), "frames": len(frames)}
    for name, det in (("pose", PoseDetector(resolve_path(cfg.models.pose_model))),
                      ("object", ObjectDetector(resolve_path(cfg.models.object_model),
                                                classes=list(cfg.detection.object_classes)))):
        det.detect(frames[0])
        ts = []
        for f in frames:
            t0 = time.perf_counter()
            det.detect(f)
            ts.append((time.perf_counter() - t0) * 1000)
        out[f"{name}_ms_mean"] = round(float(np.mean(ts)), 1)
        out[f"{name}_ms_p95"] = round(float(np.percentile(ts, 95)), 1)
        out["providers"] = det.providers
    import joblib
    for name, rel, nfeat in (("posture_model", cfg.models.posture_model, None),
                             ("fall_model", cfg.models.fall_model, None)):
        p = resolve_path(rel)
        if not p.exists():
            continue
        b = joblib.load(p)
        x = np.random.rand(1, len(b["feature_names"]))
        b["model"].predict_proba(x)
        t0 = time.perf_counter()
        for _ in range(50):
            b["model"].predict_proba(x)
        out[f"{name}_ms_per_call"] = round((time.perf_counter() - t0) / 50 * 1000, 2)
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None)
    ap.add_argument("--speed-frames", type=int, default=40)
    args = ap.parse_args()
    cfg = load_config(args.config)
    cov = pose_coverage(cfg.detection.keypoint_conf)
    write_json(EVALUATION / "pose" / "pose_coverage.json", cov)
    log.info("Pose coverage: %s", cov)
    sp = speed(cfg, args.speed_frames)
    write_json(EVALUATION / "speed.json", sp)
    log.info("Speed: %s", sp)

    L = ["# SentinelAI - measured results", "", f"Generated {now_iso()} by `training/evaluate_models.py`.",
         "All numbers below were produced by scripts in this repository; nothing is estimated by hand.", "",
         "## Pose model coverage on GMDCSA-24", "",
         f"- Sampled frames: {cov['sampled_frames']} from {cov['videos']} videos",
         f"- Frames with at least one detected person: {cov['frame_coverage']:.1%}",
         f"- Labelled person: mean visible keypoints {cov['primary_person_mean_visible_keypoints']} / 17; "
         f"frames with >= 7 keypoints: {cov['primary_person_frames_with_at_least_7_keypoints']:.1%}",
         f"- Videos where a person was found in < 80% of frames: {cov['videos_with_coverage_below_80pct']}",
         f"- {cov['note']}", ""]
    for name, path in (("Posture classifier", EVALUATION / "posture" / "report.md"),
                       ("Fall detection", EVALUATION / "fall" / "report.md")):
        if path.exists():
            body = path.read_text(encoding="utf-8").split("\n", 1)[1]
            L += [f"## {name}", body, ""]
        else:
            L += [f"## {name}", "", "Not trained yet.", ""]
    L += ["## Speed on the build machine", "",
          f"- CPU: {sp['machine']}; ONNX Runtime providers: {sp.get('providers')}",
          f"- Pose model: {sp['pose_ms_mean']} ms/frame (p95 {sp['pose_ms_p95']})",
          f"- Object model: {sp['object_ms_mean']} ms/frame (p95 {sp['object_ms_p95']})"]
    for k in ("posture_model_ms_per_call", "fall_model_ms_per_call"):
        if k in sp:
            L.append(f"- {k.replace('_', ' ')}: {sp[k]} ms")
    L += ["", "Your laptop's speed will differ; the dashboard shows live FPS and inference time.", "",
          "## Software", "", "```", json.dumps(software_versions(), indent=2), "```"]
    (EVALUATION / "SUMMARY.md").write_text("\n".join(L) + "\n", encoding="utf-8")
    log.info("Wrote evaluation/SUMMARY.md")


if __name__ == "__main__":
    main()
