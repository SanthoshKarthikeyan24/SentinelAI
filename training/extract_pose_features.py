"""Run the pretrained pose model + tracker over every indexed video and save the results.

This is the slow step (neural network inference), so it is done once and
cached in data/processed/pose_features/<video_id>.npz. Training and evaluation
then work on these files and take seconds.

It uses exactly the same PoseDetector and PersonTracker classes as the live
application, so training data matches runtime behaviour.

Each .npz contains, for every detection of every sampled frame:
    t         (N,)      timestamp in seconds
    frame     (N,)      original frame index
    track_id  (N,)      tracker id
    box       (N, 4)    x1 y1 x2 y2 pixels
    score     (N,)      detector score
    kpts      (N,17,3)  keypoints (x, y, score)
plus frame_times (all sampled frame times, including frames without people),
width, height, fps, sample_fps.

Usage:
    python training/extract_pose_features.py                 # all videos, 15 fps
    python training/extract_pose_features.py --fps 10 --overwrite
    python training/extract_pose_features.py --limit 3       # quick check
"""
from __future__ import annotations

import argparse
import csv
import time

import cv2
import numpy as np

from common import POSE_FEATURES, ROOT, VIDEO_INDEX, get_logger, now_iso, software_versions, write_json
from src.config import load_config, resolve_path
from src.detection.yolo_onnx import PoseDetector
from src.tracking.tracker import PersonTracker

log = get_logger("extract")


def extract_video(path, detector: PoseDetector, tracker_kwargs: dict, sample_fps: float) -> dict:
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise IOError(f"cannot open {path}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    step = max(1, int(round(fps / sample_fps)))
    tracker = PersonTracker(**tracker_kwargs)
    rows_t, rows_f, rows_id, rows_box, rows_s, rows_k, frame_times = [], [], [], [], [], [], []
    w = h = 0
    idx = 0
    while True:
        ok = cap.grab()
        if not ok:
            break
        if idx % step == 0:
            ok, frame = cap.retrieve()
            if not ok or frame is None:
                break
            h, w = frame.shape[:2]
            t = idx / fps
            frame_times.append(t)
            dets = detector.detect(frame)
            tracker.update(dets, t, (w, h))
            for d in dets:
                rows_t.append(t)
                rows_f.append(idx)
                rows_id.append(d.track_id)
                rows_box.append(d.box)
                rows_s.append(d.score)
                rows_k.append(d.keypoints)
        idx += 1
    cap.release()
    return {
        "t": np.array(rows_t, dtype=np.float64), "frame": np.array(rows_f, dtype=np.int32),
        "track_id": np.array(rows_id, dtype=np.int32),
        "box": np.array(rows_box, dtype=np.float32).reshape(-1, 4),
        "score": np.array(rows_s, dtype=np.float32),
        "kpts": np.array(rows_k, dtype=np.float32).reshape(-1, 17, 3),
        "frame_times": np.array(frame_times, dtype=np.float64),
        "width": np.int32(w), "height": np.int32(h), "fps": np.float64(fps),
        "sample_fps": np.float64(fps / step),
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None)
    ap.add_argument("--fps", type=float, default=15.0, help="frames per second to analyse")
    ap.add_argument("--overwrite", action="store_true")
    ap.add_argument("--limit", type=int, default=0, help="only process the first N videos")
    ap.add_argument("--dataset", default=None, help="only this dataset (e.g. gmdcsa24)")
    args = ap.parse_args()

    if not VIDEO_INDEX.exists():
        raise SystemExit("data/annotations/video_index.csv missing - run training/prepare_dataset.py first")
    cfg = load_config(args.config)
    det = PoseDetector(resolve_path(cfg.models.pose_model), cfg.detection.pose_imgsz,
                       cfg.detection.person_conf, cfg.detection.nms_iou,
                       list(cfg.models.providers), cfg.models.num_threads)
    trk = dict(iou_match_threshold=cfg.tracking.iou_match_threshold,
               max_center_distance=cfg.tracking.max_center_distance,
               min_hits=cfg.tracking.min_hits, max_missed_s=cfg.tracking.max_missed_s,
               history_s=cfg.tracking.history_s)
    with open(VIDEO_INDEX, newline="", encoding="utf-8") as fh:
        videos = list(csv.DictReader(fh))
    if args.dataset:
        videos = [v for v in videos if v["dataset"] == args.dataset]
    if args.limit:
        videos = videos[: args.limit]
    POSE_FEATURES.mkdir(parents=True, exist_ok=True)
    failures, done, skipped = [], 0, 0
    t_start = time.time()
    for k, v in enumerate(videos, 1):
        out = POSE_FEATURES / f"{v['video_id']}.npz"
        if out.exists() and not args.overwrite:
            skipped += 1
            continue
        t0 = time.time()
        try:
            res = extract_video(ROOT / v["path"], det, trk, args.fps)
        except Exception as exc:
            failures.append({"video_id": v["video_id"], "error": f"{type(exc).__name__}: {exc}"})
            log.error("%s failed: %s", v["video_id"], exc)
            continue
        np.savez_compressed(out, **res)
        done += 1
        n_frames = len(res["frame_times"])
        with_person = len(np.unique(res["t"])) if len(res["t"]) else 0
        log.info("[%d/%d] %s: %d frames, person in %d (%.0f%%), %d track ids, %.1fs",
                 k, len(videos), v["video_id"], n_frames, with_person,
                 100.0 * with_person / max(n_frames, 1), len(np.unique(res["track_id"])), time.time() - t0)
    write_json(POSE_FEATURES / "extraction_info.json", {
        "created": now_iso(), "pose_model": cfg.models.pose_model, "sample_fps": args.fps,
        "person_conf": cfg.detection.person_conf, "processed": done, "skipped_existing": skipped,
        "failures": failures, "seconds": round(time.time() - t_start, 1), "software": software_versions(),
    })
    log.info("Done: %d processed, %d skipped (already extracted), %d failed", done, skipped, len(failures))
    if failures:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
