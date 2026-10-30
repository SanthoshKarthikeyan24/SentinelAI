"""Inspect and validate the dataset folders, annotations, pose files and splits.

Checks
  * expected folders exist
  * every indexed video exists and opens
  * every interval has a known label, start < end, and lies inside its video
  * overlapping posture intervals with different labels (ambiguous frames)
  * which videos still need pose extraction
  * subject-level splits do not leak (no subject/video in two parts)
Nothing is deleted; problems are listed in evaluation/dataset_inspection.json.

Usage:
    python training/inspect_dataset.py
    python training/inspect_dataset.py --open-videos    # also try to open every video (slower)
"""
from __future__ import annotations

import argparse
import json
from collections import Counter, defaultdict

import cv2

from common import (ANNOTATIONS, DATA, EVALUATION, EVENTS_CSV, POSE_FEATURES, ROOT, SPLITS, VIDEO_INDEX,
                    get_logger, now_iso, write_json)

log = get_logger("inspect")
KNOWN = {"FALL", "FALL_LIKE_EVENT", "STANDING", "WALKING", "SITTING", "LYING", "BENDING", "KNEELING",
         "RECOVERY", "OCCLUDED", "UNKNOWN", "ACT_EXERCISING", "ACT_DRINKING", "ACT_EATING",
         "ACT_READING", "ACT_WRITING"}
POSTURE = {"STANDING", "WALKING", "SITTING", "LYING", "BENDING", "KNEELING"}
EXPECTED_DIRS = ["raw/public_datasets", "annotations/video_events", "processed/pose_features", "splits"]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--open-videos", action="store_true")
    args = ap.parse_args()
    from dataset_lib import load_events, load_index
    report: dict = {"created": now_iso(), "problems": [], "warnings": []}
    for d in EXPECTED_DIRS:
        if not (DATA / d).exists():
            report["warnings"].append(f"missing folder data/{d}")
    if not VIDEO_INDEX.exists() or not EVENTS_CSV.exists():
        raise SystemExit("Run training/prepare_dataset.py first (no index/events found).")
    index = load_index()
    events = load_events()
    report["videos"] = len(index)
    report["by_dataset"] = dict(Counter(v["dataset"] for v in index))
    report["by_subject_category"] = dict(Counter(f"{v['subject']}/{v['category']}" for v in index))
    report["total_minutes"] = round(sum(v["duration_s"] for v in index) / 60, 2)
    label_counts, label_seconds = Counter(), defaultdict(float)
    need_pose = []
    for v in index:
        vid = v["video_id"]
        path = ROOT / v["path"]
        if not path.exists():
            report["problems"].append({"video_id": vid, "problem": "video file missing", "path": v["path"]})
        elif args.open_videos:
            cap = cv2.VideoCapture(str(path))
            ok = cap.isOpened() and cap.read()[0]
            cap.release()
            if not ok:
                report["problems"].append({"video_id": vid, "problem": "video cannot be decoded"})
        if not (POSE_FEATURES / f"{vid}.npz").exists():
            need_pose.append(vid)
        evs = events.get(vid, [])
        if not evs:
            report["warnings"].append({"video_id": vid, "warning": "no labelled intervals"})
        for e in evs:
            label_counts[e["label"]] += 1
            label_seconds[e["label"]] += e["end_s"] - e["start_s"]
            if e["label"] not in KNOWN:
                report["problems"].append({"video_id": vid, "problem": f"unknown label {e['label']}"})
            if e["end_s"] <= e["start_s"]:
                report["problems"].append({"video_id": vid, "problem": f"empty interval {e['label']}"})
            if e["start_s"] < 0 or e["end_s"] > v["duration_s"] + 0.5:
                report["problems"].append({"video_id": vid, "problem":
                                           f"{e['label']} [{e['start_s']}, {e['end_s']}] outside video "
                                           f"(0-{v['duration_s']}s)"})
        post = [e for e in evs if e["label"] in POSTURE]
        for i, a in enumerate(post):
            for b in post[i + 1:]:
                if a["label"] != b["label"] and min(a["end_s"], b["end_s"]) - max(a["start_s"], b["start_s"]) > 0.05:
                    report["warnings"].append({"video_id": vid, "warning":
                                               f"overlapping {a['label']} and {b['label']} (frames skipped in training)"})
    report["label_counts"] = dict(label_counts)
    report["label_seconds"] = {k: round(v, 1) for k, v in label_seconds.items()}
    report["videos_without_pose_features"] = need_pose
    split_file = SPLITS / "splits.json"
    if split_file.exists():
        sp = json.loads(split_file.read_text())
        f = sp["fixed"]
        sets = {k: set(f[k]) for k in ("train", "validation", "test")}
        subj = {k: {v["subject"] for v in index if v["video_id"] in sets[k]} for k in sets}
        for a in sets:
            for b in sets:
                if a < b and (sets[a] & sets[b] or subj[a] & subj[b]):
                    report["problems"].append({"problem": f"split leakage between {a} and {b}"})
        for fold in sp["loso_folds"]:
            if set(fold["test"]) & set(fold["train"]):
                report["problems"].append({"problem": f"LOSO fold {fold['test_subject']} leaks"})
        report["splits"] = {k: len(v) for k, v in sets.items()}
        indexed = {v["video_id"] for v in index}
        in_splits = set().union(*sets.values())
        if indexed - in_splits:
            report["warnings"].append(f"{len(indexed - in_splits)} videos not in any split - re-run split_dataset.py")
    else:
        report["warnings"].append("no splits yet - run training/split_dataset.py")
    prep = ANNOTATIONS / "prepare_report.json"
    if prep.exists():
        report["excluded_or_repaired_during_preparation"] = json.loads(prep.read_text())
    write_json(EVALUATION / "dataset_inspection.json", report)
    log.info("%d videos (%.1f min) %s", len(index), report["total_minutes"], report["by_dataset"])
    log.info("Labels (count): %s", dict(label_counts))
    log.info("Labels (seconds): %s", report["label_seconds"])
    log.info("Pose features missing for %d videos", len(need_pose))
    log.info("%d problems, %d warnings -> evaluation/dataset_inspection.json",
             len(report["problems"]), len(report["warnings"]))
    for p in report["problems"][:20]:
        log.error("  %s", p)


if __name__ == "__main__":
    main()
