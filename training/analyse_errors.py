"""List the false alarms and missed falls of the trained system (leave-one-subject-out).

Helps you understand WHERE the detector fails, e.g. "lying down on a bed
quickly" or "fall happened outside the camera view".

Usage:
    python training/analyse_errors.py                    # threshold from the saved model
    python training/analyse_errors.py --threshold 0.6
Writes evaluation/fall/error_analysis.csv
"""
from __future__ import annotations

import argparse
import csv
import json

from common import ARTIFACTS, EVALUATION, get_logger
from dataset_lib import build_all_caches, gt_falls, load_events, load_index, load_splits
from src.analysis.fall_detector import FallParams
from src.config import load_config
from train_fall_classifier import Ctx

log = get_logger("errors")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None)
    ap.add_argument("--threshold", type=float, default=None)
    ap.add_argument("--model", default=None, choices=["rf", "hgb"])
    args = ap.parse_args()
    cfg = load_config(args.config)
    card = json.loads((ARTIFACTS / "fall_classifier" / "model_card.json").read_text())
    thr = args.threshold if args.threshold is not None else card["decision_threshold"]
    model = args.model or {"Random Forest": "rf", "Hist Gradient Boosting": "hgb"}[card["model_type"]]
    index = {v["video_id"]: v for v in load_index()}
    events = load_events()
    p = FallParams.from_config(cfg.fall)
    caches = build_all_caches(list(index), p, cfg.detection.keypoint_conf, cfg.tracking.history_s, log)
    ctx = Ctx(cfg, caches, events, index, "rf")
    rows = []
    for fold in load_splits()["loso_folds"]:
        ffn, _ = ctx.fall_fn(model, fold["train_subjects"])
        pfn, pcls = ctx.posture_fn(fold["train_subjects"])
        alarms = ctx.replay(ctx.videos_of([fold["test_subject"]]), ffn, thr, pfn, pcls)
        for v, al in alarms.items():
            gt = gt_falls(events.get(v, []))
            for kind in ("FALL_SUSPECTED", "FALL_CONFIRMED"):
                times = [a["t"] for a in al if a["type"] == kind]
                for s, e in gt:
                    if not any(s - 0.5 <= t <= e + 1.0 for t in times):
                        rows.append({"video_id": v, "error": "MISSED", "level": kind, "time_s": f"{s:.1f}",
                                     "description": index[v]["description"]})
                for t in times:
                    if not any(s - 0.5 <= t <= e + 1.0 for s, e in gt):
                        rows.append({"video_id": v, "error": "FALSE_ALARM", "level": kind, "time_s": f"{t:.1f}",
                                     "description": index[v]["description"]})
    out = EVALUATION / "fall" / "error_analysis.csv"
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=["video_id", "error", "level", "time_s", "description"])
        w.writeheader()
        w.writerows(rows)
    for r in rows:
        if r["level"] == "FALL_SUSPECTED":
            log.info("%-12s %-26s t=%5ss  %s", r["error"], r["video_id"], r["time_s"], r["description"][:70])
    log.info("Wrote %s (%d rows)", out, len(rows))


if __name__ == "__main__":
    main()
