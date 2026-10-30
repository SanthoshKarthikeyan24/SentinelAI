"""Compare design options for fall detection WITHOUT touching the test subject.

Each option is scored by leave-one-subject-out on the development subjects
only (all subjects except the fixed-split test subject, S4 by default), using
event-level F1 of FALL_SUSPECTED with the threshold chosen inside those folds.
The test subject stays unseen, so the final reported numbers are not tuned
on it.

Options compared
  * feature set: original 24 window features vs extended set
  * window length (window_s)
  * minimum time down before FALL_SUSPECTED (suspect_min_down_s)

Usage:
    python training/tune_fall_settings.py
Writes evaluation/fall/tuning.json and prints a table. Copy the best values
into config/settings.yaml, then re-run train_fall_classifier.py.
"""
from __future__ import annotations

import argparse
import itertools

from common import EVALUATION, get_logger, now_iso, write_json
from dataset_lib import build_all_caches, load_events, load_index, load_splits
from src.analysis.fall_detector import FallParams
from src.analysis.features import BASE_WINDOW_FEATURES, WINDOW_FEATURES
from src.config import load_config
from train_fall_classifier import Ctx, select

log = get_logger("tune")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None)
    ap.add_argument("--windows", nargs="+", type=float, default=[1.5, 2.0])
    ap.add_argument("--down", nargs="+", type=float, default=[0.4, 0.8])
    ap.add_argument("--models", nargs="+", default=["rf"], choices=["rf", "hgb"])
    args = ap.parse_args()
    cfg = load_config(args.config)
    splits = load_splits()
    test_subject = splits["fixed"]["subjects"]["test"][0]
    dev = [s for s in splits["subjects"] if s != test_subject]
    index = {v["video_id"]: v for v in load_index() if v["subject"] != test_subject}
    events = load_events()
    grid = [0.3, 0.4, 0.5, 0.6, 0.7]
    results = []
    for w in args.windows:
        base = FallParams.from_config(cfg.fall)
        base.window_s = w
        caches = build_all_caches(list(index), base, cfg.detection.keypoint_conf, cfg.tracking.history_s, log)
        for feats, down in itertools.product(("base", "extended"), args.down):
            p = FallParams(**{**base.__dict__, "suspect_min_down_s": down})
            names = BASE_WINDOW_FEATURES if feats == "base" else WINDOW_FEATURES
            ctx = Ctx(cfg, caches, events, index, "rf", params=p, feature_names=names)
            sel = select(ctx, dev, args.models, grid)
            best = sel["grid"][f"{sel['model']}@{sel['threshold']}"]
            row = {"window_s": w, "features": feats, "suspect_min_down_s": down, "model": sel["model"],
                   "threshold": sel["threshold"], "dev_f1": best["f1"], "dev_precision": best["precision"],
                   "dev_recall": best["recall"]}
            results.append(row)
            log.info("window %.1f  %-8s down %.1f -> %s@%.1f  F1 %.3f (P %.3f R %.3f)", w, feats, down,
                     sel["model"], sel["threshold"], best["f1"], best["precision"], best["recall"])
    best = max(results, key=lambda r: (r["dev_f1"], r["dev_precision"]))
    write_json(EVALUATION / "fall" / "tuning.json", {"created": now_iso(), "development_subjects": dev,
                                                     "held_out_subject": test_subject, "results": results,
                                                     "best": best})
    log.info("Best on development subjects: %s", best)


if __name__ == "__main__":
    main()
