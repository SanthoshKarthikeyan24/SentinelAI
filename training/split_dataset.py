"""Create reproducible subject-level data splits (prevents data leakage).

Frames from one recording are almost identical, and the same person looks
similar in every recording. If neighbouring frames or the same person appear
in both training and test data, the test score is unrealistically high.

So every split here is by SUBJECT (person):
  * loso_folds : leave-one-subject-out folds. Each fold trains on all other
                 subjects and tests on one unseen subject. Main protocol for
                 reported metrics (every video is tested exactly once by a model
                 that never saw that person).
  * fixed      : one fixed train / validation / test assignment for quick
                 experiments (test subject never used for any tuning).

Usage:
    python training/split_dataset.py
    python training/split_dataset.py --test-subject S4 --val-subject S3
"""
from __future__ import annotations

import argparse
from collections import Counter

from common import SPLITS, get_logger, now_iso, write_json
from dataset_lib import load_events, load_index

log = get_logger("split")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--test-subject", default=None, help="subject id for the fixed test split")
    ap.add_argument("--val-subject", default=None, help="subject id for the fixed validation split")
    args = ap.parse_args()

    index = load_index()
    events = load_events()
    subjects = sorted({v["subject"] for v in index})
    if len(subjects) < 3:
        raise SystemExit(f"Need at least 3 subjects for subject-level splits, found {subjects}")
    test_s = args.test_subject or subjects[-1]
    val_s = args.val_subject or subjects[-2]
    for s in (test_s, val_s):
        if s not in subjects:
            raise SystemExit(f"Unknown subject {s}; available: {subjects}")
    if test_s == val_s:
        raise SystemExit("validation and test subject must differ")

    by_subject = {s: [v["video_id"] for v in index if v["subject"] == s] for s in subjects}
    fixed = {
        "train": [v for s in subjects if s not in (test_s, val_s) for v in by_subject[s]],
        "validation": by_subject[val_s], "test": by_subject[test_s],
        "subjects": {"train": [s for s in subjects if s not in (test_s, val_s)],
                     "validation": [val_s], "test": [test_s]},
    }
    folds = [{"test_subject": s, "train_subjects": [o for o in subjects if o != s],
              "test": by_subject[s], "train": [v for o in subjects if o != s for v in by_subject[o]]}
             for s in subjects]
    stats = {}
    for s in subjects:
        c = Counter(e["label"] for v in by_subject[s] for e in events.get(v, []))
        stats[s] = {"videos": len(by_subject[s]), "fall_events": c.get("FALL", 0), "labels": dict(c)}
    # sanity: no video in two parts of the fixed split, no subject overlap
    parts = [set(fixed["train"]), set(fixed["validation"]), set(fixed["test"])]
    assert not (parts[0] & parts[1] or parts[0] & parts[2] or parts[1] & parts[2]), "split overlap!"
    write_json(SPLITS / "splits.json", {"created": now_iso(), "unit": "subject", "subjects": subjects,
                                         "loso_folds": folds, "fixed": fixed, "subject_stats": stats})
    for name in ("train", "validation", "test"):
        (SPLITS / name).mkdir(parents=True, exist_ok=True)
        (SPLITS / name / "videos.txt").write_text("\n".join(fixed[name]) + "\n", encoding="utf-8")
    log.info("Subjects: %s", subjects)
    for s, st in stats.items():
        log.info("  %s: %d videos, %d fall events", s, st["videos"], st["fall_events"])
    log.info("Fixed split: train %s | validation %s | test %s", fixed["subjects"]["train"], val_s, test_s)
    log.info("Wrote data/splits/splits.json (%d LOSO folds)", len(folds))


if __name__ == "__main__":
    main()
