"""Build data/dataset_inventory.csv: one row per video with everything needed to judge the data.

Columns: video_id, dataset, subject, category, file, duration_s, fps, resolution, lighting,
attire, labels, fall_intervals, labelled_s, pose_file, pose_frames, person_found_pct,
mean_visible_keypoints, used_for, loso_test_fold, fixed_split, license, status, notes

Run:  python training/build_inventory.py
(It only reads the index, annotations, pose files and the dataset's own CSV metadata.)
"""
from __future__ import annotations

import csv
from collections import Counter

import numpy as np

from common import ANNOTATIONS, DATA, POSE_FEATURES, RAW_PUBLIC, ROOT, get_logger, write_json
from dataset_lib import FALL_LABELS, load_events, load_index, load_splits

log = get_logger("inventory")
OUT = DATA / "dataset_inventory.csv"
COLUMNS = ["video_id", "dataset", "subject", "category", "file", "duration_s", "fps", "resolution", "lighting",
           "lighting_group", "attire", "labels", "fall_intervals", "labelled_s", "pose_file", "pose_frames", "person_found_pct",
           "mean_visible_keypoints", "used_for", "loso_test_fold", "fixed_split", "license", "status", "notes"]
LICENSES = {"gmdcsa24": "MIT (GMDCSA-24 repository LICENSE, Copyright (c) 2024 Ekram Alam)",
            "collected": "own recordings - consent required, do not publish"}


def gmdcsa_metadata() -> dict[str, dict]:
    """(subject, category, file) -> Time of Recording / Attire from the dataset's own CSV files."""
    out = {}
    for s in range(1, 5):
        for cat in ("ADL", "Fall"):
            p = RAW_PUBLIC / "GMDCSA24" / f"Subject {s}" / f"{cat}.csv"
            if not p.exists():
                continue
            with open(p, newline="", encoding="utf-8-sig") as fh:
                for r in csv.DictReader(fh):
                    r = {k.strip(): (v or "").strip() for k, v in r.items() if k}
                    out[(f"S{s}", cat, r.get("File Name", ""))] = r
    return out


def lighting_group(text: str) -> str:
    """Day (Light On)/Day/Day (Lights On) -> day_lit; Day (Lights Off) -> day_dim; Night -> night."""
    t = text.lower()
    if not t:
        return "unknown"
    if "night" in t:
        return "night"
    if "off" in t:
        return "day_dim"
    return "day_lit"


def main() -> None:
    index = load_index()
    events = load_events()
    splits = load_splits()
    meta = gmdcsa_metadata()
    fold_of = {v: f["test_subject"] for f in splits["loso_folds"] for v in f["test"]}
    fixed = {v: name for name in ("train", "validation", "test") for v in splits["fixed"].get(name, [])}
    report = {}
    try:
        import json
        report = json.loads((ANNOTATIONS / "prepare_report.json").read_text(encoding="utf-8"))
    except (OSError, ValueError):
        pass
    issues: dict[str, list[str]] = {}
    for ds in ("gmdcsa24", "collected"):
        for it in (report.get(ds) or {}).get("issues", []) if isinstance(report.get(ds), dict) else []:
            issues.setdefault(it["video_id"], []).append(it.get("problem", "") + (
                f" ({it['action']})" if it.get("action") else ""))
    rows = []
    for v in index:
        vid = v["video_id"]
        evs = events.get(vid, [])
        labels = Counter(e["label"] for e in evs)
        n_fall = sum(1 for e in evs if e["label"] in FALL_LABELS)
        labelled = sum(max(0.0, e["end_s"] - e["start_s"]) for e in evs if not e["label"].startswith("ACT_"))
        fname = v["path"].replace("\\", "/").split("/")[-1]
        cat = "Fall" if v["category"].upper() == "FALL" else "ADL"
        m = meta.get((v["subject"], cat, fname), {})
        pose_path = POSE_FEATURES / f"{vid}.npz"
        pose_frames = found = kp = ""
        if pose_path.exists():
            with np.load(pose_path) as z:
                ft, t, kpts = z["frame_times"], z["t"], z["kpts"]
                pose_frames = len(ft)
                found = round(100.0 * len(np.unique(t)) / max(1, len(ft)), 1)
                kp = round(float((kpts[..., 2] >= 0.35).sum(-1).mean()), 2) if len(kpts) else 0.0
        used = []
        if any(e["label"] in ("STANDING", "WALKING", "SITTING", "LYING") for e in evs):
            used.append("posture")
        used.append("fall" if v["category"] == "Fall" else "fall(negatives)")
        rows.append({
            "video_id": vid, "dataset": v["dataset"], "subject": v["subject"], "category": v["category"],
            "file": v["path"], "duration_s": round(v["duration_s"], 2), "fps": round(float(v["fps"]), 2),
            "resolution": f"{v['width']}x{v['height']}", "lighting": m.get("Time of Recording", "unknown"),
            "lighting_group": lighting_group(m.get("Time of Recording", "")),
            "attire": m.get("Attire", ""), "labels": ";".join(f"{k}:{n}" for k, n in sorted(labels.items())),
            "fall_intervals": n_fall, "labelled_s": round(labelled, 2), "pose_file": pose_path.exists(),
            "pose_frames": pose_frames, "person_found_pct": found, "mean_visible_keypoints": kp,
            "used_for": "+".join(used), "loso_test_fold": fold_of.get(vid, ""), "fixed_split": fixed.get(vid, ""),
            "license": LICENSES.get(v["dataset"], "check"), "status": "used",
            "notes": " | ".join(issues.get(vid, [])),
        })
    # excluded videos are listed too, so nothing silently disappears
    for vid, probs in issues.items():
        if any("EXCLUDED" in p for p in probs) and not any(r["video_id"] == vid for r in rows):
            ds = vid.split("_")[0]
            rows.append({c: "" for c in COLUMNS} | {"video_id": vid, "dataset": ds, "subject": vid.split("_")[1],
                                                    "category": vid.split("_")[2], "status": "excluded",
                                                    "license": LICENSES.get(ds, "check"), "notes": " | ".join(probs)})
    rows.sort(key=lambda r: r["video_id"])
    with open(OUT, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=COLUMNS)
        w.writeheader()
        w.writerows(rows)
    used = [r for r in rows if r["status"] == "used"]
    summary = {
        "videos_used": len(used), "videos_excluded": len(rows) - len(used),
        "hours": round(sum(r["duration_s"] for r in used) / 3600, 3),
        "by_subject": dict(Counter(r["subject"] for r in used)),
        "by_category": dict(Counter(r["category"] for r in used)),
        "by_lighting": dict(Counter(r["lighting"] for r in used)),
        "by_lighting_group": dict(Counter(r["lighting_group"] for r in used)),
        "fall_intervals": sum(r["fall_intervals"] for r in used),
        "resolutions": dict(Counter(r["resolution"] for r in used)),
    }
    write_json(DATA / "dataset_inventory_summary.json", summary)
    log.info("Wrote %s (%d rows): %s", OUT.relative_to(ROOT), len(rows), summary)


if __name__ == "__main__":
    main()
