"""Convert dataset annotations into the project's standard format.

Standard format (documented in data/README.md):
  data/annotations/video_index.csv        one row per video
  data/annotations/video_events/events.csv one row per labelled time interval

Supported sources
  * gmdcsa24  : GMDCSA-24 (github.com/ekramalam/GMDCSA24-A-Dataset-for-Human-Fall-Detection-in-Videos)
                expected in data/raw/public_datasets/GMDCSA24/Subject N/{ADL,Fall}.csv
  * collected : your own recordings made with training/collect_data.py and labelled with
                training/annotate_videos.py (data/annotations/video_events/collected_events.csv)

Problems (unparseable rows, unknown labels, missing videos, intervals
outside the video) are never silently dropped: they are written to
data/annotations/prepare_report.json and printed.

Usage:
    python training/prepare_dataset.py            # all available sources
    python training/prepare_dataset.py --sources gmdcsa24
"""
from __future__ import annotations

import argparse
import csv
import re
from pathlib import Path

import cv2

from common import (ANNOTATIONS, EVENT_COLUMNS, EVENTS_CSV, INDEX_COLUMNS, RAW_COLLECTED,
                    RAW_PUBLIC, ROOT, VIDEO_EVENTS, VIDEO_INDEX, get_logger, now_iso, write_json)

log = get_logger("prepare")

# Raw GMDCSA-24 label -> project label
GMDCSA_LABELS = {
    "falling": "FALL", "fall": "FALL",
    "standing": "STANDING", "walking": "WALKING", "sitting": "SITTING", "sleeping": "LYING",
    # activities that overlap a posture interval (not postures themselves)
    "exercising": "ACT_EXERCISING", "drinking": "ACT_DRINKING", "eating": "ACT_EATING",
    "reading": "ACT_READING", "writing": "ACT_WRITING",
}
_LABEL_RE = re.compile(r"([A-Za-z][A-Za-z ]*?)\s*(?:\(\s*([A-Za-z]+)\s*\)?)?\s*\[([^\[\]]*)")
_MISSING_CLOSE_RE = re.compile(r"(\d)\s*;\s*([A-Za-z][A-Za-z ]*\s*(?:\([A-Za-z]+\)?)?\s*\[)")
_INTERVAL_RE = re.compile(r"(\d+(?:\.\d+)?)\s*(?:(to)\s*|\s+)(\d+(?:\.\d+)?)")


def video_meta(path: Path) -> dict | None:
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        return None
    fps = cap.get(cv2.CAP_PROP_FPS) or 0.0
    frames = cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0.0
    w, h = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    ok, _ = cap.read()
    cap.release()
    if not ok or fps <= 0:
        return None
    return {"fps": round(fps, 3), "duration_s": round(frames / fps, 3), "width": w, "height": h}


def parse_gmdcsa_classes(text: str) -> list[tuple[str, str, float, float, str]]:
    """Parse e.g. 'Falling (SW)[3.4 to 6]; Sitting[0 to 3.4]' -> [(label, raw, start, end), ...]."""
    out = []
    # repair a missing ']' e.g. "Standing[2.4 to 3.6; 6.6 to 7.9; Walking[3.6 to 5.2]"
    repaired = _MISSING_CLOSE_RE.sub(r"\1]; \2", text)
    repair_note = "missing ']' repaired" if repaired != text else ""
    text = repaired
    for name, sub, body in _LABEL_RE.findall(text):
        key = name.strip().lower()
        label = GMDCSA_LABELS.get(key)
        raw = name.strip() + (f" ({sub})" if sub else "")
        for a, word, b in _INTERVAL_RE.findall(body):
            note = "" if word else f"interval '{a} {b}' written without 'to'"
            note = "; ".join(x for x in (note, repair_note) if x)
            out.append((label or f"UNRECOGNISED:{name.strip()}", raw, float(a), float(b), note))
    return out


def prepare_gmdcsa24(report: dict) -> tuple[list[dict], list[dict]]:
    root = RAW_PUBLIC / "GMDCSA24"
    index, events = [], []
    if not root.exists():
        report["gmdcsa24"] = {"status": "missing", "expected": str(root.relative_to(ROOT))}
        log.warning("GMDCSA-24 not found at %s (run training/download_datasets.py)", root)
        return index, events
    issues = []
    subject_dirs = sorted(p for p in root.iterdir() if p.is_dir() and p.name.lower().startswith("subject"))
    for sdir in subject_dirs:
        subject = "S" + sdir.name.split()[-1]
        for category in ("ADL", "Fall"):
            csv_path = sdir / f"{category}.csv"
            if not csv_path.exists():
                issues.append({"file": str(csv_path), "problem": "annotation csv missing"})
                continue
            lines = csv_path.read_text(encoding="utf-8", errors="replace").splitlines()
            for line_no, line in enumerate(lines[1:], start=2):
                if not line.strip():
                    continue
                fname = line.split(",")[0].strip()
                if not fname.lower().endswith(".mp4"):
                    issues.append({"file": str(csv_path.name), "line": line_no, "subject": subject,
                                   "problem": "row without a video file name", "text": line[:120]})
                    continue
                vpath = sdir / category / fname
                vid = f"gmdcsa24_{subject}_{category}_{Path(fname).stem}"
                meta = video_meta(vpath) if vpath.exists() else None
                if meta is None:
                    issues.append({"video_id": vid, "problem": "video missing or unreadable"})
                    continue
                parts = line.split(",")
                description = ",".join(parts[4:-1]).strip() if len(parts) > 5 else ""
                # labels live in the last field; search the text after the file name only
                parsed = parse_gmdcsa_classes(",".join(parts[1:]))
                if not parsed:
                    issues.append({"video_id": vid, "problem": "no labelled intervals parsed",
                                   "text": line[:160]})
                if category == "Fall" and not any(p_[0] == "FALL" for p_ in parsed):
                    # Using this video would turn a real fall into "no fall" training data.
                    issues.append({"video_id": vid, "action": "EXCLUDED",
                                   "problem": "fall video without a usable fall interval",
                                   "text": parts[-1].strip()[:120]})
                    continue
                index.append({"video_id": vid, "dataset": "gmdcsa24",
                              "path": str(vpath.relative_to(ROOT)).replace("\\", "/"),
                              "subject": subject, "session": f"{subject}_{category}",
                              "category": category.upper(), "description": description, **meta})
                for label, raw, s, e, note in parsed:
                    if note:
                        issues.append({"video_id": vid, "problem": note, "action": "parsed as interval"})
                    if label.startswith("UNRECOGNISED"):
                        issues.append({"video_id": vid, "problem": f"unknown label '{raw}'"})
                        continue
                    if e <= s:
                        issues.append({"video_id": vid, "problem": f"empty interval {raw} [{s}, {e}]"})
                        continue
                    if e > meta["duration_s"] + 0.5:
                        note = (note + "; " if note else "") + f"end {e}s clipped to video length {meta['duration_s']}s"
                        issues.append({"video_id": vid, "problem": note})
                        e = meta["duration_s"]
                    events.append({"video_id": vid, "label": label, "start_s": s, "end_s": e,
                                   "raw_label": raw, "person_id": subject,
                                   "uncertainty": "interval boundaries as provided by dataset authors",
                                   "notes": note})
    n_fall_videos = len({e["video_id"] for e in events if e["label"] == "FALL"})
    report["gmdcsa24"] = {"status": "ok", "videos": len(index), "events": len(events),
                          "fall_videos": n_fall_videos, "issues": issues}
    log.info("GMDCSA-24: %d videos, %d labelled intervals, %d fall videos, %d issues",
             len(index), len(events), n_fall_videos, len(issues))
    return index, events


def prepare_collected(report: dict) -> tuple[list[dict], list[dict]]:
    ann = VIDEO_EVENTS / "collected_events.csv"
    if not ann.exists():
        report["collected"] = {"status": "none yet", "expected": str(ann.relative_to(ROOT))}
        return [], []
    index, events, issues = [], [], []
    with open(ann, newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    seen = {}
    for r in rows:
        vpath = RAW_COLLECTED / r["video_file"]
        vid = "collected_" + Path(r["video_file"]).stem
        if vid not in seen:
            meta = video_meta(vpath) if vpath.exists() else None
            if meta is None:
                issues.append({"video_id": vid, "problem": "video missing or unreadable"})
                seen[vid] = None
                continue
            seen[vid] = meta
            index.append({"video_id": vid, "dataset": "collected",
                          "path": str(vpath.relative_to(ROOT)).replace("\\", "/"),
                          "subject": r.get("person_id") or "unknown",
                          "session": r.get("session") or Path(r["video_file"]).stem,
                          "category": "FALL" if any(x["video_file"] == r["video_file"] and x["label"].strip().upper() in ("FALL", "FALL_LIKE_EVENT")
                                                    for x in rows) else "ADL",
                          "description": r.get("notes", ""), **meta})
        if seen[vid] is None:
            continue
        try:
            s, e = float(r["start_s"]), float(r["end_s"])
        except ValueError:
            issues.append({"video_id": vid, "problem": f"bad times {r['start_s']}..{r['end_s']}"})
            continue
        if e <= s:
            issues.append({"video_id": vid, "problem": f"empty interval {r['label']}"})
            continue
        events.append({"video_id": vid, "label": r["label"].strip().upper(), "start_s": s, "end_s": e,
                       "raw_label": r["label"], "person_id": r.get("person_id", ""),
                       "uncertainty": r.get("uncertainty", ""), "notes": r.get("notes", "")})
    report["collected"] = {"status": "ok", "videos": len(index), "events": len(events), "issues": issues}
    return index, events


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sources", nargs="+", default=["gmdcsa24", "collected"],
                    choices=["gmdcsa24", "collected"])
    args = ap.parse_args()
    report: dict = {"created": now_iso()}
    index, events = [], []
    for src in args.sources:
        i, e = {"gmdcsa24": prepare_gmdcsa24, "collected": prepare_collected}[src](report)
        index += i
        events += e
    if not index:
        raise SystemExit("No videos found. Download a dataset first (see docs/DATASET_RESEARCH.md).")
    ANNOTATIONS.mkdir(parents=True, exist_ok=True)
    VIDEO_EVENTS.mkdir(parents=True, exist_ok=True)
    with open(VIDEO_INDEX, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=INDEX_COLUMNS)
        w.writeheader()
        w.writerows(index)
    with open(EVENTS_CSV, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=EVENT_COLUMNS)
        w.writeheader()
        w.writerows(events)
    write_json(ANNOTATIONS / "prepare_report.json", report)
    for src, rep in report.items():
        if isinstance(rep, dict) and rep.get("issues"):
            log.warning("%s: %d issue(s) - see data/annotations/prepare_report.json", src, len(rep["issues"]))
            for it in rep["issues"][:10]:
                log.warning("   %s", it)
    log.info("Wrote %s (%d videos) and %s (%d intervals)", VIDEO_INDEX.relative_to(ROOT), len(index),
             EVENTS_CSV.relative_to(ROOT), len(events))


if __name__ == "__main__":
    main()
