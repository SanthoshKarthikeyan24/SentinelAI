"""Label time intervals in your own recordings (posture and fall-like events).

Labels are stored as INTERVALS, not one label per whole video:
    data/annotations/video_events/collected_events.csv
    columns: video_file, label, start_s, end_s, person_id, session, uncertainty, notes

Interactive player (OpenCV window):
    python training/annotate_videos.py data/raw/collected_videos/S20261101_P01_sitting_....mp4
      space  play/pause           a / d   back / forward 1 frame
      j / l  back / forward 1 s   1-9     choose label (shown on screen)
      s      mark start           e       mark end -> saves the interval
      u      undo last interval    q      quit

Command line (no window), e.g. for scripted labelling:
    python training/annotate_videos.py VIDEO --add SITTING 0.0 4.2 --person P01
    python training/annotate_videos.py VIDEO --list

Label definitions (keep these consistent!):
    STANDING   upright, feet on the floor, not walking
    WALKING    upright and moving
    SITTING    weight on buttocks (chair, bed, floor)
    BENDING    torso tilted forward > ~35 deg with legs mostly straight
    KNEELING   one or both knees on the floor
    LYING      torso horizontal on a bed / sofa / floor
    FALL_LIKE_EVENT  from the first loss of balance until the body comes to rest
    RECOVERY   getting up after lying / a fall-like event
    OCCLUDED   person mostly hidden or out of frame
    UNKNOWN    cannot tell
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import cv2

from common import RAW_COLLECTED, ROOT, VIDEO_EVENTS, get_logger

log = get_logger("annotate")
LABELS = ["STANDING", "WALKING", "SITTING", "BENDING", "KNEELING", "LYING", "FALL_LIKE_EVENT",
          "RECOVERY", "OCCLUDED", "UNKNOWN"]
COLUMNS = ["video_file", "label", "start_s", "end_s", "person_id", "session", "uncertainty", "notes"]
ANN_FILE = VIDEO_EVENTS / "collected_events.csv"


def video_rel(path: Path) -> str:
    """Path of the video relative to data/raw/collected_videos (where prepare_dataset looks)."""
    path = path.resolve()
    try:
        return str(path.relative_to(RAW_COLLECTED.resolve())).replace("\\", "/")
    except ValueError:
        raise SystemExit(f"Put the video inside {RAW_COLLECTED.relative_to(ROOT)} first: {path}")


def load_rows() -> list[dict]:
    if not ANN_FILE.exists():
        return []
    with open(ANN_FILE, newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def save_rows(rows: list[dict]) -> None:
    ANN_FILE.parent.mkdir(parents=True, exist_ok=True)
    with open(ANN_FILE, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=COLUMNS)
        w.writeheader()
        w.writerows(rows)


def add_interval(video: Path, label: str, start: float, end: float, person: str = "", session: str = "",
                 uncertainty: str = "", notes: str = "") -> dict:
    label = label.upper()
    if label not in LABELS:
        raise ValueError(f"unknown label {label}; choose from {LABELS}")
    if end <= start:
        raise ValueError("end must be after start")
    cap = cv2.VideoCapture(str(video))
    fps = cap.get(cv2.CAP_PROP_FPS) or 0
    dur = (cap.get(cv2.CAP_PROP_FRAME_COUNT) / fps) if fps else 0
    cap.release()
    if dur and end > dur + 0.05:
        raise ValueError(f"end {end:.2f}s is after the end of the video ({dur:.2f}s)")
    side = video.with_suffix(".json")
    if side.exists() and (not person or not session):
        meta = json.loads(side.read_text(encoding="utf-8"))
        person = person or meta.get("person_id", "")
        session = session or meta.get("session", "")
    row = {"video_file": video_rel(video), "label": label, "start_s": f"{start:.2f}", "end_s": f"{end:.2f}",
           "person_id": person, "session": session, "uncertainty": uncertainty, "notes": notes}
    rows = load_rows()
    rows.append(row)
    save_rows(rows)
    return row


def interactive(video: Path, person: str) -> None:
    cap = cv2.VideoCapture(str(video))
    if not cap.isOpened():
        raise SystemExit(f"cannot open {video}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    pos, playing, label_i, start = 0, False, 0, None
    added: list[dict] = []
    while True:
        cap.set(cv2.CAP_PROP_POS_FRAMES, pos)
        ok, img = cap.read()
        if not ok:
            pos = max(0, n - 1)
            playing = False
            continue
        t = pos / fps
        view = img.copy()
        keys = "  ".join(f"{i + 1}:{l}" for i, l in enumerate(LABELS[:9]))
        cv2.putText(view, f"t={t:6.2f}s  label={LABELS[label_i]}  start={start if start is None else round(start, 2)}",
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(view, keys, (10, view.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        cv2.imshow("SentinelAI annotator", view)
        key = cv2.waitKey(int(1000 / fps) if playing else 0) & 0xFF
        if playing and key == 255:
            pos = min(n - 1, pos + 1)
            continue
        if key == ord(" "):
            playing = not playing
        elif key == ord("d"):
            pos = min(n - 1, pos + 1)
        elif key == ord("a"):
            pos = max(0, pos - 1)
        elif key == ord("l"):
            pos = min(n - 1, pos + int(fps))
        elif key == ord("j"):
            pos = max(0, pos - int(fps))
        elif ord("1") <= key <= ord("9"):
            label_i = key - ord("1")
        elif key == ord("s"):
            start = t
        elif key == ord("e") and start is not None:
            try:
                added.append(add_interval(video, LABELS[label_i], min(start, t), max(start, t), person))
                log.info("Saved %s %.2f-%.2f", LABELS[label_i], min(start, t), max(start, t))
            except ValueError as exc:
                log.error("%s", exc)
            start = None
        elif key == ord("u") and added:
            rows = load_rows()
            last = added.pop()
            if rows and rows[-1] == last:
                save_rows(rows[:-1])
                log.info("Removed %s", last)
        elif key == ord("q"):
            break
    cap.release()
    cv2.destroyAllWindows()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("video", type=Path)
    ap.add_argument("--add", nargs=3, metavar=("LABEL", "START_S", "END_S"))
    ap.add_argument("--person", default="")
    ap.add_argument("--uncertainty", default="", help="e.g. 'start +-0.3s'")
    ap.add_argument("--notes", default="")
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()
    if not args.video.exists():
        raise SystemExit(f"video not found: {args.video}")
    if args.list:
        rel = video_rel(args.video)
        for r in load_rows():
            if r["video_file"] == rel:
                print(f"{r['label']:16s} {r['start_s']:>7s} - {r['end_s']:>7s}  {r['notes']}")
        return
    if args.add:
        lab, s, e = args.add
        try:
            row = add_interval(args.video, lab, float(s), float(e), args.person, "", args.uncertainty, args.notes)
        except ValueError as exc:
            raise SystemExit(f"Not saved: {exc}")
        log.info("Saved %s", row)
        return
    interactive(args.video, args.person)


if __name__ == "__main__":
    main()
