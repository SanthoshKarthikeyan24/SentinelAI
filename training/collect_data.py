"""Record short labelled-activity clips with the phone camera or webcam.

SAFETY AND CONSENT
  * Only record people who have agreed to it. The script asks you to confirm.
  * Never ask anyone to perform a real fall. Fall-like clips must be SAFE,
    staged demonstrations onto a thick mattress / crash mat, done slowly
    by a willing adult, or taken from existing datasets.
  * Use a pseudonym (P01, P02 ...) as person id - not real names.

Each clip is saved to data/raw/collected_videos/<session>_<person>_<time>.mp4
with a .json sidecar (session, person id, planned activity, notes).
Label the clip afterwards with training/annotate_videos.py.

Interactive (preview window):
    python training/collect_data.py --source http://PHONE_IP:8080/video --person P01 --activity SITTING
        r = start/stop recording   q = quit
Non-interactive (fixed duration, no window):
    python training/collect_data.py --source 0 --person P01 --activity WALKING --seconds 10 --headless --yes
"""
from __future__ import annotations

import argparse
import json
import time
from datetime import datetime

import cv2

from common import RAW_COLLECTED, get_logger
from src.camera.stream import CameraStream
from src.config import parse_source

log = get_logger("collect")
ACTIVITIES = ["STANDING", "WALKING", "SITTING", "BENDING", "KNEELING", "LYING",
              "FALL_LIKE_EVENT", "RECOVERY", "MIXED"]


class ClipWriter:
    def __init__(self, session: str, person: str, activity: str, notes: str, fps: float):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        RAW_COLLECTED.mkdir(parents=True, exist_ok=True)
        self.path = RAW_COLLECTED / f"{session}_{person}_{activity.lower()}_{stamp}.mp4"
        self.meta = {"file": self.path.name, "session": session, "person_id": person,
                     "planned_activity": activity, "notes": notes, "recorded": stamp, "fps": fps,
                     "consent_confirmed": True}
        self.fps = fps
        self.writer = None
        self.frames = 0

    def write(self, img) -> None:
        if self.writer is None:
            h, w = img.shape[:2]
            self.writer = cv2.VideoWriter(str(self.path), cv2.VideoWriter_fourcc(*"mp4v"), self.fps, (w, h))
            if not self.writer.isOpened():
                raise IOError(f"Could not open a video writer for {self.path}")
        self.writer.write(img)
        self.frames += 1

    def close(self) -> None:
        if self.writer is not None:
            self.writer.release()
        self.meta["frames"] = self.frames
        self.meta["duration_s"] = round(self.frames / self.fps, 2)
        self.path.with_suffix(".json").write_text(json.dumps(self.meta, indent=2), encoding="utf-8")
        log.info("Saved %s (%d frames, %.1f s)", self.path.name, self.frames, self.frames / self.fps)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", default="0")
    ap.add_argument("--person", required=True, help="pseudonym such as P01")
    ap.add_argument("--activity", required=True, choices=ACTIVITIES)
    ap.add_argument("--session", default=datetime.now().strftime("S%Y%m%d"))
    ap.add_argument("--notes", default="")
    ap.add_argument("--fps", type=float, default=15.0, help="frames per second written to the file")
    ap.add_argument("--seconds", type=float, default=0, help="headless mode: clip length")
    ap.add_argument("--headless", action="store_true", help="no preview window (needs --seconds)")
    ap.add_argument("--yes", action="store_true", help="confirm consent non-interactively")
    args = ap.parse_args()

    if not args.yes:
        ans = input("Has everyone who will appear in the video agreed to be recorded? (yes/no): ")
        if ans.strip().lower() not in ("y", "yes"):
            raise SystemExit("Recording cancelled: consent not confirmed.")
    if args.activity == "FALL_LIKE_EVENT":
        log.warning("Fall-like clips must be SAFE staged demonstrations onto a mattress. Never a real fall.")
    if args.headless and args.seconds <= 0:
        raise SystemExit("--headless needs --seconds")

    cam = CameraStream(parse_source(args.source), max_reconnect_attempts=3)
    if not cam.open():
        raise SystemExit(f"Camera not available: {cam.last_error or cam.status}")
    clip: ClipWriter | None = None
    interval = 1.0 / args.fps
    next_t = time.time()
    try:
        if args.headless:
            clip = ClipWriter(args.session, args.person, args.activity, args.notes, args.fps)
            end = time.time() + args.seconds
            while time.time() < end or (not cam.live and clip.frames < args.seconds * args.fps):
                f = cam.read()
                if f is None:
                    if cam.status in ("ended", "error"):
                        break
                    continue
                if cam.live:
                    if time.time() < next_t:
                        continue
                    next_t += interval
                elif (f.frame_id - 1) % max(1, round((f.source_fps or args.fps) / args.fps)):
                    continue   # video file: keep every k-th frame to match --fps
                clip.write(f.image)
                if not cam.live and clip.frames >= args.seconds * args.fps:
                    break
            return
        log.info("Press r to start/stop recording, q to quit")
        while True:
            f = cam.read()
            if f is None:
                if cam.status in ("ended", "error"):
                    break
                continue
            view = f.image.copy()
            if clip is not None and time.time() >= next_t:
                next_t += interval
                clip.write(f.image)
            label = f"REC {clip.frames / args.fps:.1f}s" if clip else "preview - r to record"
            cv2.putText(view, f"{args.activity} | {label}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 0, 255) if clip else (0, 255, 0), 2)
            cv2.imshow("SentinelAI data collection", view)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("r"):
                if clip is None:
                    clip = ClipWriter(args.session, args.person, args.activity, args.notes, args.fps)
                    next_t = time.time()
                else:
                    clip.close()
                    clip = None
            elif key == ord("q"):
                break
    finally:
        if clip is not None:
            clip.close()
        cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
