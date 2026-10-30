"""Optional pre/post-event video clip for alerts (OFF by default - privacy).

When enabled (logging.clip_buffer.enabled: true), the last ``pre_s`` seconds of
annotated frames are kept in memory only (never written while nothing happens).
When an alert fires, those frames plus the next ``post_s`` seconds are written
to one short video file in logs/clips/, and only the newest ``max_clips`` files
are kept (older clips are deleted automatically). Nothing is uploaded anywhere.
"""
from __future__ import annotations

import logging
import threading
import time
from collections import deque
from pathlib import Path

import cv2

log = logging.getLogger("sentinel.clips")


class ClipBuffer:
    def __init__(self, directory: str | Path, enabled: bool = False, pre_s: float = 5.0, post_s: float = 5.0,
                 fps: float = 10.0, max_clips: int = 20, max_width: int = 640):
        self.enabled = bool(enabled)
        self.dir = Path(directory)
        self.pre_s, self.post_s, self.fps = float(pre_s), float(post_s), float(fps)
        self.max_clips = int(max_clips)
        self.max_width = int(max_width)
        self._frames: deque = deque()
        self._lock = threading.Lock()
        self._recording: list | None = None
        self._record_until = 0.0
        self._name = ""
        self._last_add = 0.0
        self.saved: list[str] = []

    def add(self, image, now: float | None = None) -> None:
        if not self.enabled:
            return
        now = time.monotonic() if now is None else now
        if now - self._last_add < 1.0 / self.fps:
            return
        self._last_add = now
        h, w = image.shape[:2]
        if w > self.max_width:
            image = cv2.resize(image, (self.max_width, int(h * self.max_width / w)))
        with self._lock:
            self._frames.append((now, image.copy()))
            while self._frames and now - self._frames[0][0] > self.pre_s:
                self._frames.popleft()
            if self._recording is not None:
                self._recording.append(image.copy())
                if now >= self._record_until:
                    frames, name = self._recording, self._name
                    self._recording = None
                    threading.Thread(target=self._write, args=(frames, name), daemon=True).start()

    def trigger(self, label: str, now: float | None = None) -> None:
        if not self.enabled:
            return
        now = time.monotonic() if now is None else now
        with self._lock:
            if self._recording is not None:          # already recording: extend
                self._record_until = now + self.post_s
                return
            self._recording = [f for _, f in self._frames]
            self._record_until = now + self.post_s
            safe = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in label)[:40]
            self._name = time.strftime("%Y%m%d_%H%M%S_") + safe + ".avi"

    def _write(self, frames, name: str) -> None:
        if not frames:
            return
        try:
            self.dir.mkdir(parents=True, exist_ok=True)
            h, w = frames[0].shape[:2]
            path = self.dir / name
            vw = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), self.fps, (w, h))
            for f in frames:
                if f.shape[:2] == (h, w):
                    vw.write(f)
            vw.release()
            self.saved.append(str(path))
            log.info("Saved alert clip %s (%d frames)", path, len(frames))
            clips = sorted(self.dir.glob("*.avi"))
            for old in clips[:-self.max_clips] if self.max_clips > 0 else []:
                old.unlink(missing_ok=True)
        except Exception as exc:     # a clip must never disturb alerting
            log.error("alert clip not saved: %s", exc)

    def status(self) -> dict:
        return {"enabled": self.enabled, "pre_s": self.pre_s, "post_s": self.post_s,
                "recording": self._recording is not None, "saved": len(self.saved), "max_clips": self.max_clips}
