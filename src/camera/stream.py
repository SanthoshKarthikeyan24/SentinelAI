"""Camera input: laptop webcam, Android IP Webcam (MJPEG URL) or a video file.

Live sources are read on a background thread that always keeps only the
newest frame, so slow AI inference never builds up a backlog of old frames
(stale video would delay fall alerts). Video files are read sequentially so
that every frame is analysed during offline tests.

The reader detects:
  * failure to open the source           -> status "error", retries with backoff
  * stalled stream (no new frame)        -> status "stalled", reconnects
  * end of a video file                  -> status "ended"
"""
from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

log = logging.getLogger("sentinel.camera")


@dataclass
class Frame:
    image: np.ndarray
    frame_id: int
    timestamp: float       # seconds: wall clock for live sources, video time for files
    source_fps: float


def describe_source(source: int | str) -> str:
    if isinstance(source, int):
        return f"webcam #{source}"
    s = str(source)
    if s.startswith(("http://", "https://", "rtsp://")):
        return f"network stream {s}"
    return f"video file {Path(s).name}"


def is_live(source: int | str) -> bool:
    return isinstance(source, int) or str(source).startswith(("http://", "https://", "rtsp://"))


class CameraStream:
    def __init__(self, source: int | str, width: int = 0, height: int = 0,
                 stall_timeout_s: float = 3.0, reconnect_delay_s: float = 2.0,
                 reconnect_max_delay_s: float = 15.0, max_reconnect_attempts: int = 0,
                 file_realtime: bool = False):
        self.source = source
        self.width, self.height = int(width), int(height)
        self.stall_timeout_s = float(stall_timeout_s)
        self.reconnect_delay_s = float(reconnect_delay_s)
        self.reconnect_max_delay_s = float(reconnect_max_delay_s)
        self.max_reconnect_attempts = int(max_reconnect_attempts)
        self.file_realtime = bool(file_realtime)
        self.live = is_live(source)
        self.status = "init"              # init|connecting|ok|stalled|error|ended|stopped
        self.last_error = ""
        self.reconnects = 0
        self.fps = 0.0
        self._cap: cv2.VideoCapture | None = None
        self._lock = threading.Lock()
        self._latest: Frame | None = None
        self._latest_consumed = True
        self._frame_counter = 0
        self._last_frame_wall = 0.0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._file_start_wall = 0.0

    # --------------------------------------------------------------- opening
    def _open(self) -> bool:
        self.status = "connecting"
        src = self.source
        if not self.live and not Path(str(src)).exists():
            self.status, self.last_error = "error", f"file not found: {src}"
            return False
        if isinstance(src, int):
            # CAP_DSHOW opens Windows webcams faster and more reliably than the default backend
            backend = cv2.CAP_DSHOW if hasattr(cv2, "CAP_DSHOW") and _is_windows() else cv2.CAP_ANY
            cap = cv2.VideoCapture(src, backend)
        else:
            cap = cv2.VideoCapture(str(src))
        if not cap.isOpened():
            cap.release()
            self.status = "error"
            self.last_error = f"could not open {describe_source(src)}"
            return False
        if self.width and self.height:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        if self.live:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.fps = cap.get(cv2.CAP_PROP_FPS) or 0.0
        self._cap = cap
        self._last_frame_wall = time.time()
        self._file_start_wall = time.time()
        self.status = "ok"
        self.last_error = ""
        log.info("Opened %s (reported fps %.1f)", describe_source(src), self.fps)
        return True

    def open(self) -> bool:
        """Open the source. For live sources this starts the reader thread."""
        if self.live:
            self._stop.clear()
            self._thread = threading.Thread(target=self._reader_loop, name="camera-reader", daemon=True)
            self._thread.start()
            # wait briefly for the first frame so callers get a useful status
            deadline = time.time() + max(self.stall_timeout_s, 5.0)
            while time.time() < deadline and self._latest is None and not self._stop.is_set():
                if self.status == "error" and self.max_reconnect_attempts == 1:
                    break
                time.sleep(0.05)
            return self._latest is not None
        return self._open()

    # ---------------------------------------------------------- live reading
    def _reader_loop(self) -> None:
        delay = self.reconnect_delay_s
        attempts = 0
        while not self._stop.is_set():
            if self._cap is None:
                if not self._open():
                    attempts += 1
                    log.warning("%s (attempt %d); retrying in %.1fs", self.last_error, attempts, delay)
                    if self.max_reconnect_attempts and attempts >= self.max_reconnect_attempts:
                        self.status = "error"
                        return
                    self._stop.wait(delay)
                    delay = min(delay * 2, self.reconnect_max_delay_s)
                    continue
                attempts, delay = 0, self.reconnect_delay_s
            ok, img = self._cap.read()
            now = time.time()
            if ok and img is not None and img.size:
                self._frame_counter += 1
                with self._lock:
                    self._latest = Frame(img, self._frame_counter, now, self.fps)
                    self._latest_consumed = False
                self._last_frame_wall = now
                if self.status != "ok":
                    self.status = "ok"
            else:
                if now - self._last_frame_wall > self.stall_timeout_s:
                    self.status = "stalled"
                    self.last_error = f"no frames for {self.stall_timeout_s:.1f}s"
                    log.warning("Stream stalled; reconnecting")
                    self._release_cap()
                    self.reconnects += 1
                else:
                    time.sleep(0.01)

    def _release_cap(self) -> None:
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:  # pragma: no cover
                pass
        self._cap = None

    # ---------------------------------------------------------------- reading
    def read(self, timeout: float = 1.0) -> Frame | None:
        """Return the next frame, or None if none is available (check ``status``)."""
        if self.live:
            deadline = time.time() + timeout
            while time.time() < deadline:
                with self._lock:
                    if self._latest is not None and not self._latest_consumed:
                        self._latest_consumed = True
                        return self._latest
                if time.time() - self._last_frame_wall > self.stall_timeout_s and self.status == "ok":
                    self.status = "stalled"
                time.sleep(0.005)
            return None
        # video file: sequential
        if self._cap is None:
            return None
        ok, img = self._cap.read()
        if not ok or img is None:
            self.status = "ended"
            return None
        self._frame_counter += 1
        fps = self.fps if self.fps > 0 else 30.0
        ts = (self._frame_counter - 1) / fps
        if self.file_realtime:
            wait = self._file_start_wall + ts - time.time()
            if wait > 0:
                time.sleep(wait)
        return Frame(img, self._frame_counter, ts, fps)

    def is_stalled(self) -> bool:
        return self.status in ("stalled", "error", "connecting")

    def release(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._release_cap()
        self.status = "stopped"

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.release()


def _is_windows() -> bool:
    import os
    return os.name == "nt"
