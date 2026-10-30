"""Structured, thread-safe event log written to JSONL and CSV.

Robot-related records carry, in ``extra.robot`` and in the CSV robot columns:
the command and its ack, commanded vs robot-reported motor state, LED and buzzer
state, link status - so a log reader can see what the laptop *asked for*
separately from what the ESP32 *reported*. ``uncertainty`` records what the
system could not know (e.g. "camera heuristic, not a distance measurement").

If an existing CSV has an older header, it is renamed (events_<time>.csv) and a
new file is started, so columns never shift. No camera images are stored here.
"""
from __future__ import annotations

import csv
import json
import threading
import uuid
from datetime import datetime
from pathlib import Path
from typing import Any

CSV_FIELDS = ["timestamp", "event_id", "track_id", "event_type", "previous_posture",
              "current_posture", "fall_state", "confidence", "reason", "acknowledged", "stream_time_s",
              "robot_command", "robot_ack", "robot_comm", "robot_state", "motors_commanded", "motors_reported",
              "led", "buzzer", "emergency_reason", "reset_time", "uncertainty"]


class EventLogger:
    def __init__(self, directory: str | Path, jsonl_name: str = "events.jsonl",
                 csv_name: str = "events.csv"):
        self.dir = Path(directory)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.jsonl_path = self.dir / jsonl_name
        self.csv_path = self.dir / csv_name
        self._lock = threading.Lock()
        if self.csv_path.exists() and self.csv_path.stat().st_size > 0:
            with open(self.csv_path, encoding="utf-8") as fh:
                header = fh.readline().strip().split(",")
            if header != CSV_FIELDS:
                old = self.csv_path.with_name(f"{self.csv_path.stem}_{datetime.now():%Y%m%d_%H%M%S}.csv")
                self.csv_path.rename(old)
        if not self.csv_path.exists() or self.csv_path.stat().st_size == 0:
            with open(self.csv_path, "w", newline="", encoding="utf-8") as fh:
                csv.DictWriter(fh, fieldnames=CSV_FIELDS).writeheader()

    def log(self, event_type: str, *, track_id: int | None = None, previous_posture: str = "",
            current_posture: str = "", fall_state: str = "", confidence: float | None = None,
            reason: str = "", acknowledged: bool = False, stream_time_s: float | None = None,
            extra: dict[str, Any] | None = None) -> dict[str, Any]:
        record = {
            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
            "event_id": uuid.uuid4().hex[:12],
            "track_id": track_id,
            "event_type": event_type,
            "previous_posture": previous_posture,
            "current_posture": current_posture,
            "fall_state": fall_state,
            "confidence": None if confidence is None else round(float(confidence), 4),
            "reason": reason,
            "acknowledged": bool(acknowledged),
            "stream_time_s": None if stream_time_s is None else round(float(stream_time_s), 3),
        }
        if extra:
            record["extra"] = extra
        row = dict(record)
        if extra:
            r = extra.get("robot") or {}
            row.update(robot_command=extra.get("robot_command", ""), robot_ack=extra.get("robot_ack", ""),
                       robot_comm=r.get("comm", ""), robot_state=r.get("robot_state") or "",
                       motors_commanded=r.get("motors_commanded") or "", motors_reported=r.get("motors_reported") or "",
                       led=r.get("led") or "", buzzer=r.get("buzzer") or "",
                       emergency_reason=extra.get("emergency_reason", ""), reset_time=extra.get("reset_time", ""),
                       uncertainty=extra.get("uncertainty", ""))
        with self._lock:
            with open(self.jsonl_path, "a", encoding="utf-8") as fh:
                fh.write(json.dumps(record) + "\n")
            with open(self.csv_path, "a", newline="", encoding="utf-8") as fh:
                csv.DictWriter(fh, fieldnames=CSV_FIELDS, extrasaction="ignore").writerow(row)
        return record

    def read_recent(self, n: int = 50) -> list[dict[str, Any]]:
        if not self.jsonl_path.exists():
            return []
        with self._lock:
            lines = self.jsonl_path.read_text(encoding="utf-8").splitlines()[-n:]
        out = []
        for line in lines:
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                continue
        return out
