"""Shared helpers for the training scripts (paths, logging, JSON writing)."""
from __future__ import annotations

import json
import logging
import platform
import sys
import warnings
from datetime import datetime, timezone
from pathlib import Path

# scikit-learn/joblib version combinations can emit this harmless warning thousands of times
warnings.filterwarnings("ignore", message=".*sklearn.utils.parallel.delayed.*")

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

DATA = ROOT / "data"
RAW_PUBLIC = DATA / "raw" / "public_datasets"
RAW_COLLECTED = DATA / "raw" / "collected_videos"
ANNOTATIONS = DATA / "annotations"
VIDEO_EVENTS = ANNOTATIONS / "video_events"
POSE_FEATURES = DATA / "processed" / "pose_features"
TEMPORAL = DATA / "processed" / "temporal_sequences"
SPLITS = DATA / "splits"
ARTIFACTS = ROOT / "artifacts"
EVALUATION = ROOT / "evaluation"

VIDEO_INDEX = ANNOTATIONS / "video_index.csv"
EVENTS_CSV = VIDEO_EVENTS / "events.csv"

EVENT_COLUMNS = ["video_id", "label", "start_s", "end_s", "raw_label", "person_id",
                 "uncertainty", "notes"]
INDEX_COLUMNS = ["video_id", "dataset", "path", "subject", "session", "category",
                 "duration_s", "fps", "width", "height", "description"]


def get_logger(name: str) -> logging.Logger:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s",
                        datefmt="%H:%M:%S")
    return logging.getLogger(name)


def write_json(path: Path, data) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(data, fh, indent=2, default=_json_default)


def _json_default(o):
    try:
        import numpy as np
        if isinstance(o, np.integer):
            return int(o)
        if isinstance(o, np.floating):
            return float(o)
        if isinstance(o, np.ndarray):
            return o.tolist()
    except ImportError:  # pragma: no cover
        pass
    return str(o)


def software_versions() -> dict:
    import importlib
    out = {"python": platform.python_version(), "platform": platform.platform()}
    for mod in ("numpy", "scipy", "sklearn", "cv2", "onnxruntime", "pandas", "joblib"):
        try:
            out[mod] = importlib.import_module(mod).__version__
        except Exception:  # pragma: no cover
            out[mod] = "not installed"
    return out


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")
