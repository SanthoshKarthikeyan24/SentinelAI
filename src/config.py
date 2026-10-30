"""Configuration loading and validation.

All runtime thresholds live in config/settings.yaml. This module loads that
file, checks that required sections and keys exist with sensible types, and
returns a nested ``Config`` object that supports both ``cfg.fall.window_s`` and
``cfg["fall"]["window_s"]`` access.
"""
from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

import yaml

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "config" / "settings.yaml"

# section -> {key: expected type(s)}. Only keys the code relies on are listed.
_SCHEMA: dict[str, dict[str, tuple[type, ...]]] = {
    "camera": {
        "source": (int, str),
        "stall_timeout_s": (int, float),
        "reconnect_delay_s": (int, float),
        "reconnect_max_delay_s": (int, float),
        "max_reconnect_attempts": (int,),
        "file_realtime": (bool,),
    },
    "models": {
        "pose_model": (str,),
        "object_model": (str,),
        "posture_model": (str,),
        "fall_model": (str,),
        "providers": (list,),
        "num_threads": (int,),
    },
    "detection": {
        "pose_imgsz": (int,),
        "person_conf": (int, float),
        "nms_iou": (int, float),
        "keypoint_conf": (int, float),
        "object_detection_enabled": (bool,),
        "object_imgsz": (int,),
        "object_conf": (int, float),
        "object_every_n_frames": (int,),
        "object_classes": (list,),
    },
    "tracking": {
        "iou_match_threshold": (int, float),
        "max_center_distance": (int, float),
        "min_hits": (int,),
        "max_missed_s": (int, float),
        "history_s": (int, float),
    },
    "posture": {
        "use_trained_model": (bool,),
        "min_visible_keypoints": (int,),
        "unknown_proba_threshold": (int, float),
        "smoothing_window": (int,),
    },
    "fall": {
        "use_trained_model": (bool,),
        "window_s": (int, float),
        "model_threshold": (int, float, str),
        "descent_speed_thresh": (int, float),
        "descent_drop_thresh": (int, float),
        "torso_angle_low_deg": (int, float),
        "possible_timeout_s": (int, float),
        "suspect_min_down_s": (int, float),
        "confirm_down_s": (int, float),
        "recovery_up_s": (int, float),
        "cooldown_s": (int, float),
        "lost_track_grace_s": (int, float),
        "max_timer_step_s": (int, float),
    },
    "alerts": {"require_acknowledgement": (bool,), "beep_on_alert": (bool,)},
    "logging": {
        "directory": (str,),
        "events_jsonl": (str,),
        "events_csv": (str,),
        "save_alert_snapshots": (bool,),
        "clip_buffer": (dict,),
    },
    "robot": {
        "enabled": (bool,),
        "host": (str,),
        "port": (int,),
        "heartbeat_s": (int, float),
        "ack_timeout_s": (int, float),
        "max_missed_acks": (int,),
        "connect_timeout_s": (int, float),
        "reconnect_delay_s": (int, float),
        "reconnect_max_delay_s": (int, float),
        "emergency_retry_s": (int, float),
        "max_command_age_s": (int, float),
        "status_stale_s": (int, float),
        "max_speed_pct": (int,),
        "default_speed_pct": (int,),
        "manual_hold_ms": (int,),
        "manual_active_s": (int, float),
        "signal_on_suspected": (bool,),
        "follow": (dict,),
    },
    "dashboard": {
        "host": (str,),
        "port": (int,),
        "jpeg_quality": (int,),
        "max_recent_events": (int,),
    },
}


class ConfigError(ValueError):
    """Raised when the configuration file is missing or invalid."""


class Config(dict):
    """A dict whose keys are also readable as attributes (recursively)."""

    def __init__(self, data: dict[str, Any] | None = None):
        super().__init__()
        for key, value in (data or {}).items():
            self[key] = Config(value) if isinstance(value, dict) else value

    def __getattr__(self, name: str) -> Any:
        try:
            return self[name]
        except KeyError as exc:
            raise AttributeError(f"Config has no key '{name}'") from exc

    def __setattr__(self, name: str, value: Any) -> None:
        self[name] = Config(value) if isinstance(value, dict) else value

    def to_dict(self) -> dict[str, Any]:
        return {k: (v.to_dict() if isinstance(v, Config) else copy.deepcopy(v)) for k, v in self.items()}


def validate(data: dict[str, Any]) -> list[str]:
    """Return a list of human-readable problems (empty list = valid)."""
    problems: list[str] = []
    for section, keys in _SCHEMA.items():
        if section not in data or not isinstance(data[section], dict):
            problems.append(f"missing section '{section}'")
            continue
        for key, types in keys.items():
            if key not in data[section]:
                problems.append(f"missing key '{section}.{key}'")
                continue
            value = data[section][key]
            # bool is a subclass of int; do not accept True where a number is expected
            if isinstance(value, bool) and bool not in types:
                problems.append(f"'{section}.{key}' must be {_type_names(types)}, got bool")
            elif not isinstance(value, types):
                problems.append(
                    f"'{section}.{key}' must be {_type_names(types)}, got {type(value).__name__}"
                )
    # value checks: run for every key that exists and has a usable type
    def number(section: str, key: str):
        v = data.get(section, {}).get(key) if isinstance(data.get(section), dict) else None
        return v if isinstance(v, (int, float)) and not isinstance(v, bool) else None

    mt = data.get("fall", {}).get("model_threshold") if isinstance(data.get("fall"), dict) else None
    if isinstance(mt, str) and mt.strip().lower() != "auto":
        problems.append("'fall.model_threshold' must be a number between 0 and 1 or 'auto'")
    for section, key in [("detection", "person_conf"), ("detection", "keypoint_conf"),
                         ("detection", "object_conf"), ("posture", "unknown_proba_threshold"),
                         ("fall", "model_threshold")]:
        v = number(section, key)
        if v is not None and not 0.0 <= float(v) <= 1.0:
            problems.append(f"'{section}.{key}' must be between 0 and 1")
    for section, key in [("detection", "object_every_n_frames"), ("posture", "smoothing_window"),
                         ("tracking", "min_hits"), ("dashboard", "port")]:
        v = number(section, key)
        if v is not None and v < 1:
            problems.append(f"'{section}.{key}' must be >= 1")
    for section, key in [("fall", "window_s"), ("camera", "stall_timeout_s"), ("tracking", "max_missed_s"),
                         ("robot", "heartbeat_s"), ("robot", "ack_timeout_s")]:
        v = number(section, key)
        if v is not None and v <= 0:
            problems.append(f"'{section}.{key}' must be > 0")
    for key in ("max_speed_pct", "default_speed_pct"):
        v = number("robot", key)
        if v is not None and not 0 <= v <= 100:
            problems.append(f"'robot.{key}' must be between 0 and 100")
    v = number("robot", "manual_hold_ms")
    if v is not None and not 50 <= v <= 2000:
        problems.append("'robot.manual_hold_ms' must be between 50 and 2000 (firmware limits)")
    v = number("robot", "status_stale_s")
    if v is not None and v < 1.5:
        problems.append("'robot.status_stale_s' must be >= 1.5 (the robot sends status every 1 s)")
    fol = data.get("robot", {}).get("follow") if isinstance(data.get("robot"), dict) else None
    if isinstance(fol, dict):
        try:
            from robot.follow import FollowParams
            FollowParams.from_config(fol)
        except (TypeError, ValueError) as exc:
            problems.append(str(exc))
    hb = number("robot", "heartbeat_s")
    if hb is not None and hb >= 0.4:
        problems.append("'robot.heartbeat_s' must be < 0.4 s so the robot watchdog (500 ms) never trips on an idle link")
    return problems


def _type_names(types: tuple[type, ...]) -> str:
    return " or ".join(t.__name__ for t in types)


def resolve_path(path: str | Path) -> Path:
    """Resolve a config path relative to the project root."""
    p = Path(path)
    return p if p.is_absolute() else (PROJECT_ROOT / p)


def load_config(path: str | Path | None = None, overrides: dict[str, Any] | None = None) -> Config:
    """Load, override and validate a YAML configuration file.

    ``overrides`` is a nested dict merged on top of the file, e.g.
    ``{"camera": {"source": "video.mp4"}}``.
    """
    cfg_path = Path(path) if path else DEFAULT_CONFIG_PATH
    if not cfg_path.exists():
        raise ConfigError(f"Configuration file not found: {cfg_path}")
    try:
        with open(cfg_path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
    except yaml.YAMLError as exc:
        raise ConfigError(f"Could not parse {cfg_path}: {exc}") from exc
    if overrides:
        _deep_merge(data, overrides)
    problems = validate(data)
    if problems:
        raise ConfigError(f"Invalid configuration in {cfg_path}:\n  - " + "\n  - ".join(problems))
    return Config(data)


def _deep_merge(base: dict[str, Any], extra: dict[str, Any]) -> None:
    for key, value in extra.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _deep_merge(base[key], value)
        else:
            base[key] = value


def parse_source(value: Any) -> int | str:
    """Turn a CLI/config camera source into an int webcam index or a string URL/path."""
    if isinstance(value, int):
        return value
    text = str(value).strip()
    return int(text) if text.isdigit() else text
