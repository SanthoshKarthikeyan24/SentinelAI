"""Alert management: raise, deduplicate, acknowledge and notify.

An alert is raised for FALL_CONFIRMED (severity "emergency") and for
FALL_SUSPECTED / TRACK_LOST_DURING_FALL (severity "warning"). Emergency
alerts stay active until a person acknowledges them on the dashboard.

Listeners (callables) receive every new alert. Phase 2 will register the
ESP32 communication client as a listener so the robot stops and sounds the
buzzer; in Phase 1 there is no robot dependency.
"""
from __future__ import annotations

import logging
import threading
import time
import uuid
from dataclasses import asdict, dataclass, field
from typing import Callable

log = logging.getLogger("sentinel.alerts")

SEVERITY = {"FALL_CONFIRMED": "emergency", "FALL_SUSPECTED": "warning",
            "TRACK_LOST_DURING_FALL": "warning"}


@dataclass
class Alert:
    alert_id: str
    event_type: str
    severity: str
    track_id: int | None
    message: str
    created: float = field(default_factory=time.time)
    acknowledged: bool = False
    acknowledged_at: float | None = None

    def to_dict(self) -> dict:
        return asdict(self)


class AlertManager:
    def __init__(self, require_acknowledgement: bool = True, beep: bool = False):
        self.require_ack = require_acknowledgement
        self.beep = beep
        self._alerts: list[Alert] = []
        self._lock = threading.Lock()
        self._listeners: list[Callable[[Alert], None]] = []

    def add_listener(self, fn: Callable[[Alert], None]) -> None:
        self._listeners.append(fn)

    def handle_event(self, event: dict) -> Alert | None:
        etype = event.get("type")
        severity = SEVERITY.get(etype)
        if severity is None:
            return None
        tid = event.get("track_id")
        with self._lock:
            # deduplicate: one active alert per (track, type)
            for a in self._alerts:
                if not a.acknowledged and a.track_id == tid and a.event_type == etype:
                    return None
            alert = Alert(uuid.uuid4().hex[:8], etype, severity, tid,
                          f"{etype.replace('_', ' ').title()} - person #{tid}: {event.get('reason', '')}")
            if not self.require_ack and severity == "warning":
                alert.acknowledged = True
            self._alerts.append(alert)
            self._alerts = self._alerts[-200:]
        log.warning("ALERT [%s] %s", severity, alert.message)
        if self.beep and severity == "emergency":
            _beep()
        for fn in list(self._listeners):
            try:
                fn(alert)
            except Exception as exc:  # a broken listener must never stop alerting
                log.error("alert listener failed: %s", exc)
        return alert

    def acknowledge(self, alert_id: str | None = None) -> int:
        """Acknowledge one alert (or all when alert_id is None). Returns count."""
        n = 0
        with self._lock:
            for a in self._alerts:
                if not a.acknowledged and (alert_id is None or a.alert_id == alert_id):
                    a.acknowledged = True
                    a.acknowledged_at = time.time()
                    n += 1
        return n

    def active(self) -> list[Alert]:
        with self._lock:
            return [a for a in self._alerts if not a.acknowledged]

    def recent(self, n: int = 20) -> list[Alert]:
        with self._lock:
            return list(self._alerts[-n:])

    @property
    def emergency_active(self) -> bool:
        return any(a.severity == "emergency" for a in self.active())


def _beep() -> None:
    try:
        import winsound  # Windows only
        threading.Thread(target=lambda: [winsound.Beep(1200, 300) for _ in range(3)], daemon=True).start()
    except Exception:
        pass
