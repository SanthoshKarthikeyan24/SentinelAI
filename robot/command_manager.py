"""Priority command queue between the laptop's decision code and the Wi-Fi client.

Why a queue? Commands come from several threads at once (AI pipeline, follow
controller, dashboard buttons). The manager makes sure that:

  1. an ``emergency_stop`` is never queued: it is latched and written to the socket at
     once, and every queued motion command is removed (nothing queued before an
     emergency may run after it);
  2. a ``stop`` also removes queued motion commands;
  3. only the NEWEST motion command is kept (motion is "coalesced": following sends
     a fresh command every frame, so an older one is never worth sending);
  4. motion commands older than ``max_age_s`` are dropped instead of sent (a robot
     must never act on an old picture);
  5. the same holds for repeated indicator/obstacle commands (newest wins).

Priorities (smaller = sent first):
  0 emergency_stop | 1 stop, clear_emergency | 2 obstacle, obstacle_clear, follow_mode
  3 led, buzzer, mute | 4 motor, follow | 5 ping, status_request
"""
from __future__ import annotations

import itertools
import logging
import threading
import time
from dataclasses import dataclass, field

from .commands import MOTION_TYPES, MOVEMENT_COMMANDS, Command, CommandError, Reply, make_command
from .wifi_client import Pending, RobotClient, RobotNotConnected

log = logging.getLogger("sentinel.commands")

PRIORITY = {"emergency_stop": 0, "stop": 1, "clear_emergency": 1, "obstacle": 2, "obstacle_clear": 2,
            "follow_mode": 2, "led": 3, "buzzer": 3, "mute": 3, "motor": 4, "follow": 4, "ping": 5,
            "status_request": 5}
COALESCE_KEY = {"motor": "motion", "follow": "motion", "obstacle": "obstacle", "obstacle_clear": "obstacle",
                "led": "led", "buzzer": "buzzer"}
# named commands (type "command"): priority and coalescing by name
NAMED_PRIORITY = {"EMERGENCY_STOP": 0, "FALL_DETECTED": 1, "STOP": 1, "FALL_CLEARED": 2, "MODEL_STARTED": 2,
                  "MODEL_STOPPED": 2, "STATUS": 5}
NAMED_COALESCE = {"MODEL_STARTED": "model", "MODEL_STOPPED": "model", "FALL_DETECTED": "fall",
                  "FALL_CLEARED": "fall"}


def priority_of(cmd: Command) -> int:
    if cmd.type == "command":
        name = cmd.get("command")
        return NAMED_PRIORITY.get(name, 4 if name in MOVEMENT_COMMANDS else 3)
    return PRIORITY[cmd.type]


def is_motion(cmd: Command) -> bool:
    return cmd.type in MOTION_TYPES or (cmd.type == "command" and cmd.get("command") in MOVEMENT_COMMANDS)


def coalesce_key(cmd: Command) -> str | None:
    if cmd.type == "command":
        name = cmd.get("command")
        if name in MOVEMENT_COMMANDS:
            return "motion"
        if name.startswith("LED_"):
            return "led"
        if name.startswith("BUZZER_"):
            return "buzzer"
        return NAMED_COALESCE.get(name)
    return COALESCE_KEY.get(cmd.type)


@dataclass
class Ticket:
    """Handle for a submitted command. outcome: queued | sent | dropped | failed."""
    cmd: Command
    priority: int
    created_at: float
    origin: str = ""
    outcome: str = "queued"
    detail: str = ""
    pending: Pending | None = None
    _done: threading.Event = field(default_factory=threading.Event)

    def wait(self, timeout: float = 1.0) -> Reply | None:
        """Wait until sent and acknowledged. None if dropped, failed or no ack in time."""
        t0 = time.monotonic()
        if not self._done.wait(timeout):
            return None
        if self.pending is None:
            return None
        return self.pending.wait(max(0.0, timeout - (time.monotonic() - t0)))

    def _finish(self, outcome: str, detail: str = "") -> None:
        self.outcome, self.detail = outcome, detail
        self._done.set()


class CommandManager:
    def __init__(self, client: RobotClient, max_age_s: float = 0.3, gate=None):
        """gate(cmd) -> str: optional last-moment safety check for motion; non-empty = refuse."""
        self.client = client
        self.max_age_s = max_age_s
        self.gate = gate
        self._queue: list[tuple[int, int, Ticket]] = []
        self._counter = itertools.count()
        self._cv = threading.Condition()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.stats = {"submitted": 0, "sent": 0, "coalesced": 0, "dropped_stale": 0, "dropped_emergency": 0,
                      "dropped_stop": 0, "dropped_gate": 0, "failed": 0}

    # ---------------------------------------------------------------- lifecycle
    def start(self) -> "CommandManager":
        if self._thread and self._thread.is_alive():
            return self
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="robot-commands", daemon=True)
        self._thread.start()
        return self

    def close(self) -> None:
        self._stop.set()
        with self._cv:
            for _, _, t in self._queue:
                t._finish("dropped", "shutting down")
            self._queue.clear()
            self._cv.notify_all()
        if self._thread:
            self._thread.join(timeout=2)

    # --------------------------------------------------------------- submit API
    def submit(self, msg_type: str, origin: str = "", created_at: float | None = None, **fields) -> Ticket:
        """Validate (raises CommandError) and queue a command. Returns at once.
        origin: who asked (operator, follow, camera, ...) - for logs only."""
        cmd = make_command(msg_type, **fields)
        return self.submit_command(cmd, origin, created_at)

    def submit_command(self, cmd: Command, origin: str = "", created_at: float | None = None) -> Ticket:
        prio = priority_of(cmd)
        t = Ticket(cmd, prio, time.monotonic() if created_at is None else created_at, origin)
        with self._cv:
            self.stats["submitted"] += 1
            name = cmd.get("command") if cmd.type == "command" else None
            if cmd.type in ("emergency_stop", "stop") or name in ("EMERGENCY_STOP", "STOP", "FALL_DETECTED"):
                key = "dropped_emergency" if (cmd.type == "emergency_stop" or name == "EMERGENCY_STOP") else "dropped_stop"
                self._drop_where(lambda q: is_motion(q.cmd), key, f"superseded by {name or cmd.type}")
            if cmd.type == "emergency_stop":
                # not queued at all: latched and written to the socket right now, on the caller's
                # thread (the client keeps retrying until the robot acknowledges it)
                self.client.trigger_emergency(cmd.get("reason"))
                self.stats["sent"] += 1
                t._finish("sent", "emergency handed to the link (retried until acknowledged)")
                return t
            k = coalesce_key(cmd)
            if k:
                self._drop_where(lambda q: coalesce_key(q.cmd) == k, "coalesced", "replaced by newer")
            self._queue.append((prio, next(self._counter), t))
            self._queue.sort(key=lambda e: (e[0], e[1]))
            self._cv.notify()
        return t

    def queued(self) -> list[str]:
        with self._cv:
            return [str(t.cmd) for _, _, t in self._queue]

    def status(self) -> dict:
        return {"queued": len(self._queue), **self.stats}

    # ---------------------------------------------------------------- internals
    def _drop_where(self, pred, stat: str, why: str) -> None:
        keep = []
        for e in self._queue:
            if pred(e[2]):
                e[2]._finish("dropped", why)
                self.stats[stat] += 1
            else:
                keep.append(e)
        self._queue = keep

    def _run(self) -> None:
        while not self._stop.is_set():
            with self._cv:
                while not self._queue and not self._stop.is_set():
                    self._cv.wait(0.2)
                if self._stop.is_set():
                    return
                _, _, t = self._queue.pop(0)
            self._send(t)

    def _send(self, t: Ticket) -> None:
        cmd = t.cmd
        if is_motion(cmd):
            age = time.monotonic() - t.created_at
            if age > self.max_age_s:
                self.stats["dropped_stale"] += 1
                t._finish("dropped", f"stale ({age:.2f}s old)")
                return
            if self.gate is not None:
                why = self.gate(cmd)
                if why:
                    self.stats["dropped_gate"] += 1
                    t._finish("dropped", why)
                    return
        try:
            t.pending = self.client.send_command(cmd, background=True)
            self.stats["sent"] += 1
            t._finish("sent")
        except (RobotNotConnected, CommandError) as exc:
            self.stats["failed"] += 1
            t._finish("failed", str(exc))
            log.debug("command %s not sent: %s", cmd, exc)
