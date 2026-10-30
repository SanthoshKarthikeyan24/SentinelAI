# Safety design

SentinelAI has **two independent safety layers**. Either one alone stops the robot.

1. **The robot (ESP32 firmware)** checks every message and stops by itself: watchdog (500 ms
   without a valid message while moving), every movement expires (≤ 2 s), disconnect = stop,
   emergency latch, obstacle refusal, speed cap, motors disabled in safe mode. See
   docs/PROTOCOL.md §7.
2. **The laptop** – one *safety supervisor* (`src/safety/safety_supervisor.py`) that every
   motion decision must ask first, a *follow state machine* (`src/safety/robot_state_machine.py`)
   and a *priority command queue* (`robot/command_manager.py`).

"Commanded" is never treated as "done": the dashboard shows what the laptop asked for next to
what the ESP32 reports in its status message, and the ESP32 can only report what it set on its
pins – it cannot feel the wheels turning.

## 1. Safety levels (highest priority first)

| # | Level | Becomes active when | Blocks |
|---|---|---|---|
| 1 | FIRMWARE_EMERGENCY | robot status says `emergency_active` | all motion |
| 2 | EMERGENCY_STOP | laptop latched an emergency (fall confirmed / E-STOP button) | all motion |
| 3 | COMM_FAULT | robot not connected, no status for `status_stale_s` (2.5 s), robot `watchdog_ok:false`, or a latched communication fault | all motion |
| 4 | OBSTACLE | robot reports an obstacle (camera report or its sensor), or the camera check sees one | following; forward manual motion (reverse/turning allowed) |
| 5 | FALL_RESPONSE | an unresolved possible-fall warning or confirmed fall | following and manual motion |
| 6 | TARGET_LOST | following is TARGET_UNCERTAIN or TARGET_LOST | following motion |
| 7 | FOLLOWING | following the selected person | manual motion |
| 8 | MANUAL | an arrow button was used in the last 2 s | – |
| 9 | IDLE | nothing else | – |

The dashboard shows the top level and its reason. Permissions are the **combination of all
active conditions** – e.g. an obstacle plus a fall warning blocks forward motion *and* manual
motion.

## 2. Latches – nothing resumes by itself

| Latch | Set by | Cleared only by |
|---|---|---|
| Emergency | fall confirmed, E-STOP button, `emergency_stop` | operator presses **Clear emergency…**, confirms the dialog, and the ESP32 accepts `clear_emergency {operator_confirmed:true}`. Motors stay stopped; following must be started again |
| Communication fault | link lost while moving or following; robot `watchdog_stop`; robot `hello` with `comm_fault:true` | operator presses **STOP** (a fresh control state) on a healthy link |
| Obstacle stop (following) | obstacle while following | path clear for 1 s → FOLLOW_READY; operator presses **Resume** (`obstacle_auto_resume: false`) |
| Target lost | person not seen for 0.7 s, or uncertain for 3 s | operator selects a person again |

After any reconnection the robot is stopped and not following, and the laptop does not re-send
any motion. The only command re-sent automatically is a latched **emergency** (if the robot
rebooted and lost it).

## 3. What happens on a fall

| AI result | Robot | Dashboard / log |
|---|---|---|
| POSSIBLE_FALL of the followed person | following ends (fall response) | event logged |
| FALL_SUSPECTED (warning alert) | following ends, LED blinks 200 ms, warning beeps; no latch | warning alert, assessment "possible fall" |
| FALL_CONFIRMED (emergency alert) | `FALL_DETECTED`: motors 0 first, following off, LED blinks, fall buzzer alert (max 60 s); motion refused by the ESP32 (`FALL_ALERT_ACTIVE`) and by the supervisor until the operator acknowledges → `FALL_CLEARED` | red banner, optional clip |
| track lost during a fall | warning alert ("tracking lost") | assessment "tracking lost" |
| Operator | reviews the person, acknowledges the alert, clears the emergency | ack time and reset time logged |

SentinelAI **cannot call emergency services** and is not a medical device. It never claims a
fall is certain: the label is "fall confirmed by model".

### Priority of the robot's LED and buzzer (highest first)

| # | State | LED | Buzzer |
|---|---|---|---|
| 1 | Emergency stop latched (operator E-STOP) | solid ON | emergency alarm (≤ 60 s, BUZZER_OFF/mute silences) |
| 2 | Fall alert (`FALL_DETECTED` until `FALL_CLEARED`) | blinking 250 ms | beep-beep pattern (≤ 60 s, BUZZER_OFF/mute silences; LED keeps blinking) |
| 3 | Obstacle | fast blink | – |
| 4 | LED / buzzer test commands | as commanded | as commanded |
| 5 | AI model running (`MODEL_STARTED`) | solid ON | – |
| 6 | Following | slow blink | – |
| 7 | Nothing | off | off |

An emergency stop therefore overrides a fall alert (it is the stronger state), but does not erase it:
after the operator resets the emergency, a fall alert that is still active shows again and needs its
own `FALL_CLEARED`. A disconnection turns the "model running" light off (it needs a live laptop) but
keeps a fall alert; the laptop re-sends its current state after reconnecting. `MODEL_STOPPED` ends a
fall alert as well.

## 4. Commands: ordering and freshness

* `emergency_stop` is never queued – it is written to the socket immediately and re-sent every
  0.25 s until acknowledged; it removes all queued motion.
* `stop` removes queued motion. Only the newest motion command is kept.
* Motion older than `max_command_age_s` (0.3 s) is dropped – the robot never acts on an old
  picture.
* Manual arrows are **press and hold**: each request lasts `manual_hold_ms` (400 ms) on the
  robot and is repeated every 150 ms while held; releasing sends `stop`; a crashed browser simply
  stops sending, and the robot stops within 400 ms.
* Following speed may grow by at most 60 %/s (`max_accel_pct_per_s`); slowing down and stopping
  are never delayed. The robot never reverses while following.

## 5. Following rules

* The operator chooses the person (click on the video or the "follow" button in the table).
  The system never picks or switches to another person – a different track id is never followed.
* A lying person, or a person in a fall state, cannot be selected.
* Distance is estimated from the person's box height – a rough cue, **not a measurement**.
* Camera obstacle detection is a heuristic (it misses low, thin, transparent and unknown objects);
  it is not a safety sensor. Use low speeds and an empty test area.

## 6. Safe shutdown

Closing the app (Ctrl+C / window) disables following, sends `stop`, waits for the ack, then closes
the link. Pressing **Disconnect** on the dashboard does the same. If the laptop crashes instead,
the robot's watchdog / disconnect detection stops it.

## 7. Network and credentials

* The dashboard listens on 127.0.0.1 only (`dashboard.host`); the robot port 3333 must stay on
  the local network – never forward it on a router or expose it to the internet.
* Wi-Fi passwords live in `firmware/sentinel_robot/secrets.h`, which is in `.gitignore`;
  `config.h` contains only placeholders and the firmware never prints a password.
* Messages are size-limited (≤ 200 characters to the robot, ≤ 640 back; dashboard requests
  ≤ 1 KB) and strictly validated on both sides.
* The protocol has no authentication: anyone on the same Wi-Fi could send commands. Use a
  private hotspot with a strong password for the robot.
