# Robot command reference (protocol v3)

Every message is one JSON object per line. The exact field rules, error codes and timing are in
`docs/PROTOCOL.md`; the laptop-side safety rules in `docs/SAFETY.md`. Examples below leave out
the four common fields (`"version":3,"type":…,"command_id":…,"seq":…`) where it helps reading.

## 1. All commands

### Motors

| `type` | Fields | Effect |
|---|---|---|
| `motor` | `mode:"manual"`, `left`, `right` (−100…100 %), `duration_ms` (50…2000) | manual drive (dashboard arrows, console). Ends by itself after `duration_ms` (`move_done`) |
| `follow` | `target_id`, `left`, `right`, `duration_ms`, `confidence` (0…1) | following drive, only in follow mode for that target |
| `stop` | – | both motors 0, follow mode off. Also acknowledges a communication fault |

Example – forward 30 % for 0.4 s:

```json
{"version":3,"type":"motor","command_id":"L1-17","seq":17,"mode":"manual","left":30,"right":30,"duration_ms":400}
```

The ack echoes the applied (clamped) speeds: `{"…","ok":true,"left":30,"right":30}`.
Speeds are capped by `MAX_SPEED_PCT` (60, firmware) and `robot.max_speed_pct` (40, laptop).
With `MOTORS_ENABLED 0` the robot accepts and tracks motion but never drives the pins.

### Following

| `type` | Fields | Effect |
|---|---|---|
| `follow_mode` | `enabled:true`, `target_id` | enter follow mode for that person (one short beep, LED blinks 500 ms) |
| `follow_mode` | `enabled:false` | leave follow mode, motors 0 |

### LED and buzzer

| `type` | Fields | Effect |
|---|---|---|
| `led` | `state:"on"` / `"off"` / `"blink"`, optional `interval_ms` 100…2000 (default 500) | red LED |
| `buzzer` | `pattern:"short_beep"` (150 ms) / `"warning"` (3 beeps) / `"emergency"` (300/200 ms, stops after 60 s) / `"off"` | buzzer |
| `mute` | – | buzzer off (an emergency stays latched, LED stays on) |

### Safety

| `type` | Fields | Effect |
|---|---|---|
| `emergency_stop` | `reason` (e.g. `"fall_confirmed"`, `"operator"`) | motors 0 **first**, LED on, emergency buzzer, follow off, latched |
| `clear_emergency` | `operator_confirmed:true` | clears the latch (anything else: `NEEDS_CONFIRM`); motors stay stopped |
| `obstacle` | `source:"camera"`, `distance_cm` (0 = unknown) | camera saw something in the path: forward motion and all following motion refused for 1.5 s (repeat while seen) |
| `obstacle_clear` | – | the camera no longer sees it |
| `ping` | – | heartbeat (every 0.15 s from the laptop) |
| `status_request` | – | robot sends a `status` message now |

### Named commands (type `command`)

`{"version":3,"type":"command","command_id":"L1-9","seq":9,"command":"LEFT"}` – readable names
used by the stage-1 test console (`python -m robot.test_console`) and available to the dashboard:
FORWARD, BACKWARD, LEFT, RIGHT (**simulated** – never drive the motors), STOP, LED_ON, LED_OFF,
LED_BLINK, BUZZER_ON (≤ 60 s), BUZZER_OFF, BUZZER_SHORT_BEEP, BUZZER_WARNING, STATUS,
EMERGENCY_STOP. Details and expected acks: docs/STAGE1_COMM_TEST.md §3.

### What the robot sends

* `ack` – one for every line: `ok:true`, or `ok:false` with `error` (and `field`).
* `status` – every second (state, motors, follow mode/target, emergency + reason, obstacle +
  source, sensor distance, watchdog, LED pattern actually shown, buzzer, last command id, uptime).
* events – `hello` (with `comm_fault`), `move_done`, `watchdog_stop`, `obstacle` (own sensor),
  `obstacle_clear`, `buzzer_timeout`, `replaced`.

## 2. What the robot shows (indicator priority)

| Situation | Red LED (`status.led`) | Buzzer | On-board blue LED |
|---|---|---|---|
| Emergency | solid on (`on`) | emergency pattern until `mute`, `clear_emergency` or 60 s | fast blink |
| Obstacle active | 100 ms blink (`blink_obstacle`) | – | link pattern |
| `led` command given | as commanded (`on` / `blink`) | as commanded | link pattern |
| Following | 500 ms blink (`blink_following`) | one short beep at start | link pattern |
| Normal | off | off | solid = laptop linked, short flash = waiting, 1 Hz = no Wi-Fi |

## 3. How the laptop uses the commands

### Falls

```text
AI result                  laptop sends                                  robot
POSSIBLE_FALL (target)     follow_mode {enabled:false}                   stops, leaves follow mode
FALL_SUSPECTED (warning)   follow_mode off; led blink 200; buzzer warning blink + 3 beeps, no latch
recovered / acknowledged   led off                                       light off
FALL_CONFIRMED (emergency) command FALL_DETECTED (state change)          motion + follow stopped, LED blinks, buzzer alert
operator acknowledges      command FALL_CLEARED                           alert off, LED back to ON (model running)
AI model starts / stops    command MODEL_STARTED / MODEL_STOPPED           LED continuously ON / LED and buzzer OFF
                           (re-sent every 0.25 s until acknowledged)
operator                   mute                                          buzzer off, LED stays on
operator reviews + confirms clear_emergency {operator_confirmed:true}    idle, motors stopped
```

### Following

```text
operator clicks person #3   follow_mode {enabled:true, target_id:3}   -> follow mode, beep
every frame while FOLLOWING follow {target_id:3, left, right, duration_ms:400, confidence}
target unseen / uncertain   follow {left:0, right:0, ...}             -> robot holds still
target lost (0.7 s)         follow_mode {enabled:false}               -> operator must select again
obstacle                    obstacle {source:"camera"} (every 0.5 s)  -> robot refuses following motion
path clear for 1 s          obstacle_clear; state FOLLOW_READY        -> operator presses Resume
```

| Camera situation (FOLLOWING) | wheels |
|---|---|
| person centred, box small (far) | equal forward speeds (min 18 %, max 35 %), growing by ≤ 60 %/s |
| person left / right | curve toward them |
| person far to the side | turn in place, no forward |
| box ≥ 80 % of the image height (close) or ≈ 55 % (follow distance estimate) | no forward |
| never | reverse |

### Manual (dashboard, press and hold)

While an arrow is held the dashboard sends `motor {…, duration_ms:400}` every 150 ms; on release
it sends `stop`. Forward = (30, 30), backward = (−30, −30), left = (−30, 30), right = (30, −30).
Refused while following, during an emergency or a communication fault, and (forward) with an
obstacle – reverse and turning stay possible to back away.

## 4. What the robot accepts in each state

| Robot state | `motor` forward | `motor` reverse / turn | `follow` | `follow_mode` on | `led` / `buzzer` | `stop` / `emergency_stop` / `clear_emergency` |
|---|---|---|---|---|---|---|
| idle / moving | yes | yes | needs follow mode | yes | yes | yes |
| following | **no** (`FOLLOW_ACTIVE`) | **no** | yes (same target, confidence ≥ 0.5) | yes | yes | yes |
| obstacle | **no** (`OBSTACLE_AHEAD`) | yes | **no** motion (`OBSTACLE_AHEAD`) | yes | yes | yes |
| emergency | **no** (`EMERGENCY_ACTIVE`) | **no** | **no** | **no** | accepted, emergency pattern wins | yes |

"Forward" means the sum of the two wheel speeds is positive.
