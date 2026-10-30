# Laptop ↔ ESP32 protocol v3 (JSON)

One protocol is used everywhere: the ESP32 firmware (`firmware/sentinel_robot/robot_core.cpp`),
the laptop client (`robot/commands.py`, `robot/wifi_client.py`), the command manager, the
safety supervisor, the dashboard, the console and both simulators.

## 1. Transport — decisions

| Item | Choice | Why |
|---|---|---|
| Network | Wi-Fi, IPv4, local network only (`docs/NETWORK.md`) | never exposed to the internet |
| Connection | **one persistent TCP connection** (robot = server on port 3333, laptop = client) | no new connection per command; delivery and order guaranteed, so a stop can never overtake an older move |
| Why not WebSocket | a WebSocket is a persistent TCP connection plus extra framing; on the ESP32 it needs a third-party library. Plain TCP with one JSON object per line gives the same live, two-way channel with less code | simpler, fewer dependencies |
| Message format | **JSON**, one object per line (`\n`), ASCII | easy to read on the Serial Monitor and in logs |
| Size limit | laptop → robot ≤ **200** characters per line (longer lines are discarded: `TOO_LONG`); robot → laptop ≤ 640 | bounded memory on the ESP32 |
| JSON subset | flat object only; values: string (no `\` escapes, ≤ 32 chars), integer (≤ 9 digits), decimal (no exponent), `true`, `false`, `null`; no nested objects/arrays; no duplicate keys; ≤ 12 fields | a small strict parser that is fully tested; anything else is rejected (`BAD_JSON`) |
| Latency | `TCP_NODELAY` on both sides | small messages leave immediately |
| Connections | one laptop at a time; the **newest connection wins** (`replaced` event to the old one) | an old socket can look alive for minutes after a laptop crash |

## 2. Common fields (every laptop → robot message)

| Field | Type | Rule |
|---|---|---|
| `version` | integer | must be **3** (`BAD_VERSION` otherwise) |
| `type` | string | one of the types in §3 (`UNKNOWN_TYPE`) |
| `command_id` | string | 1–24 characters `A-Z a-z 0-9 _ - .`; echoed in the acknowledgement |
| `seq` | integer | 1–999 999 999, **strictly increasing on each connection** (restarts at 1 after reconnecting). Not larger than the previous one → `BAD_SEQ` (duplicate or stale command) |

Unknown extra fields are rejected (`UNKNOWN_FIELD`) so typing mistakes are noticed.

## 3. Laptop → robot messages

| `type` | Extra fields | Effect |
|---|---|---|
| `ping` | – | heartbeat (feeds the watchdog) |
| `status_request` | – | robot sends a `status` message immediately |
| `motor` | `mode`:`"manual"`, `left`, `right`: integer −100…100 (% of full PWM, + = forward), `duration_ms`: 50…2000 | manual drive; stops by itself after `duration_ms`. Refused while following (`FOLLOW_ACTIVE`), in emergency (`EMERGENCY_ACTIVE`), or going forward with an obstacle (`OBSTACLE_AHEAD`) |
| `stop` | – | both motors 0, follow mode off |
| `follow_mode` | `enabled`: bool, `target_id`: integer 0…65535 (required when enabled) | enter/leave follow mode for that target (one short beep when entered) |
| `follow` | `target_id`, `left`, `right`, `duration_ms` (50…2000), `confidence`: number 0…1 | following drive request. Refused without follow mode (`FOLLOW_NOT_ENABLED`), for another target (`TARGET_MISMATCH`), in emergency, with **any** obstacle (`OBSTACLE_AHEAD` — stop-on-obstacle), or with `confidence` < 0.5 (`LOW_CONFIDENCE`, motors stopped) |
| `led` | `state`: `"on"`/`"off"`/`"blink"`, optional `interval_ms` 100…2000 (default 500) | red LED |
| `buzzer` | `pattern`: `"off"`/`"short_beep"`/`"warning"`/`"emergency"`/`"on"` | buzzer pattern (§5); `on` = continuous, stops by itself after 60 s |
| `mute` | – | buzzer off (an emergency stays latched, LED stays on) |
| `emergency_stop` | `reason`: 1–24 chars `A-Z a-z 0-9 _ - .` | **motors 0 first**, then red LED on + emergency buzzer; follow mode off; EMERGENCY latched |
| `clear_emergency` | `operator_confirmed`: bool | only `true` clears (`NEEDS_CONFIRM` otherwise). Motors stay stopped, follow mode stays off: movement needs new commands |
| `obstacle` | `source`: `"camera"`, `distance_cm`: 0…999 (0 = unknown) | camera saw something in the path: stop forward motion, refuse forward/following for 1.5 s (repeat while seen) |
| `obstacle_clear` | – | the camera no longer sees it |
| `command` | `command`: one of `FORWARD BACKWARD LEFT RIGHT STOP LED_ON LED_OFF LED_BLINK BUZZER_ON BUZZER_OFF BUZZER_SHORT_BEEP BUZZER_WARNING STATUS EMERGENCY_STOP` or the AI events `MODEL_STARTED MODEL_STOPPED FALL_DETECTED FALL_CLEARED` (results `MODEL_RUNNING`, `MODEL_STOPPED`, `FALL_ALERT_ACTIVE`, `FALL_ALERT_CLEARED`/`NO_FALL_ALERT`; LED/buzzer priority in docs/SAFETY.md); optional `duration_ms` 50…2000 for the four movement names only | **named commands** (readable alias layer, stage-1 test console). Movement names are **simulated**: recorded in `status.simulated_movement`, ack `result:"SIMULATED_MOVEMENT_RECEIVED: <name>"`, never sent to the motor outputs; end after `duration_ms` (default 2000), STOP, emergency, watchdog or disconnect. Other names behave like the typed messages. Unknown name → `UNKNOWN_COMMAND`. The ack carries `command` and `result` (docs/STAGE1_COMM_TEST.md §3) |

Example:

```json
{"version":3,"type":"motor","command_id":"m-17","seq":17,"mode":"manual","left":30,"right":30,"duration_ms":300}
```

## 4. Robot → laptop messages

**Acknowledgement** — exactly one per received line:

```json
{"version":3,"type":"ack","command_id":"m-17","seq":17,"ok":true,"left":30,"right":30}
{"version":3,"type":"ack","command_id":"m-18","seq":18,"ok":false,"error":"OBSTACLE_AHEAD"}
{"version":3,"type":"ack","command_id":null,"seq":0,"ok":false,"error":"BAD_JSON"}
```

`left`/`right` (after clamping) appear for `motor` and `follow`. Errors about one field add
`"field":"<name>"`. An `ok:true` ack means the robot **accepted** the command; whether the
motors are stopped is reported by `status` (`motors_stopped`), which reflects what the firmware
commanded to the driver — it cannot sense wheel rotation.

**Status** — every 1 s while connected, and right after `status_request`:

```json
{"version":3,"type":"status","state":"idle","motors_stopped":true,"left":0,"right":0,
 "motors_enabled":false,"follow_mode":false,"target_id":null,"emergency_active":false,
 "emergency_reason":null,"obstacle_detected":false,"obstacle_source":"none",
 "sensor_installed":false,"distance_cm":null,"watchdog_ok":true,"watchdog_trips":0,
 "led":"off","buzzer":"off","simulated_movement":"none","model_running":false,"fall_alert":false,
 "last_command_id":"p-41","uptime_s":12}
```

(one line on the wire). `state`: `emergency` > `moving` > `following` > `idle`.
`obstacle_source`: `none`, `camera` (laptop report), `sensor` (robot's own sensor), `both`.
`led` is the pattern the LED actually shows: `on` (also: model running), `off`, `blink` (led command),
`blink_fall` (fall alert), `blink_obstacle`
(100 ms, obstacle active) or `blink_following` (500 ms, follow mode); during an emergency it is
`on`. `buzzer` is the pattern being played (`fall_alert` during a fall alert; `off` once a one-shot pattern has finished or the
emergency buzzer was muted / timed out). `distance_cm` is `null` unless a sensor is installed. `watchdog_ok` is `false` after a
communication fault until the laptop sends a fresh control command (`stop`, `motor`,
`follow_mode` or `follow`).

**Events**:

| `event` | Extra fields | When |
|---|---|---|
| `hello` | `protocol`, `state`, `motors_enabled`, `sensor_installed`, `led_installed`, `buzzer_installed`, `comm_fault`, `watchdog_trips` | right after the laptop connects; `comm_fault:true` reports that the previous link failed while the robot was (simulated) moving |
| `move_done` | `simulated:true` for a named movement | a movement ran its full duration |
| `watchdog_stop` | `simulated:true` for a named movement | no valid message for 500 ms while moving → motors stopped / simulated movement cleared |
| `obstacle` | `source`:`"sensor"`, `distance_cm` | the robot's own sensor stopped forward motion |
| `obstacle_clear` | – | no obstacle any more |
| `buzzer_timeout` | – | the emergency buzzer stopped after 60 s (LED stays on) |
| `replaced` | – | a newer laptop connection took over |

## 5. LED and buzzer timing (exact)

| Pattern | Timing |
|---|---|
| LED `blink` | on for `interval_ms`, off for `interval_ms`, phase = robot uptime |
| buzzer `short_beep` | on 150 ms, then off |
| buzzer `warning` | 3 × (150 ms on, 150 ms off), then off |
| buzzer `emergency` | 300 ms on / 200 ms off, repeating, **stops after 60 s** (`EMERGENCY_BUZZ_MAX_MS`) |
| emergency latched | red LED solid on, buzzer `emergency` until `mute`, timeout or `clear_emergency` |
| following (LED not set) | LED blinks 500 ms |
| obstacle active | LED blinks 100 ms |

Indicator priority: emergency > obstacle > `led` command > following > off.

## 6. Errors

`BAD_JSON`, `TOO_LONG`, `BAD_VERSION`, `MISSING_FIELD`, `BAD_FIELD`, `UNKNOWN_FIELD`,
`UNKNOWN_TYPE`, `BAD_SEQ`, `EMERGENCY_ACTIVE`, `OBSTACLE_AHEAD`, `FOLLOW_ACTIVE`,
`FOLLOW_NOT_ENABLED`, `TARGET_MISMATCH`, `LOW_CONFIDENCE`, `NEEDS_CONFIRM`, `UNKNOWN_COMMAND`,
`FALL_ALERT_ACTIVE` (movement refused while a fall alert is active).
Checks run in this order: JSON → version → command_id → seq → type → fields → robot state.
Rejected messages never change the motors (except `LOW_CONFIDENCE`, which stops them) and do
not feed the watchdog.

## 7. Safety rules enforced by the robot itself

1. **Boot**: motor pins LOW before Wi-Fi starts; nothing moves until a command arrives.
2. **Watchdog**: while moving, no valid message for 500 ms → motors 0, `watchdog_stop`,
   `watchdog_ok:false` until a fresh control command.
3. **Every movement expires** (≤ 2 s).
4. **Disconnection / Wi-Fi loss** → motors 0, follow mode off; reported as `comm_fault` in the
   next `hello`. A reconnection never resumes movement.
5. **Emergency** overrides everything, is accepted in any state, and only
   `clear_emergency` with `operator_confirmed:true` (or a power cycle) clears it.
6. **Obstacle**: forward manual motion and all following motion are refused while active.
7. **Speed cap** `MAX_SPEED_PCT` per wheel; plus laptop caps.
8. **Phase 2 safe mode**: `MOTORS_ENABLED 0` never drives the motor pins.

## 8. Laptop-side rules

Described in `docs/SAFETY.md` (safety supervisor) and `docs/COMMANDS.md` (what the AI sends when).
