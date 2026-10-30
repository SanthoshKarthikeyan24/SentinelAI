# Stage 1 – ESP32 Wi-Fi communication test (no motors)

Goal: prove that the laptop and the ESP32 DevKit V1 talk reliably over Wi-Fi **before** any
motor is connected. Movement commands are **simulated**: the ESP32 records and acknowledges
them and never touches a motor pin. The stage-1 firmware contains no motor code at all.

Contents: 1 audit · 2 Wi-Fi arrangement · 3 protocol (incl. AI events, LED/buzzer priority) ·
4 files · 5 install · 6 configure · 7 upload (Arduino IDE) · 8 run the tests (manual sequence
with exact Serial output, checklist) · 9 temporary wiring · 10 power ·
11 troubleshooting · 12 what has been tested · 13 safety now and later · 14 integration ·
15 AI/ML status and remaining tasks · 16 step-by-step summary

---

## 1. Audit of the project (what already exists)

Checked in the project folder on your laptop (`E:\Sentinal AI`) and in the build environment.

| Area | Exists | Where | Tested? |
|---|---|---|---|
| Python app (camera → pose → tracking → posture → fall → alerts → dashboard) | yes | `src/`, `run_app.py`, `start_sentinel.bat` | with recorded dataset videos; ran once on your laptop (webcam index 0 showed a static logo – probably a virtual camera) |
| Python environment | `.venv` (Python 3.13) | created by `start_sentinel.bat` | yes, on your laptop |
| IP Webcam input | yes (`camera.source: http://PHONE_IP:8080/video`) | `src/camera/stream.py` | code tested with files and broken URLs; **your phone stream not yet tested from Python** |
| ESP32 firmware | yes, full robot firmware (protocol v3) | `firmware/sentinel_robot/` | compiles (18 variants); logic tested on the PC; **never run on your board** |
| ESP32 communication-test firmware | **new in this stage** | `firmware/esp32_test/` | compiles (12 variants); same core as above |
| Laptop ↔ ESP32 client | yes | `robot/wifi_client.py`, `robot/commands.py`, `robot/command_manager.py` | against simulators |
| Simple client API + test console | **new** | `robot/esp32_client.py`, `robot/test_console.py` | against simulators |
| Safety supervisor / state machine | yes | `src/safety/` | unit + simulation tests |
| Arduino IDE vs PlatformIO | Arduino IDE 2 (an `.arduinoIDE` folder exists in your user folder; no `platformio.ini` in the project) | – | – |
| Wi-Fi secrets file | `secrets.example.h` exists; **your `secrets.h` does not exist yet** | `firmware/*/` | – |

Missing (needs you): the ESP32 has not been flashed yet, your Wi-Fi name/password, the ESP32's
IP address, and the phone's IP Webcam address.

## 2. Wi-Fi arrangement

The laptop must reach **two** devices at once: the phone (video) and the ESP32 (commands).
A laptop joins one Wi-Fi network at a time, so all three must be on the **same** network.
The ESP32 only works on **2.4 GHz** Wi-Fi.

| Option | How | Use it when | Notes |
|---|---|---|---|
| **A – home/hostel router (recommended)** | phone, laptop and ESP32 all join the router's 2.4 GHz Wi-Fi | you have a router you control | best video bandwidth; college/guest networks often block device-to-device traffic ("client isolation") – then use B |
| **B – phone hotspot** | phone: hotspot ON *and* IP Webcam running; laptop and ESP32 join the hotspot | no router | the phone is router and camera at once; set the hotspot to 2.4 GHz / "maximise compatibility" |
| B2 – laptop hotspot | Windows *Settings → Network → Mobile hotspot*, band **2.4 GHz**; phone and ESP32 join it | no router, phone hotspot blocks clients | laptop IP is then usually 192.168.137.1 |
| C – ESP32 access point (last resort) | ESP32 makes Wi-Fi `SentinelRobot` (IP 192.168.4.1); laptop **and phone** join it | nothing else works | limited bandwidth for video; set `SENTINEL_WIFI_MODE` to `SENTINEL_MODE_ACCESS_POINT` |

Decision: **station mode** (the ESP32 joins your network – option A or B). It keeps the camera
stream working and keeps the ESP32 off the internet: it only listens on your local network,
port 3333. Never forward port 3333 on a router.

### Check your network (Windows PowerShell)

```powershell
netsh wlan show interfaces        # "SSID" = network name; "Channel" 1-13 = 2.4 GHz, 36+ = 5 GHz
ipconfig                          # "IPv4 Address" of the Wi-Fi adapter = laptop IP, e.g. 192.168.1.23
```

* **Phone IP:** open IP Webcam → *Start server*; the screen shows e.g. `http://192.168.1.31:8080`.
  Test it in the laptop browser: `http://192.168.1.31:8080` must show the IP Webcam page.
  The video URL for SentinelAI is `http://192.168.1.31:8080/video`.
* **ESP32 IP:** printed on the Arduino Serial Monitor after upload (section 7), e.g.
  `[WiFi] ESP32 IP: 192.168.1.50   (signal -55 dBm)`.
* **Same network?** The first three numbers usually match (192.168.1.x). Then:

```powershell
ping 192.168.1.31                                    # phone - expect "Reply from ..."
ping 192.168.1.50                                    # ESP32 - expect "Reply from ..."
Test-NetConnection 192.168.1.50 -Port 3333           # expect "TcpTestSucceeded : True"
.venv\Scripts\python -m robot.network_check --camera http://192.168.1.31:8080/video --robot 192.168.1.50 --seconds 15
```

The last command reads the video **and** talks to the ESP32 at the same time and prints
`camera and robot both reachable`.

## 3. Protocol

**Choice: one persistent TCP connection carrying one JSON object per line** (port 3333).
You preferred WebSocket. A WebSocket is a TCP connection plus extra framing; it would need an
extra library on the ESP32 and on Windows. Plain TCP with JSON lines gives the same things you
asked for – a persistent two-way connection, JSON messages, acknowledgements, status pushes –
using only built-in libraries on both sides (Arduino `WiFiServer`, Python `socket`). It is
already implemented and tested, so the robot firmware and this test share one protocol.

| Item | Value |
|---|---|
| Address | the ESP32's IP (Serial Monitor), or `sentinel-robot.local` where mDNS works |
| Port | 3333 (`TCP_PORT` in `config.h` = `robot.port` in `config/settings.yaml`) |
| Connection | the laptop connects; one laptop at a time (a newer connection replaces the older one) |
| Format | one JSON object per line, ASCII, ≤ 200 characters to the ESP32 (longer → `TOO_LONG`), ≤ 640 back |
| Version | `"version": 3` (one protocol for test and robot firmware; anything else → `BAD_VERSION`) |
| Heartbeat | the laptop sends `ping` every 0.15 s; the ESP32 watchdog is 500 ms |
| Reconnection | the laptop reconnects by itself (1 s, then up to 3–5 s back-off); **no movement is resumed** |

**Command (laptop → ESP32):**

```json
{"version":3,"type":"command","command_id":"L1-12","seq":12,"command":"LEFT"}
```

* `command_id`: unique per command (the client makes `L<connection>-<seq>`), 1–24 characters
  `A-Z a-z 0-9 _ - .`.
* `seq`: 1, 2, 3 … on each connection; a repeated or older number is refused (`BAD_SEQ`).
* `command`: one of `FORWARD BACKWARD LEFT RIGHT STOP LED_ON LED_OFF LED_BLINK BUZZER_ON
  BUZZER_OFF BUZZER_SHORT_BEEP BUZZER_WARNING STATUS EMERGENCY_STOP` or one of the four **AI
  events** `MODEL_STARTED MODEL_STOPPED FALL_DETECTED FALL_CLEARED` (exact upper case). The events
  use the same `"type":"command"` message – no new message type, so the protocol stays v3.
* Movement names may carry `"duration_ms"` (50–2000, default 2000): a simulated movement ends
  by itself after that time.

**Acknowledgement (ESP32 → laptop), exactly one per line received:**

```json
{"version":3,"type":"ack","command_id":"L1-12","seq":12,"ok":true,"command":"LEFT","result":"SIMULATED_MOVEMENT_RECEIVED: LEFT"}
{"version":3,"type":"ack","command_id":"L1-13","seq":13,"ok":false,"error":"EMERGENCY_ACTIVE"}
{"version":3,"type":"ack","command_id":null,"seq":0,"ok":false,"error":"BAD_JSON"}
```

| Command | `result` | Effect in stage 1 |
|---|---|---|
| FORWARD / BACKWARD / LEFT / RIGHT | `SIMULATED_MOVEMENT_RECEIVED: <name>` | recorded as `simulated_movement`; ends after 2 s, on STOP, emergency, watchdog or disconnect. **No motor output.** Refused during an emergency; FORWARD refused with an obstacle report |
| STOP | `STOPPED` | clears the simulated movement; also the "fresh control state" after a fault |
| LED_ON / LED_OFF / LED_BLINK | same name | red LED on / off / 500 ms blink |
| BUZZER_ON | `BUZZER_ON` | continuous, **stops by itself after 60 s** |
| BUZZER_OFF | `BUZZER_OFF` | silence (also mutes an emergency buzzer) |
| BUZZER_SHORT_BEEP / BUZZER_WARNING | same name | 150 ms beep / 3 short beeps |
| STATUS | `STATUS_SENT` | a `status` message follows immediately |
| EMERGENCY_STOP | `EMERGENCY_LATCHED` | simulated movement cleared, LED on, emergency buzzer (≤ 60 s), latched |
| MODEL_STARTED | `MODEL_RUNNING` | the AI model runs: red LED **continuously ON** (never blinks in normal operation); ends an LED test |
| FALL_DETECTED | `FALL_ALERT_ACTIVE` | fall alert: LED blinks (250 ms), buzzer "beep-beep" pattern (stops by itself after 60 s, the alert stays); simulated movement cleared; movement refused with `FALL_ALERT_ACTIVE`. **Sent again while active → nothing restarts** |
| FALL_CLEARED | `FALL_ALERT_CLEARED` (or `NO_FALL_ALERT`) | buzzer stops; LED back to ON if the model still runs, else off |
| MODEL_STOPPED | `MODEL_STOPPED` | LED off, buzzer off, any fall alert ended |

**LED / buzzer priority (highest first):** emergency stop (LED solid, alarm) > fall alert (LED
blinks, beep-beep) > LED/buzzer test commands > model running (LED solid) > off. A test command
(6–12) therefore never hides a fall alert; `LED_OFF` ends a test and the LED returns to its
automatic state (ON while the model runs). `BUZZER_OFF` during a fall alert mutes the buzzer –
the LED keeps blinking until `FALL_CLEARED`.

Leaving the emergency needs an explicit operator action – a separate message the console sends
only after you type `YES`:

```json
{"version":3,"type":"clear_emergency","command_id":"L1-20","seq":20,"operator_confirmed":true}
```

**Status (every second, and after STATUS)** – only values the firmware knows:

```json
{"version":3,"type":"status","state":"idle","motors_stopped":true,"left":0,"right":0,"motors_enabled":false,
 "follow_mode":false,"target_id":null,"emergency_active":false,"emergency_reason":null,"obstacle_detected":false,
 "obstacle_source":"none","sensor_installed":false,"distance_cm":null,"watchdog_ok":true,"watchdog_trips":0,
 "led":"on","buzzer":"off","simulated_movement":"left","model_running":true,"fall_alert":false,
 "last_command_id":"L1-12","uptime_s":42}
```

`led`/`buzzer` are what the firmware drives on the pins; the firmware cannot see whether an LED
is really wired (`hello` says `led_installed` / `buzzer_installed` from `config.h`).
`motors_enabled` is always `false` in the test firmware. `led` is `off on blink blink_fall`
(and `blink_obstacle` / `blink_following` in the robot firmware), `buzzer` is `off on short_beep
warning emergency fall_alert`.

**Events:** `hello` (on connect: protocol, state, motors_enabled, led/buzzer installed,
`comm_fault`), `move_done` / `watchdog_stop` (with `"simulated":true`), `buzzer_timeout`, `replaced`.

**Error codes:** `BAD_JSON`, `TOO_LONG`, `BAD_VERSION`, `MISSING_FIELD`, `BAD_FIELD` (+ `field`),
`UNKNOWN_FIELD`, `UNKNOWN_TYPE`, `UNKNOWN_COMMAND`, `BAD_SEQ`, `EMERGENCY_ACTIVE`,
`FALL_ALERT_ACTIVE`, `OBSTACLE_AHEAD`, `FOLLOW_ACTIVE`, `NEEDS_CONFIRM`. Full specification: `docs/PROTOCOL.md`.

## 4. Files

| File | What it is |
|---|---|
| `firmware/esp32_test/esp32_test.ino` | stage-1 firmware: Wi-Fi, TCP server, LED, buzzer, status LED, power-up self-test. **No motor code** |
| `firmware/esp32_test/config.h` | settings: Wi-Fi mode, pins, LED/buzzer installed, timings |
| `firmware/esp32_test/secrets.example.h` | copy to `secrets.h` and enter your Wi-Fi name/password (never published) |
| `firmware/esp32_test/robot_core.h/.cpp` | protocol + safety logic, identical to `firmware/sentinel_robot/` (`tools/sync_firmware_core.py`) |
| `robot/esp32_client.py` | `ESP32Client`: `connect() disconnect() send_command() send_simulated_movement() stop() set_led() set_buzzer() get_status() emergency_stop() reset_emergency()` and the AI events `model_started() model_stopped() fall_detected() fall_cleared() send_event(name)` |
| `robot/test_console.py` | the menu console; `--hw-test` LED + buzzer test without the AI model; `--run-all` sends everything once and prints a summary; log in `logs/` |
| `robot/robot_logic.py`, `src/app_runner.py` | the AI app: sends MODEL_STARTED / FALL_DETECTED / FALL_CLEARED / MODEL_STOPPED on state changes only |
| `robot/comm_test.py` | automated 18-step test (incl. invalid messages, watchdog, reconnect) |
| `robot/sim_robot.py` | a simulated ESP32 for practice (`--sim`) |

## 5. Install (once)

The project environment already exists. In PowerShell:

```powershell
cd "E:\Sentinal AI"
.venv\Scripts\python -m pip install -r requirements.txt     # adds psutil; everything else is already there
.venv\Scripts\python -m robot.test_console --sim             # practise with a simulated ESP32 (no hardware)
```

Expected: the menu appears with `Connection: CONNECTED`, `Motors: DISABLED`; type `19` to exit.

Arduino IDE 2 (already installed on your laptop):
1. *File → Preferences → Additional boards manager URLs*:
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
2. *Tools → Board → Boards Manager* → search **esp32** → install **esp32 by Espressif Systems**
   (version 3.x; I compiled with 3.3.12 and 2.0.17).
3. USB driver: if no COM port appears when the ESP32 is plugged in, install the
   **CP210x** (Silicon Labs) or **CH340** driver – the chip name is printed on the small
   chip next to the USB socket. Use a USB cable that carries data (some are charge-only).

## 6. Configure

1. In `E:\Sentinal AI\firmware\esp32_test\` copy `secrets.example.h` → `secrets.h` and edit:

   ```cpp
   #define WIFI_SSID "YourWiFiName"          // 2.4 GHz network from section 2
   #define WIFI_PASSWORD "YourWiFiPassword"
   #define AP_PASSWORD "change-me-123"       // only used in access-point mode
   ```

2. Check `config.h`: `LED_INSTALLED 1` / `BUZZER_INSTALLED 1` only if wired (section 9);
   `BUZZER_PASSIVE 0` for an active buzzer (see 9.3); `PIN_STATUS_LED 2` (set `-1` if your board
   has no blue LED on GPIO2).
3. After the first upload, put the ESP32 IP into `config/settings.yaml`:

   ```yaml
   robot:
     host: 192.168.1.50      # from the Serial Monitor
     port: 3333
   ```

   (or pass `--host 192.168.1.50` to every command). Keep `robot.enabled: false` until the
   checklist passes.

## 7. Upload the firmware (Arduino IDE 2)

1. **Create `secrets.h` first** (section 6). Without it the sketch still compiles (warning
   `secrets.h not found`) but the ESP32 tries to join a network called `YOUR_WIFI_NAME` and never
   connects.
2. Plug the ESP32 into the laptop with a **data** USB cable. USB powers the board, the LED and the
   buzzer – no battery is needed for this stage.
3. Arduino IDE: *File → Open…* → `E:\Sentinal AI\firmware\esp32_test\esp32_test.ino`. The tabs
   `config.h`, `robot_core.cpp`, `robot_core.h` (and `secrets.h`) open with it. Do **not** open
   `firmware\sentinel_robot\` – that is the robot firmware with motor code.
4. *Tools → Board → esp32 →* **DOIT ESP32 DEVKIT V1**.
5. *Tools → Port →* the COM port that appears when you plug the board in (unplug/replug to see
   which one is new; e.g. `COM5`). In PowerShell you can list them:
   `Get-CimInstance Win32_SerialPort | Select DeviceID, Description` (or *Device Manager → Ports
   (COM & LPT)* – "Silicon Labs CP210x" or "USB-SERIAL CH340").
6. *Tools → Upload Speed:* 921600 (if the upload fails, choose 115200).
7. Click **Verify** (✓). Expected at the bottom: `Sketch uses … bytes (…%) of program storage
   space`. The only acceptable warning is `secrets.h not found` – and only if you have not created
   `secrets.h` yet.
8. Click **Upload** (→). If it stops at `Connecting........_____`, press and hold the **BOOT**
   button until `Writing at 0x…` appears. Expected end: `Hard resetting via RTS pin...`.
9. *Tools → Serial Monitor*, set **115200 baud** and *Both NL & CR* (any line ending works),
   press the board's **EN** button. Expected:

```text
[BOOT] SentinelAI ESP32 communication test - protocol v3
[BOOT] Motors: NONE in this firmware - movement commands are simulated only
[BOOT] Safe start: LED off, buzzer off, no movement, waiting for the laptop
[BOOT] Red LED on GPIO 22
[BOOT] Buzzer (active) on GPIO 23 via transistor
[SELFTEST] Red LED blinks 3 times, buzzer beeps once
[WiFi] Connecting to 'YourWiFiName' ...
[WiFi] Connected
[WiFi] ESP32 IP: 192.168.1.50   (signal -55 dBm)
[TCP] Listening on port 3333
[WiFi] Use this IP with: python -m robot.test_console --host <IP>
```

**Write down the IP** (`192.168.1.50` is only an example). The password is never printed.
At this point the red LED is **off** (the AI model has not started) – that is correct.

Blue on-board LED: 1 Hz blink = no Wi-Fi, short flash every 2 s = waiting for the laptop,
solid = laptop connected, fast blink = emergency.

Serial Monitor and Python can both run at the same time (Serial is USB, Python is Wi-Fi). Close
the Serial Monitor before you **upload** again – the COM port can be used by one program only.

Every message from the laptop is printed in plain words (`LOG_COMMANDS 1` in `config.h`).
Heartbeat `ping` messages are not printed. For protocol debugging set `LOG_RAW_JSON 1` to also
see every raw JSON line (`[RX-JSON] …` / `[TX] …`).

## 8. Run the tests

All commands in PowerShell, in the project folder. Replace `192.168.1.50` with your ESP32's IP.

```powershell
cd "E:\Sentinal AI"
ping 192.168.1.50                                                   # expect "Reply from 192.168.1.50"
Test-NetConnection 192.168.1.50 -Port 3333                          # expect "TcpTestSucceeded : True"
.venv\Scripts\python -m robot.test_console --host 192.168.1.50 --hw-test   # LED + buzzer only (8.2)
.venv\Scripts\python -m robot.test_console --host 192.168.1.50             # interactive menu (8.1, 8.3)
.venv\Scripts\python -m robot.test_console --host 192.168.1.50 --run-all   # everything once, summary
.venv\Scripts\python -m robot.comm_test --host 192.168.1.50                # 18-step protocol test, JSON report
```

`Test-NetConnection` opens and closes a connection – the Serial Monitor shows a short
connect/disconnect; that is normal. Only one laptop program can be connected at a time: a newer
connection replaces the older one (the older one prints `replaced`).

### 8.1 Interactive console

```text
Connecting to the ESP32 at 192.168.1.50:3333 ...
[ESP32] hello: protocol 3, state idle, motors_enabled False, comm_fault False

================================
 SENTINELAI ESP32 TEST CONSOLE
================================
Connection: CONNECTED   (192.168.1.50:3333)
ESP32: RESPONDING   state idle, simulated movement none, LED off, buzzer off
Motors: DISABLED (movement is simulated)
AI model (as told to the ESP32): not running   fall alert: none
  -- SIMULATED MOVEMENT - acknowledged only, no motor moves --
 1. FORWARD
 2. BACKWARD
 3. LEFT
 4. RIGHT
 5. STOP (clears the simulated movement)
  -- PHYSICAL LED TEST (red LED, GPIO 22) --
 6. LED ON
 7. LED OFF (ends the test; LED returns to its automatic state)
 8. LED BLINK
  -- PHYSICAL BUZZER TEST (GPIO 23 via transistor) --
 9. BUZZER ON (stops by itself after 60 s)
10. BUZZER OFF (also mutes an active alarm)
11. SHORT BEEP
12. WARNING PATTERN (3 beeps)
  -- AI EVENTS - sent by YOU as deliberate tests (operate the real LED + buzzer) --
13. MODEL_STARTED  -> LED continuously ON
14. FALL_DETECTED  -> LED blinks + buzzer alert  [TEST - no real fall]
15. FALL_CLEARED   -> alert off, LED ON again if the model runs
16. MODEL_STOPPED  -> LED and buzzer OFF
  -- OTHER --
17. STATUS
18. EMERGENCY STOP (LED ON + alarm, latched until R)
19. EXIT (sends STOP, disconnects)
 H. HARDWARE TEST: LED on/blink/off + short beep + warning (no AI model needed)
 R. RESET EMERGENCY (asks for confirmation)
 T. send INVALID test messages (bad JSON, unknown command, wrong version, too long)
 M. show this menu again
```

Each choice prints what was sent and the ESP32's answer, e.g.

```text
Enter command: 14
-> FALL_DETECTED
   (deliberate TEST event sent by you - not a fall detected by the AI)
<- ACK FALL_DETECTED: OK (FALL_ALERT_ACTIVE) id=L1-9 in 14.2 ms
Enter command: 1
-> FORWARD
<- FORWARD: FAILED - refused by ESP32: FALL_ALERT_ACTIVE
```

Items 13–16 are **test events typed by you**. The console never says a fall happened; only the
AI app sends FALL_DETECTED on its own, and only after the model reports a confirmed fall.

### 8.2 LED and buzzer test without the AI model

Menu item **H**, or `--hw-test` on the command line. It sends LED_ON, LED_BLINK, LED_OFF,
BUZZER_SHORT_BEEP, BUZZER_WARNING, BUZZER_ON, BUZZER_OFF with a pause between them and tells you
what you should see or hear (`expected now: LED should be ON` …). The ESP32 can only report what
it set on its pins, so **only your eyes and ears can confirm** the LED and buzzer work.

### 8.3 Manual test sequence with the exact Serial Monitor output

Keep the Serial Monitor (115200) open next to the console. Start the console (8.1). Serial shows:

```text
[TCP] Laptop client connected (from 192.168.1.23)
```

Then type the menu numbers in this order. "Serial" is what the ESP32 prints, word for word
(checked against the real firmware code on the PC); "See / hear" is what the hardware should do.

| Step | Type | Serial Monitor (exact) | See / hear |
|---|---|---|---|
| 1 | `13` MODEL_STARTED | `[RX] Event received: MODEL_STARTED`<br>`[LED] Model running: LED continuously ON` | LED **on** and stays on |
| 2 | wait 10 s, then `17` STATUS | `[RX] Command received: STATUS`<br>`[STATUS] Status sent to the laptop` | LED still on, **no flicker**; console status `led on`, `model_running: true` |
| 3 | `13` again | `[RX] Event received: MODEL_STARTED`<br>`[LED] Model running (already): LED continuously ON` | no change |
| 4 | `3` LEFT | `[RX] Command received: LEFT`<br>`[MOVEMENT] LEFT received - SIMULATION ONLY, motors disabled` | nothing moves; LED stays on |
| 5 | wait 2 s | `[MOVEMENT] Simulated movement finished (time limit reached)` | – |
| 6 | `14` FALL_DETECTED | `[RX] Event received: FALL_DETECTED`<br>`[ALERT] Fall detected`<br>`[LED] Fall alert: blinking`<br>`[BUZZER] Fall warning active` | LED **blinks** fast (4×/s); buzzer **beep-beep … beep-beep** every 1.2 s |
| 7 | `14` again | `[RX] Event received: FALL_DETECTED`<br>`[ALERT] Fall alert already active - not restarted` | same pattern, not restarted or doubled |
| 8 | `1` FORWARD | `[RX] Command received: FORWARD`<br>`[REJECTED] FORWARD: FALL_ALERT_ACTIVE` | console: `refused by ESP32: FALL_ALERT_ACTIVE` |
| 9 | `10` BUZZER OFF | `[RX] Command received: BUZZER_OFF`<br>`[BUZZER] OFF - alarm muted, alert stays active` | buzzer silent, LED **keeps blinking** |
| 10 | `15` FALL_CLEARED | `[RX] Event received: FALL_CLEARED`<br>`[ALERT] Fall alert cleared`<br>`[LED] Model running: LED continuously ON`<br>`[BUZZER] Alert stopped` | LED **on** again (solid), buzzer silent |
| 11 | `8` LED BLINK | `[RX] Command received: LED_BLINK`<br>`[LED] Test command: BLINK` | LED blinks slowly (0.5 s) |
| 12 | `7` LED OFF | `[RX] Command received: LED_OFF`<br>`[LED] Test command OFF: LED back to automatic state (ON - model running)` | LED **on** (model still running) |
| 13 | `16` MODEL_STOPPED | `[RX] Event received: MODEL_STOPPED`<br>`[LED] Model stopped: LED OFF`<br>`[BUZZER] OFF` | LED **off**, buzzer silent |
| 14 | `6` LED ON | `[RX] Command received: LED_ON`<br>`[LED] Test command: ON` | LED on |
| 15 | `7` LED OFF | `[RX] Command received: LED_OFF`<br>`[LED] Test command OFF: LED back to automatic state (off)` | LED off |
| 16 | `11` SHORT BEEP | `[RX] Command received: BUZZER_SHORT_BEEP`<br>`[BUZZER] Test command: SHORT_BEEP` | one short beep |
| 17 | `12` WARNING | `[RX] Command received: BUZZER_WARNING`<br>`[BUZZER] Test command: WARNING` | three short beeps |
| 18 | `9` BUZZER ON | `[RX] Command received: BUZZER_ON`<br>`[BUZZER] Test command: ON (stops by itself after 60 s)` | continuous tone |
| 19 | `10` BUZZER OFF | `[RX] Command received: BUZZER_OFF`<br>`[BUZZER] OFF` | silent |
| 20 | `18` EMERGENCY STOP | `[RX] Command received: EMERGENCY_STOP`<br>`[EMERGENCY] Emergency stop latched - LED ON, alarm buzzer. Reset needs operator confirmation` | LED solid on, alarm sound (max 60 s) |
| 21 | `3` LEFT | `[RX] Command received: LEFT`<br>`[REJECTED] LEFT: EMERGENCY_ACTIVE` | refused |
| 22 | `R`, then type `YES` | `[RX] Command received: clear_emergency`<br>`[EMERGENCY] Cleared by the operator - all movement stays stopped`<br>`[LED] LED OFF` | LED off, silent; nothing resumes |
| 23 | `T` invalid messages | `[RX] Malformed JSON - rejected (BAD_JSON)`<br>`[RX] Command received: JUMP`<br>`[REJECTED] JUMP: UNKNOWN_COMMAND`<br>`[RX] Command received: STOP`<br>`[REJECTED] STOP: BAD_VERSION`<br>`[RX] Command received: command`<br>`[REJECTED] command: MISSING_FIELD (field 'command')`<br>`[RX] Command received: STOP`<br>`[REJECTED] STOP: BAD_FIELD (field 'command_id')`<br>`[RX] Command received: LEFT`<br>`[REJECTED] LEFT: BAD_FIELD (field 'duration_ms')`<br>`[RX] Message longer than 200 characters - rejected (TOO_LONG)`<br>`[RX] Command received: STATUS` / `[STATUS] Status sent to the laptop` | console: `link still usable afterwards: yes` |
| 24 | `13`, `14`, then close the console window (X) | `[TCP] Laptop client disconnected (…) - simulated movement cleared, model LED off` | LED keeps **blinking** (fall alert stays – it was not cleared); after 60 s the buzzer stops with `[BUZZER] Stopped after the 60 s limit (the alert itself stays active)` |
| 25 | start the console again, `17` | `[TCP] Laptop client connected (from …)` | status: `model_running: false`, `fall_alert: true`, `simulated_movement: none` – nothing resumed |
| 26 | `15`, then `19` EXIT | `[ALERT] Fall alert cleared` / `[LED] Model not running: LED OFF` / `[BUZZER] Alert stopped`, then `[RX] Command received: STOP`, `[MOVEMENT] STOP received - simulated movement cleared (no motors)`, `[TCP] Laptop client disconnected (closed) - …` | LED off, silent |

### 8.4 Automatic runs

```powershell
.venv\Scripts\python -m robot.test_console --host 192.168.1.50 --run-all
.venv\Scripts\python -m robot.comm_test --host 192.168.1.50
```

Expected last lines: `RESULT: ALL AS EXPECTED` and `RESULT: ALL PASSED`. `--run-all` also sends
the four AI events and checks that a fall alert refuses movement (the LED blinks and the buzzer
sounds briefly during that part). Results go to `logs\esp32_test_<date>_<time>.log` and
`evaluation\communication\comm_test_<date>_<time>.json` – **send me both files**.

### 8.5 Checklist

"Auto" = covered by the automated tests on the PC (already passing against the real firmware
core in a simulator) **and** by `--run-all` / `comm_test` on your board. "You" = needs your
hands or eyes on the real ESP32. Nothing in the "Observed on your ESP32" column is filled in
until you report it.

| # | Test | How | Expected | Type | Observed on your ESP32 |
|---|---|---|---|---|---|
| 1 | Normal connection | start the console | `Connection: CONNECTED`, `ESP32: RESPONDING`, `Motors: DISABLED`; Serial `[TCP] Laptop client connected` | Auto + You | |
| 2 | MODEL_STARTED → LED on | 13 | LED solid on | Auto + **You (look)** | |
| 3 | LED stays on | wait 30 s, 17 | no blinking, `led: on` | Auto + **You** | |
| 4 | FALL_DETECTED → blink + buzzer | 14, 14 | blinking + beep-beep; second 14 does not restart | Auto + **You (look, listen)** | |
| 5 | FALL_CLEARED → LED back on | 15 | buzzer stops, LED solid on | Auto + **You** | |
| 6 | MODEL_STOPPED → all off | 16 | LED off, buzzer off | Auto + **You** | |
| 7 | Simulated movement | 1, 2, 3, 4 then 5 | each `SIMULATED_MOVEMENT_RECEIVED: …`; `[MOVEMENT] … SIMULATION ONLY`; status `motors_stopped: true`, `motors_enabled: false` | Auto | |
| 8 | LED / buzzer test without model | H (or `--hw-test`) | LED on / blink / off; beep, 3 beeps, tone, silence | Auto + **You** | |
| 9 | Invalid / malformed messages | T | `BAD_JSON`, `UNKNOWN_COMMAND`, `BAD_VERSION`, `TOO_LONG` … refused and printed; link still usable | Auto | |
| 10 | Repeated commands | press 6 twenty times quickly; `comm_test` sends 50 pings | every one acknowledged, no timeouts | Auto | |
| 11 | Disconnect / reconnect | step 24–25 of 8.3; also switch the hotspot/router off 10 s | `CONNECTION LOST … reconnecting`, then `CONNECTED`; **nothing resumes**; model LED off until MODEL_STARTED again | Auto + **You** | |
| 12 | ESP32 reboot | press **EN** while the console runs | link lost, reconnects; self-test blink/beep; LED off; status `simulated_movement none` | **You** | |
| 13 | Emergency | 18, then 3, then R + `YES` | `EMERGENCY_LATCHED`; LED on + alarm; LEFT refused `EMERGENCY_ACTIVE`; reset → nothing resumes | Auto + **You** | |
| 14 | Safe shutdown | 19 (EXIT) | `Sending STOP and disconnecting …`; Serial `[RX] Command received: STOP` then `[TCP] Laptop client disconnected (closed)` | Auto + You | |
| 15 | Silent laptop (watchdog) | `comm_test` step 16 | `[SAFETY] Laptop silent for 0.5 s - simulated movement cleared (watchdog)` | Auto | |
| 16 | Camera + ESP32 together | `robot.network_check` (section 2) | `camera and robot both reachable` | **You** | |
| 17 | AI app → ESP32 | section 14: start the app with the robot enabled | Serial `MODEL_STARTED` once (not per frame); LED on; stopping the app → `MODEL_STOPPED`, LED off | **You** | |

## 9. Temporary wiring (breadboard, no motors)

Parts: ESP32 DevKit V1, a breadboard, jumper wires, **red LED + 330 Ω resistor**, **buzzer +
NPN transistor (2N2222, BC547 or S8050) + 1 kΩ resistor**, optional 1N4148 diode.
Do all wiring **with the USB cable unplugged**.

### 9.1 ESP32 on the breadboard

1. Push the ESP32 across the centre gap of the breadboard, so each row of pins has free holes on
   its outer side. (A wide DevKit leaves only one free row on one side – that is fine, or use two
   breadboards side by side.)
2. Connect an ESP32 **GND** pin to the breadboard's **blue (−) rail**. Every GND below goes to
   this rail.
3. Do not use the 3V3 or VIN pins for anything except what is listed below.

### 9.2 Red LED (GPIO 22)

| Step | Connect |
|---|---|
| 1 | ESP32 **D22** (GPIO 22) → one leg of the **330 Ω resistor** |
| 2 | other resistor leg → LED **long leg** (anode, +) |
| 3 | LED **short leg** (cathode, flat edge) → **GND rail** |

Current ≈ (3.3 V − 2.0 V) / 330 Ω ≈ 4 mA – safe for the pin. Check: at power-up the self-test
blinks it 3 times. No blink → LED reversed, or wrong pin.

**On-board LED:** most DOIT DevKit V1 boards have a blue LED on GPIO 2 (used as link light).
To check yours: Arduino IDE → *File → Examples → 01.Basics → Blink*, change `LED_BUILTIN` to `2`,
upload; the blue LED should blink. If not, set `PIN_STATUS_LED -1` in `config.h`.

### 9.3 Buzzer (GPIO 23, through a transistor)

First find out what you have:
* **Active buzzer**: usually has a sealed bottom and a sticker; touching its legs to 3.3 V/GND
  (or a 5 V USB supply, + to +) makes a continuous tone → `BUZZER_PASSIVE 0`.
* **Passive buzzer**: open bottom with a visible circuit board; only clicks on plain DC →
  `BUZZER_PASSIVE 1` (the firmware then makes a 2 kHz tone).
* A 5 V buzzer typically takes 20–30 mA – more than an ESP32 pin should give (≈ 12 mA), so the
  pin only switches a transistor.

Transistor pins (plastic TO-92 package, **flat face toward you, legs pointing down**, left → right):

| Part | left | middle | right |
|---|---|---|---|
| PN2222A / most "2N2222" TO-92 | E | B | C |
| S8050 | E | B | C |
| BC547 / BC548 | **C** | B | **E** |

Clones differ – if your marking is not in the table, look up its datasheet, or use a multimeter's
hFE socket. B (base) is the middle pin for all of these.

| Step | Connect |
|---|---|
| 1 | ESP32 **D23** (GPIO 23) → **1 kΩ resistor** → transistor **B** (middle) |
| 2 | transistor **E** → **GND rail** |
| 3 | transistor **C** → buzzer **−** (short leg / black) |
| 4 | buzzer **+** (long leg / red / marked +) → ESP32 **VIN** pin (≈ 5 V from USB) for a 5 V buzzer, or **3V3** for a 3 V buzzer |
| 5 | (passive/magnetic buzzer: required; active: optional) 1N4148 diode across the buzzer, **stripe toward +** |

Check: the self-test beeps once at power-up; `BUZZER_SHORT_BEEP` beeps once.
Note: the ESP32 **VIN** pin carries ≈ 5 V only while the board is powered from USB.

### 9.4 Final check before plugging in

* Nothing is connected to GPIO 0, 2 (except the on-board LED), 5, 12 or 15.
* No wire from 5 V/VIN goes to any GPIO.
* The L298N and motors are **not** connected.

```text
            ESP32 DevKit V1
   D22 ──[330 Ω]──►|── GND          (red LED, long leg toward the resistor)
   D23 ──[1 kΩ]── B  (NPN)
                  E ─────────── GND
                  C ── buzzer(−)   buzzer(+) ── VIN (5 V) or 3V3
   GND ─────────── breadboard − rail
```

## 10. Power during stage 1

* **Uploading / debugging:** laptop USB.
* **Wireless operation (no laptop cable):** a **USB power bank or a phone charger** into the
  ESP32's USB socket. Simplest and safe; the Wi-Fi test does not need a battery pack.
* Your **18650 Li-ion cells (3.7 V, with BMS)** are for the robot stage. Do **not** connect one cell
  directly to the ESP32: 3V3 accepts at most 3.6 V (a full cell is 4.2 V) and VIN needs about
  5 V. A single cell would need a 5 V boost module – not needed now.
* Robot plan (later, after this stage passes): the right pack depends on the motors and on your
  BMS board (see `docs/HARDWARE.md` §4). Nothing is wired to the cells in stage 1.

## 11. Troubleshooting

**Arduino IDE / upload**

| Symptom | Cause | Fix |
|---|---|---|
| No COM port in *Tools → Port* | charge-only cable / missing driver | other cable; install the CP210x or CH340 driver (name on the chip next to the USB socket) |
| `Failed to connect to ESP32: … Wrong boot mode` / stops at `Connecting....` | board not in download mode | hold **BOOT** while uploading starts |
| `could not open port 'COM5': Access is denied` | Serial Monitor or another program has the port | close the Serial Monitor / other IDE window, upload again |
| `#warning "secrets.h not found"` | file not created, or named `secrets.h.txt` | copy `secrets.example.h` → `secrets.h` in `firmware\esp32_test\`; in Explorer enable *View → File name extensions* |
| Serial shows garbage | wrong baud | 115200 |
| Nothing at all in Serial Monitor | board booted before the monitor opened | press **EN** |

**Wi-Fi**

| Symptom | Cause | Fix |
|---|---|---|
| `[WiFi] Still not connected - retrying …` | wrong name/password, 5 GHz network, too far | check spelling (case matters) in `secrets.h`; use 2.4 GHz ("maximise compatibility" on a phone hotspot); move closer; upload again after editing |
| `[WiFi] Disconnected - …` repeatedly | weak signal / power dip | closer to the router; better USB cable/port |
| Console: `Could not connect: … timed out` | wrong IP, laptop on another network, client isolation (college/guest Wi-Fi) | compare with `[WiFi] ESP32 IP:`; `ping <IP>`; section 2; use a phone/laptop hotspot |
| `Connection refused` | IP correct but firmware not listening / wrong port | press EN; port 3333 in both places |
| `ping` works, `Test-NetConnection … -Port 3333` fails | firmware not running the TCP server (board rebooting / old firmware) | look for `[TCP] Listening on port 3333` in Serial; press EN |
| Console: `CONNECTION LOST … reconnecting` while you do nothing | Wi-Fi drop or ESP32 reset (look for `[BOOT]` in Serial) | expected to recover by itself; nothing resumes – send MODEL_STARTED again if needed |

**Red LED (GPIO 22)**

| Symptom | Cause | Fix |
|---|---|---|
| No self-test blink at power-up | LED reversed, wrong breadboard row, wrong pin, `LED_INSTALLED 0` | long leg toward the resistor, short leg to GND; wire on **D22**; check `config.h` |
| Self-test blinks but `13` does not light it | MODEL_STARTED not acknowledged, or an emergency/fall alert has priority | console must show `ACK MODEL_STARTED: OK`; check status `led`; R / 15 |
| LED "flickers" while the model runs | not a firmware state (status `led: on`) – loose wire | push the LED/resistor in firmly |
| LED very dim | wrong resistor (e.g. 330 kΩ instead of 330 Ω: orange-orange-**brown**) | use 330 Ω |
| Status says `led: on` but the LED is dark | firmware sets the pin; the wiring is the problem | measure ≈ 3.3 V between D22 and GND |

**Buzzer (GPIO 23 via transistor)**

| Symptom | Cause | Fix |
|---|---|---|
| Silent on self-test and `11` | transistor E/C swapped, base resistor not on B, buzzer + not on VIN/3V3 | section 9.3 pin table; B is the middle pin |
| Only clicks | passive buzzer with `BUZZER_PASSIVE 0` | set `BUZZER_PASSIVE 1`, upload again |
| Always on, even before Wi-Fi | C–E shorted, or base wired to 3V3 instead of D23 | rewire per 9.3 |
| ESP32 resets (`[BOOT]` lines) when it beeps | supply dip | buzzer + to VIN (5 V) not 3V3; better USB port/cable |
| Fall alert: LED blinks but no sound | the buzzer was muted (`10`) or the 60 s limit passed – by design | 15 then 14 again to retest |
| `[BUZZER] Test … ignored while an alarm is active` | test commands never override an alarm | clear the alert (15 / R) first |

**Protocol**

| Symptom | Cause | Fix |
|---|---|---|
| `robot speaks protocol vX` | old firmware on the board | upload `esp32_test.ino` again |
| `refused by ESP32: UNKNOWN_COMMAND` for MODEL_STARTED etc. | the board still runs the previous upload (before the AI events) | upload this version |
| `refused by ESP32: EMERGENCY_ACTIVE` | emergency latched | R → YES |
| `refused by ESP32: FALL_ALERT_ACTIVE` | movement during a fall alert – by design | 15 (FALL_CLEARED) |
| `no acknowledgement within 1.0 s` | weak Wi-Fi / busy network | move closer; `--timeout 2` |
| Need to see the raw JSON | – | `LOG_RAW_JSON 1` in `config.h`, upload |
| Camera stops when the console connects | laptop joined a different network | section 2, same network for all |

## 12. What has actually been tested (build environment, no hardware)

| Item | Result |
|---|---|
| `esp32_test.ino` compiles, board DOIT ESP32 DEVKIT V1, ESP32 cores 3.3.12 and 2.0.17, 6 variants each (default, no secrets.h, passive buzzer, no LED/buzzer/status LED, access point, static IP) | 12/12, only the intended `secrets.h not found` warning – `reports/firmware_build_matrix_esp32_test.log` |
| Shared core unit tests (C++, on the PC) | 570 checks passed: MODEL_STARTED → LED solid and never blinking; FALL_DETECTED → blink + buzzer pattern, repeated event not restarted; FALL_CLEARED → LED back on; MODEL_STOPPED → all off; movement acknowledged but the motor outputs never written; invalid messages rejected and logged; disconnect/reconnect resumes nothing; emergency priority and reset; exact Serial log lines |
| Python simulator ≡ firmware core | byte-identical output on a scenario with every named command and AI event + 16 random scripts × 700 steps |
| `robot.test_console --run-all` | `ALL AS EXPECTED` against the C++ firmware core and the Python simulator |
| `robot.comm_test` | 18/18 against the C++ firmware core |
| AI app → robot link | state-change-only sending (one MODEL_STARTED for many frames), FALL_DETECTED/FALL_CLEARED, re-sync after reconnect, MODEL_STOPPED on close – `tests/test_esp32_test_stage.py` |
| Serial output in section 8.3 | produced by running the real `robot_core.cpp` on the PC |

**Not tested (needs your board):** upload, Wi-Fi join, real latency, the LED and buzzer actually
lighting/sounding, Wi-Fi interruption, ESP32 reboot, camera + ESP32 on the same network. Please
run section 8 and send me the log files and the Serial Monitor text.

## 13. Safety – now and when motors are added

Already active in stage 1 (same code as the robot firmware):

| Safeguard | Stage 1 behaviour |
|---|---|
| Command validation | strict JSON, version, ID, sequence, field ranges, known commands only |
| Watchdog | no valid message for 500 ms during a simulated movement → cleared, `watchdog_stop`, `watchdog_ok:false` |
| Every movement ends by itself | ≤ 2 s |
| Emergency priority | sent first by the laptop and re-sent until acknowledged; latched until operator reset |
| Stale commands | duplicate/older `seq` refused; the laptop drops motion commands older than 0.3 s |
| Safe start-up | outputs off before Wi-Fi starts; no movement until a command |
| No resume after reconnection | disconnect clears the movement; the next hello reports `comm_fault`; STOP needed |
| Bounded buzzer | ≤ 60 s |

When motors are introduced (only after this checklist passes), in this order:
1. Wire the L298N **without motors**, flash `firmware/sentinel_robot` (it has the motor code)
   with `MOTORS_ENABLED 0`, repeat the checklist.
2. Lift the wheels; set `MOTORS_ENABLED 1`; run `comm_test --allow-motion`; calibrate
   (`docs/HARDWARE.md` §6). The named movement commands stay simulated until a deliberate code
   change maps them onto the tested `motor` command – changing a setting alone never makes
   FORWARD drive a wheel.
3. Only then manual driving from the dashboard, then following.

## 14. Integration with SentinelAI

The test uses the **same** client as the main app, so nothing is thrown away:

```text
robot/test_console.py ─► robot/esp32_client.py (simple API) ─┐
dashboard / AI ─► robot/robot_logic.py (RobotLink) ─► src/safety/safety_supervisor.py
                                                    ─► robot/command_manager.py (priority queue)
                                                    └─► robot/wifi_client.py (RobotClient) ─► ESP32
```

| Your suggested file | This project |
|---|---|
| `src/communication/protocol.py` | `robot/commands.py` |
| `src/communication/esp32_client.py` | `robot/esp32_client.py` (simple API) + `robot/wifi_client.py` (connection) |
| `src/communication/connection_manager.py` | `robot/wifi_client.py` (reconnect, heartbeat) + `robot/command_manager.py` (priorities) |
| `src/safety/*` | `src/safety/safety_supervisor.py`, `src/safety/robot_state_machine.py` |
| `firmware/esp32_test/esp32_test.ino` | same |
| `configs/` | `config/settings.yaml` |

### 14.1 Run the AI app with the ESP32

`config/settings.yaml` now has `robot.enabled: true` and `robot.host: auto`, so the normal start
links the AI to the ESP32 – no IP address to type:

1. ESP32 powered (USB), Serial Monitor shows `[WiFi] Connected`.
2. Laptop on the **same Wi-Fi** (your phone hotspot).
3. **Close the test console** (19 EXIT). Only one laptop program can be connected to the ESP32;
   a newer connection replaces the older one, and the replaced program stops reconnecting.
4. Start the app: double-click `start_sentinel.bat` (or
   `.venv\Scripts\python run_app.py`). With the phone camera:
   `start_sentinel.bat --source http://PHONE_IP:8080/video`.

Expected in the app window (black console):

```text
Robot link enabled: searching the Wi-Fi for the ESP32 (port 3333) ...
Searching for the robot on 10.61.254.0/24 (port 3333) ...
Robot found at 10.61.254.98 (network scan)
Robot link up: 10.61.254.98:3333
ESP32 connected at 10.61.254.98 - sending the current AI state (model running, fall alert none)
```

and on the Serial Monitor `[RX] Event received: MODEL_STARTED` – **the red LED turns on and stays
on while the AI runs**. A confirmed fall (red alert on the dashboard) → `ESP32 <- FALL_DETECTED` →
LED blinks + buzzer. Press **Acknowledge** on the dashboard → `FALL_CLEARED` → LED solid again.
Close the app → `MODEL_STOPPED` → LED off.

How `auto` finds the ESP32 (`robot/discovery.py`): the address that worked last time
(`config/robot_last_host.txt`), then `sentinel-robot.local`, then a scan of the laptop's own
Wi-Fi network (x.x.x.1–254) for a device that answers on port 3333 with the SentinelAI hello. Only
private (LAN) addresses are scanned. While searching, the ESP32 may print a short
`[TCP] Laptop client connected` / `disconnected` pair – harmless. If the robot drops off the
network, the app keeps running and searches again. A fixed address still works:
`host: 10.61.254.98`, or `start_sentinel.bat --robot 10.61.254.98`; `--no-robot` = AI only.

| Symptom | Fix |
|---|---|
| `ESP32 not found (tried sentinel-robot.local and port 3333 on …)` | ESP32 off or on another Wi-Fi; laptop and ESP32 must be on the same hotspot; the hotspot may block devices from seeing each other – try `--robot <IP from Serial Monitor>` |
| LED does not light although the app runs | dashboard robot panel: connected? The test console may have taken over the link – close it, then press **Connect** in the robot panel |
| `Another laptop connected to the robot - this laptop stops reconnecting` | you started the console while the app was running; close the console, press **Connect** |
| LED blinks fast + 3 beeps without a confirmed fall | a *possible fall* warning (`robot.signal_on_suspected: true`); set it to `false` to signal confirmed falls only |

The dashboard's robot panel
(connect/disconnect, STOP, E-STOP, LED, buzzer test, clear emergency) works with the stage-1
firmware; arrow buttons are accepted as **simulated** motion because the test firmware has no
motors.

The AI app tells the ESP32 about its state – **only when the state changes**, never per frame
(`robot/robot_logic.py`, thread-safe, called from `src/app_runner.py`):

| App | Sent to the ESP32 | ESP32 |
|---|---|---|
| first processed frame | `MODEL_STARTED` (once) | LED continuously ON |
| model reports a **confirmed** fall (emergency alert) | `FALL_DETECTED` (once) | LED blinks, buzzer pattern, movement refused |
| operator acknowledges the alert on the dashboard (no emergency alert left) | `FALL_CLEARED` | buzzer off, LED ON again |
| app stops (Ctrl+C / window closed / error) | `MODEL_STOPPED`, then the link closes | LED and buzzer off |
| link reconnects | current state again (`FALL_DETECTED` or `FALL_CLEARED`, then `MODEL_STARTED` if running) | same state as before the drop – never a movement |

A "possible fall" warning does not send FALL_DETECTED. The operator E-STOP button is separate and
unchanged (emergency latch, needs *Clear emergency…*). The log (`logs/events.jsonl`) records each
`ROBOT_AI_EVENT`.

In your own code:

```python
from robot.esp32_client import ESP32Client
esp = ESP32Client("192.168.1.50")
esp.connect(); esp.model_started(); print(esp.get_status()); esp.model_stopped(); esp.disconnect()
# inside the app, reuse the running link instead of a second connection:
esp = ESP32Client("", client=runner.robot.client)
```

## 15. AI/ML status and remaining tasks

Done and measured (details: `evaluation/SUMMARY.md`, `reports/experiments.md`,
`reports/error_analysis.md`):

| Component | State | Measured |
|---|---|---|
| Person detection + pose | pretrained YOLO11n-pose (ONNX) | person found in 92.7 % of dataset frames; no box/keypoint ground truth → precision/recall **cannot** be measured on this dataset |
| Tracking | own IoU tracker | no ID ground truth → not measured |
| Posture (STANDING/SITTING/LYING) | Random Forest on keypoints, separate from fall detection | leave-one-subject-out accuracy 0.976, macro-F1 0.965 |
| Fall events | temporal classifier + state machine; "lying ≠ fall"; uncertain/visibility labels | nested LOSO: FALL_SUSPECTED F1 0.890, FALL_CONFIRMED F1 0.819, false alarms and delay reported |
| Speed (build machine, 2 CPU cores) | – | pose 53.5 ms/frame; your laptop: run `tools\check_system.bat` |
| Dataset pipeline | GMDCSA-24 (MIT), subject-level splits, inventory, validation tests | – |
| Experiments | resolution, model choice (image vs keypoint), thresholds, augmentation, imbalance, temporal, lighting | – |

Remaining (in order):
1. **Run on your laptop**: `tools\check_system.bat` (real webcam index, speed), then the app on
   your IP Webcam stream (`--source http://PHONE_IP:8080/video`) – real FPS and latency.
2. **Your own labelled recordings** (with consent; `training/collect_data.py`,
   `training/annotate_videos.py`): robot-height camera, your rooms, night/dim light, lying on a
   bed vs floor. This is the only way to measure real-world accuracy – today's numbers come from
   4 actors in one dataset.
3. **Second public dataset** as an external test (UR Fall, Le2i, CAUCAFall – licences must be
   checked; they could not be downloaded in my environment).
4. Re-run `training/experiments.py` and the error report on the new data; deploy a new model
   only if it is better (`training/model_registry.py`).
5. Robot integration in order: emergency on confirmed fall (ready) → manual control (after
   motors) → following (after manual control is verified) → obstacle handling (after a sensor is
   chosen and verified).

## 16. Step-by-step summary

1. Wire the LED and buzzer on the breadboard (section 9) – USB unplugged, no motors, no battery.
2. Create `firmware\esp32_test\secrets.h` with your 2.4 GHz Wi-Fi (section 6).
3. Arduino IDE: open `esp32_test.ino`, board **DOIT ESP32 DEVKIT V1**, your COM port, Verify,
   Upload (section 7); watch the self-test blink/beep; note the IP from the Serial Monitor.
4. Check the network: `ping <IP>`, `Test-NetConnection <IP> -Port 3333` (section 8).
5. LED/buzzer without the model: `python -m robot.test_console --host <IP> --hw-test`.
6. Manual sequence: `python -m robot.test_console --host <IP>`, steps 1–26 of section 8.3, Serial
   Monitor open beside it.
7. Automatic: `--run-all` and `python -m robot.comm_test --host <IP>`.
8. The "You" tests: Wi-Fi off/on, EN button, closing and reopening the console.
9. Optional: the full app with `robot.enabled: true` (section 14) – LED on while the model runs.
10. Send me `logs\esp32_test_*.log`, `evaluation\communication\comm_test_*.json` and the Serial
    Monitor text. After that: motors (section 13).
