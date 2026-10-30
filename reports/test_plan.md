# Test plan and status

Status words (master prompt): **Not implemented** · **Implemented, untested** · **Tested
independently** · **Integrated and tested** (simulation / recorded video) · **Requires hardware
verification**. Nothing marked "Integrated and tested" has been run on your ESP32, motors,
buzzer, LED, phone camera or laptop in this version.

Build environment for every result below: Linux, Python 3.13, 2 CPU cores, no GPU.

## 1. Automated tests – `python -m pytest -q` → **205 passed, 0 failed** (≈ 3.5 min)

| File | Tests | What it proves | Status |
|---|---|---|---|
| test_protocol.py | 26 | every message type builds and validates; 21 invalid commands refused by the laptop; encoding exact and ≤ 200 chars; everything the laptop can send is accepted by the robot parser (500 random commands round-trip) | Tested independently |
| test_core_parity.py | 21 | Python simulator ≡ real firmware core, byte for byte: scenario covering all 13 types, all 18 named commands incl. the 4 AI events, 15 error codes and 6 events (3 configurations) + 16 random scripts × 700 steps incl. malformed JSON | Tested independently |
| test_firmware_core.py | 6 | 570 C++ checks of `robot_core.cpp` (AI events → LED/buzzer states, LED priority, non-restarting fall alert, exact Serial log lines, parser, named commands never driving motors, validation, watchdog, comm fault, emergency latch + bounded buzzer, LED/buzzer patterns, effective LED status, follow rules, obstacles, queue, millis wrap); laptop ↔ firmware settings agree; no Wi-Fi password in published source or serial prints; stage-1 sketch has no motor code and the same core | Tested independently |
| test_robot_comm.py | 26 | full 15-step comm test vs the C++ and Python simulators; heartbeat; frozen laptop → watchdog_stop reported; reconnect never resumes and flags a comm fault; commanded vs reported kept apart; emergency retried and re-latched after a robot reboot; "replaced" stops reconnect ping-pong; disconnect/close send stop; garbage/over-long data; protocol mismatch; stale motion refused; command queue priority, coalescing, stale drop; RobotLink press-and-hold, emergency, alerts, comm-fault acknowledgement, refusal logging | Integrated and tested (simulation) |
| test_safety_supervisor.py | 21 | each condition → its level; priority correct for every pair of conditions; hard stops block all motion; permissions combine; comm-fault latch needs operator on a healthy link | Tested independently |
| test_follow.py | 24 | follow geometry, never reverses, no distance claims; bounded acceleration (stopping never delayed); camera obstacle rules; all follow state-machine transitions incl. no target switching and operator resume; FollowManager with a simulated robot (target selection, lost target, obstacle stop/resume, stale frames, fall alert, emergency, manual refused while following) | Integrated and tested (simulation) |
| test_dataset_splits.py | 5 | LOSO folds and fixed split subject-disjoint; every video tested once; excluded video never used; inventory matches index; model registry sha256 matches deployed files | Tested independently |
| test_app_dashboard.py | 5 | real fall video through the whole pipeline; dashboard endpoints; assessment labels; robot actions refused when the link is disabled; with a simulated robot: buzzer/LED test, manual, follow selection errors, E-STOP latches, clear needs confirmation, stop, disconnect/connect, events logged | Integrated and tested (recorded video + simulation) |
| test_fall_tracker.py, test_features_posture.py, test_infrastructure.py, test_training_lib.py | 38 | Phase 1 AI logic (unchanged in this session, still passing) | Integrated and tested |

| test_esp32_test_stage.py | 24 | stage-1: named-command schema (18 names), AI events drive the LED/buzzer state, LED/buzzer test without the model, emergency over fall alert, disconnect turns the model light off and resumes nothing, the app sends AI events on state changes only, every command acknowledged, simulated movement reported and never driving, LED/buzzer status, emergency needs explicit reset, lost connection clears the movement and nothing resumes, offline ESP32 gives readable errors, `test_console --run-all` vs C++ core and Python simulator, offline exit code | Integrated and tested (simulation) |
| test_discovery.py | 8 | `robot.host: auto`: only a SentinelAI protocol-3 hello counts as the robot; network scan finds it and remembers the address; a lost robot triggers a new search (no resume); a fixed host is never searched; only private networks scanned | Integrated and tested (simulation) |

## 2. Firmware builds

`arduino-cli`, board DOIT ESP32 DEVKIT V1, `--warnings all`, ESP32 Arduino cores **3.3.12** and
**2.0.17**, 9 variants each (default, no secrets.h, motors on, sensor on, access point, passive
buzzer, static IP, all options + dead-zone 20, board "ESP32 Dev Module"): **18/18 compile; the only
warning is the intended "secrets.h not found"** (log: reports/firmware_build_matrix.log). Flash use
56–73 %. Status: Tested independently (compile only).

## 3. Measured in simulation (loopback – NOT Wi-Fi numbers)

| Item | Result |
|---|---|
| comm_test vs C++ simulator, Python simulator, C++ with motors+sensor (with and without `--allow-motion`) | 15/15 steps each |
| watchdog stop after silence | 503–504 ms (setting 500) |
| camera obstacle hold expiry | ≈ 1503 ms (setting 1500) |
| emergency_stop behind 20 queued pings | acknowledged in < 1 ms (loopback) |
| app + simulated robot + recorded video, operated through the dashboard API | select → follow mode on robot; E-STOP → robot reports emergency, LED on; clear → idle; SIGTERM → stop sent and link closed |

## 4. AI evaluation (reports/experiments.md, reports/error_analysis.md, evaluation/SUMMARY.md)

| Item | Result | Status |
|---|---|---|
| Posture, LOSO (deployed RF) | accuracy 0.976, macro-F1 0.965 | Integrated and tested (dataset) |
| Fall, nested LOSO | SUSPECTED F1 0.890, CONFIRMED F1 0.819 | Integrated and tested (dataset) |
| Experiments A–H | A, B, C, D, E, G, H run; F justified as not run | Tested independently |

## 5. Tests you must run (hardware / laptop) – Requires hardware verification

Do them in this order. Motors stay disconnected (or `MOTORS_ENABLED 0`) until H-tests.

| # | Test | How | Expected | Your result |
|---|---|---|---|---|
| L1 | Laptop self-check | double-click `tools\check_system.bat` | packages present, settings OK, one camera "live picture", pose model time printed | |
| L2 | Unit tests on Windows | `.venv\Scripts\python -m pytest -q` | all pass (C++ tests skip without g++) | |
| L3 | Real webcam | `start_sentinel.bat --source <index from L1>` | live video, skeleton, FPS | |
| L4 | Phone camera | IP Webcam, `--source http://PHONE_IP:8080/video` | camera pill green | |
| L5 | Stream drop | stop IP Webcam 10 s, restart | stalled → ok, no fall alert | |
| C1 | Board boots | Serial Monitor 115200 | `protocol v3, motors DISABLED`, Wi-Fi connected, robot IP | |
| C2 | Camera + robot together | `python -m robot.network_check --seconds 15` | both reachable | |
| C3 | Automated plan | `python -m robot.comm_test --host <IP>` | ALL PASSED; note ping mean/p95 | |
| C4 | Wi-Fi drop | `comm_test --manual-network-test` | step M passes | |
| C5 | Robot power loss | unplug ESP32 USB while connected, replug | dashboard: comm fault; nothing moves; STOP clears it | |
| C6 | Dashboard robot panel | `start_sentinel.bat --robot <IP>` | commanded and reported columns agree; status age < 1.5 s | |
| C7 | Emergency | dashboard E-STOP, Mute, Clear emergency… | ESP32 confirms latch; clear needs the dialog | |
| C8 | LED / buzzer (wired: LED GPIO 22, buzzer GPIO 23) | `test_console --hw-test`, then menu 13–16 | LED on / blink / off; buzzer as docs/STAGE1_COMM_TEST.md §8.3 | |
| H1 | Motor direction, dead zone, trim | docs/HARDWARE.md §6, wheels lifted | values written into config.h | |
| H2 | Stop paths with motion | `comm_test --allow-motion`, wheels lifted | watchdog / disconnect stop the wheels ≤ 0.6 s | |
| H3 | Press-and-hold | dashboard arrows, wheels lifted | moves only while held; release stops | |
| H4 | Following | phone on robot, empty room, low speed, click a person | follows, stops when you leave view, never reverses | |
| H5 | Obstacle | box in front while following | OBSTACLE_STOP; Resume needed | |
| H6 | Staged fall (safe) | volunteer lowers themselves onto a mattress | robot stops, LED on, buzzer; alert on dashboard | |
| H7 | (optional) HC-SR04 | hand 15 cm in front | `obstacle_source` sensor, forward refused | |

Stage-1 communication test (do this first, no motors): manual sequence with exact Serial output
docs/STAGE1_COMM_TEST.md §8.3, checklist §8.5.
Test firmware builds: 12/12 (6 variants × cores 3.3.12 and 2.0.17), only the intended
"secrets.h not found" warning – reports/firmware_build_matrix_esp32_test.log.

Never perform or ask anyone to perform a real fall.
