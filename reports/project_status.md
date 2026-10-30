# SentinelAI — project audit (start of the master-prompt work)

Audit date: 2026-10-30. Method: read every source file in `E:\Sentinal AI` (mirrored in the
build environment), re-ran the automated tests, checked the laptop folder over the device link.

## 1. Environment

| Item | Finding | How verified |
|---|---|---|
| Language / frameworks | Python 3 (laptop), C++ Arduino sketch (ESP32). Key libraries: OpenCV, ONNX Runtime, NumPy/SciPy, scikit-learn 1.9, Flask | source + `requirements.txt` |
| Laptop Python | **Python 3.13** (`py -3.13`), virtual env `E:\Sentinal AI\.venv` created by `start_sentinel.bat` | watched the launcher window on your laptop |
| Laptop packages | numpy 2.5.4, scikit-learn 1.9.1, opencv-python 5.0.0.93, onnxruntime 1.31.1, scipy 1.18.1, Flask 3.1.3 | pip output on your laptop |
| Laptop app start | dashboard opened, webcam #0 "ok", models loaded, ≈12 FPS, 71 ms inference | screenshot on your laptop |
| Webcam #0 | shows a **static logo** — probably a virtual camera, not the real webcam | screenshot |
| IP Webcam from Python | **not verified by me** (you report it works in the browser; the stream URL is not in `settings.yaml` yet) | – |
| Build environment | Linux, Python 3.13, 2 CPU cores, no GPU; ESP32 toolchains (Arduino core 3.3.12 and 2.0.17) | used for all automated tests and compiles |

## 2. Models in use

| Model | File | Origin | Input | Settings | Size |
|---|---|---|---|---|---|
| Person + 17 keypoints | `artifacts/pose_estimation/yolo11n-pose.onnx` | Ultralytics YOLO11n-pose v8.3.0 release, COCO-Keypoints, AGPL-3.0 | fixed 640×640 | person_conf 0.40, NMS 0.5, keypoint_conf 0.35 | 11.9 MB |
| Objects (80 COCO classes) | `artifacts/object_detection/yolo11n.onnx` | Ultralytics YOLO11n, COCO 2017, AGPL-3.0 | fixed 640×640 | object_conf 0.40, every 10th frame | 10.9 MB |
| Posture | `artifacts/posture_classifier/posture_model.joblib` | **trained here**: Random Forest, 18 keypoint features, GMDCSA-24 (MIT) | features | UNKNOWN if p < 0.5 or < 7 keypoints | 0.95 MB |
| Fall window | `artifacts/fall_classifier/fall_model.joblib` | **trained here**: Hist Gradient Boosting, 30 temporal features, GMDCSA-24 | 1.5 s window | threshold 0.80 (chosen by nested LOSO) | 0.27 MB |

## 3. Feature status (before this work)

Legend: **NI** not implemented · **IU** implemented, untested · **TI** tested independently ·
**IT** integrated and tested (simulation / recorded video) · **HW** still requires hardware verification

| # | Feature | Status | Evidence / gap |
|---|---|---|---|
| 1 | Camera input (webcam, URL, file, reconnect, stall) | IT + HW | file-based tests pass; webcam ran on your laptop; IP Webcam from Python not yet verified |
| 2 | Person detection | IT | pretrained; coverage 92.7 % of dataset frames; **no box-level precision/recall/mAP** (no box labels) |
| 3 | Pose estimation | IT | pretrained; keypoint accuracy not measurable on GMDCSA-24 (no keypoint labels) |
| 4 | Tracking | IT | own IoU/centre tracker + fall hand-over; **no tracking metrics** (no ID labels) |
| 5 | Posture classification | IT | LOSO macro-F1 0.965 (3 classes); BENDING/KNEELING rules only; **only one approach compared against rules** |
| 6 | Temporal fall detection | IT | nested LOSO event F1 0.890 (suspected) / 0.819 (confirmed); no "lying without fall" / "unknown visibility" labels exposed |
| 7 | Dashboard + alerts | IT + HW | works in browser test and on your laptop; **no manual movement controls, no target selection, no emergency ack status** |
| 8 | Event logging | IT | JSONL/CSV; **no robot command/ack/motor/LED/buzzer fields, no reset time** |
| 9 | Dataset research / preparation / splits | IT | GMDCSA-24 used; **no `data/dataset_inventory.csv`**; others not downloadable here |
| 10 | Training + evaluation scripts | IT | posture, fall, tuning, error analysis; **no versioned model registry** (retraining overwrites) |
| 11 | Data collection + annotation tools | TI | headless paths tested; interactive windows HW |
| 12 | Laptop ↔ ESP32 protocol | IT (sim) | text protocol v2; **master prompt asks for versioned JSON with command IDs and periodic status** |
| 13 | ESP32 firmware | TI (compile + PC tests) + HW | compiles on 2 cores; never run on your board |
| 14 | Motor control | TI + HW | logic tested; outputs disabled (`MOTORS_ENABLED 0`); **no left/right calibration** |
| 15 | LED / buzzer | TI + HW | patterns tested in simulation; **buzzer emergency is unbounded** (master prompt forbids) |
| 16 | Watchdog / comm loss | IT (sim) + HW | 500 ms watchdog; **fault not reported on reconnect** |
| 17 | Central safety supervisor | **NI** | rules spread over `robot_logic.py` and `follow.py`; no documented 9-level priority |
| 18 | Person following | IT (sim) | **auto-switches target after loss** (master prompt forbids), resumes after obstacle automatically |
| 19 | Obstacle handling | IT (sim) | camera heuristic + optional HC-SR04 code; **no sensor confirmed** |
| 20 | Hardware docs (pins, wiring, power) | partial | pins in `ESP32_SETUP.md`; **no wiring/power plan; battery unverified** |
| 21 | Reports (error analysis, test plan, limitations) | partial | content scattered in `docs/` and `evaluation/` |

## 4. Spec file names → this project

The master prompt allows adapting file names to the existing code. Mapping:

| Master prompt | This project |
|---|---|
| `src/camera/camera_manager.py` | `src/camera/stream.py` |
| `src/detection/person_detector.py`, `src/pose/pose_estimator.py` | `src/detection/yolo_onnx.py` (one ONNX model gives boxes + keypoints) |
| `src/tracking/person_tracker.py` | `src/tracking/tracker.py` |
| `src/posture/posture_classifier.py` | `src/analysis/posture.py` |
| `src/fall_detection/fall_detector.py` | `src/analysis/fall_detector.py` |
| `src/following/follow_controller.py`, `src/obstacle_detection/obstacle_detector.py` | `robot/follow.py` |
| `src/safety/safety_supervisor.py`, `robot_state_machine.py` | **new:** `src/safety/` |
| `src/communication/protocol.py`, `esp32_client.py`, `command_manager.py` | `robot/commands.py`, `robot/wifi_client.py`, **new:** `robot/command_manager.py` |
| `firmware/esp32_robot/esp32_robot.ino` | `firmware/sentinel_robot/sentinel_robot.ino` |
| `src/dashboard/app.py` + panels | `src/dashboard/server.py` + `index.html` (one page, panels inside) |
| `src/logging/event_logger.py` | `src/logging_system/event_logger.py` |
| `training/train_posture.py`, `train_fall_model.py` | `training/train_posture_classifier.py`, `train_fall_classifier.py` |
| `configs/config.yaml` | `config/settings.yaml` |

## 5. Work list for this session

1. JSON protocol v3 (firmware + laptop + simulators), bounded buzzer, fault report on reconnect, Wi-Fi secrets out of source, motor trims.
2. Central safety supervisor with the 9-level priority; no automatic movement after reconnect, watchdog, emergency or obstacle.
3. Following: operator selects the target; no automatic switching; follow states; gradual speed changes.
4. Dashboard: manual press-and-hold controls, target selection, emergency panel with ESP32 acknowledgement, AI status panel.
5. Event log with robot fields; optional pre/post-event clip (off by default).
6. AI: dataset inventory, model registry/versioning, experiments A/C/D/E/G/H, image-based posture baseline, error-analysis report.
7. A self-check tool for your laptop (environment, cameras, model speed) that I can run and read.
8. Hardware documentation (pins, wiring, power plan pending battery details), test plan, known limitations.

The final state of every row is in §6, added at the end of the session.

## 6. Final state at the end of this session

Legend as in §3. "HW" = still needs your hardware; nothing below has run on your ESP32 yet.

| # | Feature | Now | What changed / evidence |
|---|---|---|---|
| 1 | Camera input | IT + HW | unchanged; `tools/check_system.py` finds the real camera index on your laptop |
| 2–4 | Detection, pose, tracking | IT | unchanged; camera-resolution experiment A: 1280×720 → 320×180 keeps 99.8 % detection, posture F1 0.976 → 0.968 |
| 5 | Posture | IT | image-based HOG classifier compared (macro-F1 0.747 vs keypoint RF 0.974); keypoint_conf, augmentation, class weighting, smoothing tested – deployed settings kept (reports/error_analysis.md §3) |
| 6 | Fall detection | IT | per-person assessment labels (no evidence / possible / confirmed by model / lying without evidence / unknown visibility / tracking lost / recovering) |
| 7 | Dashboard | IT (sim) + HW | press-and-hold arrows, connect/disconnect, STOP, E-STOP, mute, clear with confirmation, LED/buzzer tests, click-to-follow, resume, safety panel, commanded vs reported table, emergency ack panel, AI panel with model versions, CPU/RAM |
| 8 | Event log | IT | robot command/ack, comm, robot state, commanded/reported motors, LED, buzzer, emergency reason, reset time, uncertainty columns; old CSVs rotated |
| 9 | Dataset | IT | `data/dataset_inventory.csv` (160 rows incl. 1 excluded; lighting groups; MIT licence) |
| 10 | Training | IT | model registry (`training/model_registry.py`): versioned, sha256-checked, explicit `--deploy` |
| 12 | Protocol | IT (sim) | JSON v3 with command_id, seq, strict validation, periodic status, events; byte-identical C++/Python behaviour |
| 13 | Firmware | TI + HW | 18/18 builds; 264 PC checks; secrets.h; effective LED/buzzer pattern in status |
| 14 | Motor control | TI + HW | dead-zone + trims + invert; still `MOTORS_ENABLED 0` |
| 15 | LED / buzzer | TI + HW | emergency buzzer bounded to 60 s (`buzzer_timeout` event) |
| 16 | Watchdog / comm loss | IT (sim) + HW | fault reported on return (`comm_fault`, `watchdog_ok:false`); needs a fresh STOP |
| 17 | Safety supervisor | IT (sim) | `src/safety/safety_supervisor.py`, 9 levels, latches, docs/SAFETY.md |
| 18 | Following | IT (sim) + HW | operator-selected target, 7-state machine, no switching, bounded acceleration, obstacle stop needs Resume |
| 19 | Obstacles | IT (sim) + HW | camera heuristic stays labelled as heuristic; sensor separate (`obstacle_source`) |
| 20 | Hardware docs | written, HW | docs/HARDWARE.md: pin map, wiring tables, LED resistor, buzzer transistor, grounding, calibration; **power plan pending your battery details** |
| 21 | Reports | done | reports/test_plan.md, known_limitations.md, error_analysis.md, experiments.md |

Open questions for you (safety-critical, needed before Phase 3 wiring): battery label (chemistry,
cells, voltage, BMS), motor label (voltage, stall current), the IP Webcam URL, and which camera
index `tools\check_system.bat` reports as live.

## 7. Stage 1 – ESP32 communication test (added after the battery/motor answer)

* New: `firmware/esp32_test/` (no motor code; same tested core), named commands in protocol v3
  (`type:"command"`; FORWARD/BACKWARD/LEFT/RIGHT simulated), `robot/esp32_client.py`,
  `robot/test_console.py`, comm_test steps 15–17, `tests/test_esp32_test_stage.py`.
* Guide with network checks, upload, checklist, wiring and troubleshooting: `docs/STAGE1_COMM_TEST.md`.
* Status: compiled (12 test-firmware + 18 robot-firmware builds), 570 C++ checks (after the LED/buzzer update), 196 pytest
  tests, console and comm_test pass against the real core in simulation.
  **Requires hardware verification** on your ESP32 (STAGE1_COMM_TEST §8.3 / §8.5).
* Update (LED on GPIO 22 + buzzer on GPIO 23 wired, USB power): AI events `MODEL_STARTED`,
  `FALL_DETECTED`, `FALL_CLEARED`, `MODEL_STOPPED` (same protocol v3 `command` message); LED solid
  while the model runs, blink + buzzer pattern during a fall alert (non-blocking, never restarted by
  a repeat), plain-language Serial log of every message; console menu 1–19 with simulated/physical
  sections, `--hw-test`; the app sends the events on state changes only. 570 C++ checks; firmware
  builds 12/12 (test) and 18/18 (robot). **LED, buzzer and Wi-Fi not yet tested on the hardware.**
* Battery: 3 × 18650 with BMS – power plan options in docs/HARDWARE.md §4; BMS S-rating still needed
  before any motor wiring. Stage 1 runs from USB only.
