# SentinelAI — AI + IoT Elderly Safety and Smart Assistance Robot

College engineering prototype. **Not a medical device**: it can miss falls and
can raise false alarms (measured below).

| Phase | Content | Status |
|---|---|---|
| 1 | Laptop AI/ML system: camera, pose, tracking, posture, fall detection, training, dashboard, logging, tests | **implemented, trained, tested in the build environment** — laptop/phone checks pending (docs/TESTING.md §3) |
| 2 | Laptop ↔ ESP32 Wi-Fi communication (JSON protocol v3), central safety supervisor, dashboard robot controls | **implemented; firmware compiles (cores 3.3.12 and 2.0.17, 18 build variants); tested against the real firmware core in simulation** — tests on your ESP32 pending (reports/test_plan.md) |
| 3 | ESP32 + L298N motors, buzzer, red LED | wiring/pin/calibration docs written (docs/HARDWARE.md); **power plan waits for your battery details**; nothing tested on hardware |
| 4 | Person following (operator-selected target, follow state machine) | implemented and tested in simulation only |
| 5 | Full integration and evaluation | not started |

## 1. What Phase 1 does

```text
camera → YOLO11n-pose (people + 17 keypoints) → tracker (IDs) → features
       → posture classifier (Random Forest) → fall-window classifier + state machine
       → alerts + event log → web dashboard
```

* Pretrained models (not trained by us): Ultralytics YOLO11n-pose and YOLO11n (ONNX, CPU).
* Trained by this project on the public **GMDCSA-24** fall dataset (159 videos, 4 people):
  a posture classifier and a fall-window classifier.
* Fall states: NORMAL → POSSIBLE_FALL → FALL_SUSPECTED → FALL_CONFIRMED → RECOVERY_OR_RESET.
  FALL_CONFIRMED is the emergency; with the robot link enabled it is sent to the ESP32 (Phase 2).

Details: [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## 2. Install (Windows 10/11)

Python **3.12** is recommended (3.11 minimum).

```powershell
cd "E:\Sentinal AI"
py -0p                                  # list installed Pythons
py -3.12 -m venv .venv
.\.venv\Scripts\Activate.ps1            # if blocked: see docs/TROUBLESHOOTING.md
python -m pip install --upgrade pip
pip install -r requirements.txt         # runtime only
pip install -r requirements-train.txt   # + training, evaluation and tests
python -m pytest -q                     # should print "passed" (dataset tests skip if no videos)
```

The pretrained ONNX models and trained classifiers are already in `artifacts/`.
If a model file is missing: `python training/download_datasets.py --models-only`.

## 3. Run

```powershell
.\start_sentinel.bat                                     # easiest: double-click it (or pass the options below)
python run_app.py                                        # laptop webcam, opens http://127.0.0.1:5000
python run_app.py --source http://PHONE_IP:8080/video    # Android IP Webcam
python run_app.py --source "path\to\video.mp4" --realtime
python run_app.py --mode window                          # OpenCV window (q quit, a acknowledge)
python run_app.py --mode headless --max-frames 300       # no UI, only logs
python run_app.py --no-objects                           # faster: skip furniture detection
```

* Speed: about 10 FPS end-to-end on the 2-core build machine (your laptop will differ).
* Dashboard: live video with skeletons and IDs, posture + reliability, fall state,
  FPS, inference time, objects, active alerts (Acknowledge button), recent events.
* Events are written to `logs/events.jsonl` and `logs/events.csv`. No video is saved.
* All thresholds are in `config/settings.yaml`.

## 4. Measured results (Phase 1)

All numbers come from `evaluation/` and were produced by the scripts in
`training/`. Protocol: **leave-one-subject-out** — each person is tested by
models trained without them; model type and threshold were chosen inside the
training folds only.

**Posture** (10,130 labelled frames, 145 videos):

| Model | Accuracy | Macro-F1 |
|---|---|---|
| Rule baseline | 0.799 | 0.833 |
| **Random Forest (deployed)** | **0.976** | **0.965** |

**Fall detection, event level** (79 labelled falls, 159 videos):

| System | Alarm | Precision | Recall | F1 | False alarms/hour* | Median delay |
|---|---|---|---|---|---|---|
| Rule baseline | FALL_SUSPECTED | 0.753 | 0.848 | 0.798 | 81 | 0.92 s |
| **Trained** | **FALL_SUSPECTED** | **0.859** | **0.924** | **0.890** | 44 | 0.93 s |
| Rule baseline | FALL_CONFIRMED | 0.838 | 0.722 | 0.775 | 41 | 2.49 s |
| **Trained** | **FALL_CONFIRMED** | **0.871** | **0.772** | **0.819** | 33 | 2.55 s |

\* extrapolated from ~16 minutes of non-fall video, so very approximate. Most
false alarms come from push-ups, sitting down on the floor and lying down on a
bed quickly (`evaluation/fall/error_analysis.csv`). Many clips end 1–3 s after the
fall, which limits FALL_CONFIRMED recall.

Full reports: [evaluation/SUMMARY.md](evaluation/SUMMARY.md).

## 4b. Stage 1 — ESP32 Wi-Fi communication test (start here with the hardware)

Before any motor is connected: flash `firmware/esp32_test/esp32_test.ino` (no motor code;
FORWARD/BACKWARD/LEFT/RIGHT are only simulated), wire the red LED and buzzer, and run

```powershell
.venv\Scripts\python -m robot.test_console --host <ESP32 IP>            # menu: every command, invalid messages, emergency reset
.venv\Scripts\python -m robot.test_console --host <ESP32 IP> --run-all  # automatic run + summary + log file
```

Complete guide (network, protocol, upload, checklist, wiring, troubleshooting):
**[docs/STAGE1_COMM_TEST.md](docs/STAGE1_COMM_TEST.md)**.

## 5. Phase 2 — talking to the ESP32

```text
laptop (robot/)  --Wi-Fi, TCP port 3333, one JSON object per line-->  ESP32 (firmware/sentinel_robot)
  ping every 150 ms, emergency_stop first,               acks with command_id, status every 1 s,
  safety supervisor + priority queue, follow stream      watchdog 500 ms, emergency latch, obstacle hold
```

| AI / operator situation | Robot command(s) | Robot reaction |
|---|---|---|
| Fall suspected | following ends; `led blink 200 ms`, `buzzer warning` | warning blink + 3 beeps (no latch) |
| AI model running / stopped | `command MODEL_STARTED` / `MODEL_STOPPED` (on state changes only) | red LED continuously ON / OFF |
| Fall confirmed | `command FALL_DETECTED` (re-sent after reconnects) | all motion + following stopped, LED blinks, buzzer alert (max 60 s); movement refused until the operator acknowledges (`FALL_CLEARED`) |
| Operator clicks a person, then following | `follow_mode {target_id}`, then `follow {...}` every frame | follows that person only; stops when uncertain, lost, too close, at an obstacle or a fall |
| Dashboard arrows (press and hold) | `motor {mode:"manual", ..., duration_ms:400}` every 150 ms, `stop` on release | moves while held |
| Obstacle seen by the camera | `obstacle {source:"camera"}` (repeated), later `obstacle_clear` | refuses forward and following motion |
| Robot's own sensor (optional HC-SR04) | – (robot decides) | stops forward motion, `obstacle` event |

Safety rules (who may move the robot, and when nothing resumes by itself): [docs/SAFETY.md](docs/SAFETY.md).
Wiring and calibration: [docs/HARDWARE.md](docs/HARDWARE.md).

* Every command: [docs/COMMANDS.md](docs/COMMANDS.md) · message format: [docs/PROTOCOL.md](docs/PROTOCOL.md) ·
  network options: [docs/NETWORK.md](docs/NETWORK.md) · flashing the board: [docs/ESP32_SETUP.md](docs/ESP32_SETUP.md).
* The firmware ships with **`MOTORS_ENABLED 0`**: motor pins are never driven; the robot only
  reports the motion it would make. Wi-Fi details go in `firmware/sentinel_robot/secrets.h`
  (copy `secrets.example.h`; the file is never published).
* Without hardware you can try everything against a simulated robot:

```powershell
.\start_sentinel.bat --robot-sim                    # full app + simulated robot (robot panel, arrows, click a person to follow)
.venv\Scripts\python -m robot.robot_console --sim  # type commands by hand: led blink, motor 30 -30, follow 3, obstacle 40, estop ...
.venv\Scripts\python -m robot.sim_robot           # simulated robot on 127.0.0.1:3333 (window 1)
.venv\Scripts\python -m robot.comm_test --host 127.0.0.1   # 15-step automated test plan (window 2)
tools\check_system.bat                                     # laptop self-check: cameras 0-3, model speed, settings
```

* With the real board (IP from the Serial Monitor):

```powershell
.venv\Scripts\python -m robot.network_check --seconds 15   # phone camera AND robot reachable together?
.venv\Scripts\python -m robot.comm_test --host 192.168.43.50
.\start_sentinel.bat --robot 192.168.43.50                    # or set robot.enabled/host in settings.yaml
```

Clear an emergency from the dashboard: **Acknowledge** the alert, check on the person, then
**Clear emergency…** and confirm (sends `clear_emergency {operator_confirmed:true}`). After a
communication fault press **STOP** once the link is back. Nothing moves again by itself.

## 6. Train again / add your own data

See [docs/TRAINING.md](docs/TRAINING.md). Short version:

```powershell
python training/download_datasets.py        # ~2 GB dataset (only needed for pose extraction)
python training/prepare_dataset.py
python training/inspect_dataset.py
python training/split_dataset.py
python training/extract_pose_features.py    # slow; results are already included
python training/train_posture_classifier.py
python training/train_fall_classifier.py
python training/evaluate_models.py
```

## 7. Project layout

```text
start_sentinel.bat      one-click start on Windows (first run creates .venv and installs packages)
run_app.py              start the live system
config/settings.yaml    every threshold
src/                    live inference (camera, detection, tracking, analysis, alerts, dashboard, logging)
training/               dataset download/prepare/inspect/split, pose extraction, training, evaluation, data collection, annotation
robot/                  laptop side of the robot link: protocol, Wi-Fi client, safety gate, simulator, console, tests tools
firmware/sentinel_robot ESP32 Arduino sketch (config.h, robot_core = protocol + safety logic)
firmware/host_test      PC unit tests and TCP simulator built from the same robot_core
artifacts/              ONNX models + trained classifiers with model cards
data/                   annotations, cached pose features, splits (videos are downloaded, not included)
evaluation/             measured metrics, confusion matrices, error analysis, summary
tests/                  pytest suite
docs/                   architecture, datasets, training, testing, troubleshooting, protocol, network, ESP32 setup
```

## 8. Known limitations

* Trained on 4 young adults performing staged falls in 3 rooms; real elderly falls may look different.
* Camera heights in the data were 64–135 cm; a robot camera near the floor sees people differently — collect your own data from the robot's viewpoint.
* BENDING and KNEELING come only from rules (no labelled training data yet).
* A person must be visible: if they fall completely out of view, the system raises a
  TRACK_LOST_DURING_FALL warning, not a confirmed fall.
* False alarms per hour are far too high for unattended real-world use; this is a prototype.

## 9. Licences and credits

* GMDCSA-24 dataset: MIT licence — E. Alam et al., *Data in Brief* 2024, DOI 10.5281/zenodo.12921216.
* YOLO11 models: Ultralytics, AGPL-3.0.
