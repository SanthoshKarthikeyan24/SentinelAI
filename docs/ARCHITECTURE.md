# SentinelAI architecture (Phase 1: laptop AI system)

## 1. Big picture

```text
 Android phone (IP Webcam) / laptop webcam / video file
            │  frames
            ▼
 ┌─────────────────────┐   src/camera/stream.py
 │ CameraStream        │   background reader, keeps only the newest frame,
 │                     │   detects stalls, reconnects with back-off
 └─────────┬───────────┘
           ▼
 ┌─────────────────────┐   src/detection/yolo_onnx.py
 │ PoseDetector        │   YOLO11n-pose (ONNX Runtime): person boxes + 17 keypoints
 │ ObjectDetector      │   YOLO11n (every N frames): chair, couch, bed, table, ...
 └─────────┬───────────┘
           ▼
 ┌─────────────────────┐   src/tracking/tracker.py
 │ PersonTracker       │   IoU + centre-distance matching (Hungarian algorithm),
 │                     │   one ID and one history per person
 └─────────┬───────────┘
           ▼  per tracked person
 ┌─────────────────────┐   src/analysis/features.py
 │ Feature extraction  │   frame features (angles, ratios) + window features (speeds, drops)
 └─────────┬───────────┘
           ▼
 ┌─────────────────────┐   src/analysis/posture.py
 │ PostureClassifier   │   trained Random Forest (+ rule baseline, UNKNOWN, smoothing)
 └─────────┬───────────┘
           ▼
 ┌─────────────────────┐   src/analysis/fall_detector.py
 │ FallDetector        │   trained fall-window classifier = motion evidence
 │ (state machine)     │   NORMAL → POSSIBLE_FALL → FALL_SUSPECTED → FALL_CONFIRMED → RECOVERY
 └─────────┬───────────┘
           ▼
 ┌─────────────────────┐   src/alerts/alert_manager.py, src/logging_system/event_logger.py
 │ Alerts + event log  │   dedup, acknowledgement, listeners (robot link in Phase 2), JSONL/CSV
 └─────────┬───────────┘
           ▼
 ┌─────────────────────┐   src/app_runner.py + src/dashboard/
 │ Dashboard           │   Flask web page: live video, people, states, alerts, events
 └─────────────────────┘
```

`src/pipeline.py` wires the AI steps together and contains **no UI code**.
`src/app_runner.py` runs the pipeline on its own thread; the dashboard only
reads the latest results, so a slow browser never slows the AI and slow AI
never freezes the page.

## 2. Why these choices

| Decision | Reason |
|---|---|
| ONNX Runtime instead of PyTorch at runtime | small Windows install, same code for training-time pose extraction and live use, CPU-friendly |
| YOLO11n (nano) models | run in real time on a laptop CPU; larger models can be exported later with Ultralytics |
| Own tracker instead of a library | needs only NumPy/SciPy, handles the fast box-shape change during a fall (centre-distance fallback) |
| Hand-designed features + Random Forest | works with ~160 videos, explainable (viva!), fast; a deep sequence model would need far more data |
| State machine after the classifier | a single frame cannot tell a fall from lying in bed; time, staying down and recovery matter |
| Everything thresholded in `config/settings.yaml` | tune without editing code |

## 3. The fall state machine

| State | Enter when | Leave when |
|---|---|---|
| NORMAL | start / after recovery | fall-window score ≥ threshold (and not in cooldown, and re-armed) → POSSIBLE_FALL |
| POSSIBLE_FALL | fall-like motion seen | person "down" for `suspect_min_down_s` → FALL_SUSPECTED; nothing for `possible_timeout_s` → NORMAL (logged FALL_DISMISSED) |
| FALL_SUSPECTED | motion + down | down another `confirm_down_s` → FALL_CONFIRMED; upright `recovery_up_s` → RECOVERY_OR_RESET; track lost → warning TRACK_LOST_DURING_FALL |
| FALL_CONFIRMED | still down | upright `recovery_up_s` → RECOVERY_OR_RESET; operator reset |
| RECOVERY_OR_RESET | recovery / reset | next frame → NORMAL with `cooldown_s` |

"Down" means: posture LYING, or torso tilted ≥ `torso_angle_low_deg`, or hips at
least `descent_drop_thresh` body-heights below where they were before the motion
(covers ending up seated on the floor).

Safety rules built in:
* timers only advance with real observations — **a camera drop can never confirm a fall**;
* camera interruption discards POSSIBLE_FALL evidence but keeps an active emergency;
* one fall → one event (re-arming + cooldown);
* a person who leaves the picture while a fall is suspected creates a warning, not silence.

## 4. Alerts

| Event | Alert severity | Robot action (robot link enabled) |
|---|---|---|
| FALL_CONFIRMED | emergency (stays until acknowledged) | `command FALL_DETECTED` → motion + following stopped, LED blinks, buzzer alert; `FALL_CLEARED` after the operator acknowledges |
| FALL_SUSPECTED | warning | following ends; LED blink + warning beeps (no latch) |
| TRACK_LOST_DURING_FALL | warning | as FALL_SUSPECTED; dashboard assessment "tracking lost" |

`AlertManager.add_listener()` connects alerts to `RobotLink.on_alert` and to the optional
alert-clip buffer (`src/alerts/clip_buffer.py`, off by default). Every person also gets a
plain-language assessment (`src/analysis/assessment.py`): no fall evidence / possible fall /
fall confirmed by model / lying, no fall evidence / unknown – poor visibility / tracking lost.

## 5. Training vs inference separation

* `src/` = live inference only. It **loads** `artifacts/*/…joblib`; it never trains.
* `training/` = dataset download, preparation, pose extraction, splitting,
  training, evaluation. It imports the same feature and state-machine code from
  `src/`, so the numbers measured offline are for the exact logic that runs live.

## 6. Data flow on disk

```text
data/raw/public_datasets/GMDCSA24/            original videos + csv (not modified)
data/annotations/video_index.csv              one row per usable video
data/annotations/video_events/events.csv      labelled intervals (standard format)
data/annotations/prepare_report.json          every repaired/excluded annotation
data/processed/pose_features/*.npz            keypoints + tracks per video (cached)
data/processed/temporal_sequences/cache_*.joblib  frame + window features (cached)
data/splits/splits.json                       subject-level folds
artifacts/posture_classifier/                 deployed model + model_card.json (+ versions/)
artifacts/fall_classifier/                    deployed model + model_card.json (+ versions/)
artifacts/model_registry.json                 versions, sha256, deploy history
data/dataset_inventory.csv                    one row per video (lighting, labels, pose coverage, split, licence)
evaluation/                                   metrics, reports, confusion matrices
logs/events.jsonl, logs/events.csv            runtime events
```

## 7. Robot communication and safety (protocol v3)

```text
 AlertManager ──alerts──► RobotLink (robot/robot_logic.py)  ◄── dashboard buttons / console
 FollowManager (robot/follow.py)  │   asks SafetySupervisor (src/safety/safety_supervisor.py)
   FollowStateMachine (src/safety/robot_state_machine.py): DISABLED/READY/FOLLOWING/
     TARGET_UNCERTAIN/TARGET_LOST/OBSTACLE_STOP/EMERGENCY_STOP
   FollowController: selected person's position → wheel speeds, RampLimiter (bounded acceleration)
   ObstacleDetector: objects in the driving corridor (camera heuristic)
                                  │
                                  ▼
                 CommandManager (robot/command_manager.py)   priority queue: emergency first (never
                                  │                           queued), stop purges motion, newest
                                  ▼                           motion wins, stale motion dropped
                 RobotClient (robot/wifi_client.py)          TCP thread: heartbeat, command_id/seq,
                                  │                           ack matching, status stream, comm-fault
                                  │                           flags, emergency retry + re-latch
                     Wi-Fi  TCP :3333, JSON lines v3 (docs/PROTOCOL.md, docs/COMMANDS.md)
                                  │
                 ESP32  sentinel_robot.ino  (Wi-Fi, TCP server, GPIO/PWM, trims, status LED, HC-SR04)
                                  │
                        robot_core.cpp   strict JSON parser + safety state machine
                        (idle / moving / following / emergency, follow target + min confidence,
                         obstacle hold + sensor hysteresis, watchdog + comm_fault, move time-outs,
                         bounded LED/buzzer patterns, status every second)
```

Safety is enforced on **both** sides (docs/SAFETY.md): the laptop's supervisor applies the
9-level priority (firmware emergency > e-stop > comm fault > obstacle > fall response > lost
target > following > manual > idle) and latches faults until an operator acts; the robot
independently stops on watchdog time-out, movement time-out, disconnection, Wi-Fi loss and
obstacles, and latches emergencies. Nothing resumes by itself after a reconnection.

The follow loop runs on the AI thread right after each frame and never waits for Wi-Fi;
each `follow` command lasts 0.4 s on the robot, so if frames stop arriving the robot stops.

`robot_core.cpp` contains no Arduino code, so the same file is compiled for the ESP32 and for
the PC (unit tests, transcript replay, TCP simulator). The Python simulator mirrors it line by
line and `tests/test_core_parity.py` proves identical output on a scenario plus 16 × 700 random
steps (including malformed JSON).

## 8. Models: versions and deployment

`training/model_registry.py` keeps every trained model in `artifacts/<kind>/versions/<id>/`
with its sha256 and model card; `artifacts/model_registry.json` records which version is
deployed and the deploy history. Training scripts only *register* a new model; it is used by the
app only after `--deploy` or `python training/model_registry.py deploy <kind> <id>`. The
dashboard's AI panel shows the deployed versions.
