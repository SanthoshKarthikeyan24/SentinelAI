# Testing

## 1. Automated tests (run them yourself)

```powershell
pip install -r requirements-train.txt
python -m pytest -q
```

| File | What it checks |
|---|---|
| `tests/test_features_posture.py` | angles/ratios on synthetic skeletons, scale invariance, missing keypoints → NaN, rule postures, UNKNOWN, smoothing, window features |
| `tests/test_fall_tracker.py` | state machine: fast fall → POSSIBLE → SUSPECTED → CONFIRMED exactly once; slow lying down not confirmed; recovery; cooldown; track lost → warning only; camera interruption; operator reset; fall-state handover after an ID switch; upright newcomer does not inherit a fall; tracker IDs, brief misses, stale removal |
| `tests/test_infrastructure.py` | config validation, event log JSONL/CSV, alert dedup/ack/listener safety, camera file reading / missing file / unreachable stream, letterbox maths, pose model on a real dataset frame, object class validation |
| `tests/test_training_lib.py` | annotation parser incl. the malformed GMDCSA-24 rows, posture/fall window labelling rules, event matching and metrics |
| `tests/test_app_dashboard.py` | end-to-end: a real fall video through camera → pose → tracking → posture → fall → alerts → log, every dashboard endpoint, and the robot controls against a simulated robot |
| robot / safety / data tests | see reports/test_plan.md §1 |

Tests that need the downloaded dataset are skipped automatically if it is missing.

**Result in the build environment (Linux, Python 3.13, CPU):** see the bottom of
this file — all tests passed. They have **not** been run on your Windows laptop
yet; please run them once after installing (section 3).

## 2. Model evaluation (measured, not estimated)

See `evaluation/SUMMARY.md`, `evaluation/posture/report.md`,
`evaluation/fall/report.md` and `evaluation/fall/error_analysis.csv`.
How the numbers are computed is explained in `docs/TRAINING.md` §3–4.

## 3. Checks you must do on your laptop (cannot be done remotely)

Tick each one and note the result.

| # | Test | How | Expected |
|---|---|---|---|
| L1 | Install | follow README §2 | `pip` finishes without errors |
| L2 | Unit tests | `python -m pytest -q` | all pass (dataset tests may be skipped if you did not download it) |
| L3 | Webcam | `python run_app.py --source 0` | dashboard shows video, skeleton, FPS |
| L4 | Phone camera | `python run_app.py --source http://PHONE_IP:8080/video` | camera pill green, FPS shown |
| L5 | Stream drop | while running, stop IP Webcam on the phone for 10 s, then start again | status `stalled`→`ok`, event `CAMERA_INTERRUPTED`/`CAMERA_RESTORED`, **no fall alert** |
| L6 | Postures | stand, sit on a chair, lie on a bed/sofa in view | posture column follows (STANDING / SITTING / LYING) |
| L7 | Safe fall-like test | a volunteer slowly and safely lowers themselves onto a thick mattress and stays down ~5 s | FALL_SUSPECTED then FALL_CONFIRMED, red banner, event logged |
| L8 | Recovery | after L7, stand up | RECOVERED event; acknowledge clears the banner |
| L9 | Ordinary activity | sit down quickly, lie down on a bed normally, pick something up | note any false alarms in a table |
| L10 | Offline video | `python run_app.py --source "data/raw/public_datasets/GMDCSA24/Subject 2/Fall/02.mp4" --realtime` | alert appears around the fall (≈3 s) |
| L11 | Logs | open `logs/events.csv` in Excel | one row per event with time, id, postures, state, reason |

Never perform or ask anyone to perform a real fall.

## 4. Results recorded in the build environment

Environment: Linux, Python 3.13, 2-core CPU, no GPU, ONNX Runtime CPU provider.

| Check | Result |
|---|---|
| `python -m pytest -q` | **41 passed** (0 failed, 0 skipped) |
| Pose extraction over 160 GMDCSA-24 videos | 160 processed, 0 failures (~22 min) |
| Headless app on a fall video (S3 Fall 04, fall starts 1.2 s) | POSSIBLE_FALL 1.47 s → FALL_SUSPECTED 3.15 s; the clip ends at 5.4 s, before the person has been down long enough for FALL_CONFIRMED |
| Headless app on an ADL video (S2 ADL 12, going to bed) | no alerts |
| Dashboard in a headless Chromium browser | page, MJPEG video, people table, events load; screenshot checked |
| End-to-end speed (pose + objects every 10 frames + tracking + classifiers) | ≈ 10 FPS on a 1280×720 video |

Not verified here (needs your hardware): laptop webcam, Android IP Webcam
stream, Windows install, OpenCV window mode (`--mode window`), Windows beep,
the interactive annotator/recorder windows. Use the table in §3.

---

# Phase 2 onward — robot link, safety, following, hardware

The complete, current test plan (every automated test file, the firmware build matrix, the
simulator results and the hardware tests you still have to run, each with its status) is in
**[reports/test_plan.md](../reports/test_plan.md)**. Known limitations:
[reports/known_limitations.md](../reports/known_limitations.md).
