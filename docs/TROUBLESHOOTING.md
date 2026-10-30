# Troubleshooting

For each problem: what it means → likely cause → fix → how to check the fix.

## Installation

**`.\.venv\Scripts\Activate.ps1 cannot be loaded because running scripts is disabled`**
* Meaning: PowerShell's execution policy blocks scripts.
* Fix (only for your user, not the whole PC):
  `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned`, or use Command Prompt:
  `.venv\Scripts\activate.bat`.
* Check: the prompt starts with `(.venv)` and `where python` shows the `.venv` path first.

**`ERROR: No matching distribution found for numpy...` / `scikit-learn>=1.9`**
* Meaning: your Python version is too old for these packages.
* Cause: Python < 3.11.
* Fix: install Python 3.12, recreate the venv: `py -3.12 -m venv .venv`.
* Check: `python --version` inside the venv prints 3.12.x.

**`InconsistentVersionWarning` or `ValueError ... was trained with different features` when starting**
* Meaning: the saved models were made with another scikit-learn version or an older feature list.
* Fix: retrain from the cached pose features (no videos needed):
  `python training/train_posture_classifier.py` then `python training/train_fall_classifier.py`.
* Check: the dashboard header shows `posture: model · fall: model`.

## Camera

**Dashboard shows `camera: error (could not open webcam #0)`**
* Cause: another app (Zoom, Teams, Camera app) uses the webcam, or the index is wrong.
* Fix: close other camera apps; try `--source 1`.
* Check: `python run_app.py --mode window --source 0` shows the picture.

**IP Webcam URL does not work**
* Check in a browser on the laptop: `http://PHONE_IP:8080` must open the IP Webcam page.
* Use the video URL: `http://PHONE_IP:8080/video`.
* Phone and laptop must be on the same Wi-Fi; some guest/college networks block
  device-to-device traffic (client isolation) — use a phone hotspot or home router.
* Check: `python run_app.py --source http://PHONE_IP:8080/video` → camera pill turns green.

**Camera status `stalled`, then reconnects**
* Meaning: no frame for `camera.stall_timeout_s` seconds.
* Cause: weak Wi-Fi, phone screen locked / app in background, phone power saving.
* Fix: keep IP Webcam in the foreground, disable battery optimisation for it, lower its resolution (e.g. 640×480).
* The fall detector deliberately ignores the gap: a disconnect never triggers a fall.

## Performance

**FPS below ~5**
* Lower the IP Webcam resolution (640×480 is enough).
* Disable objects: `python run_app.py --no-objects` or raise `detection.object_every_n_frames`.
* On Windows with a GPU try `pip uninstall onnxruntime` then `pip install onnxruntime-directml`
  (the code automatically uses `DmlExecutionProvider` when present).
* Check: dashboard "inference ms" drops.

## Detection quality

**Posture often UNKNOWN**
* Too few keypoints visible (person cut off by the frame, dark room). Place the camera so the whole body is visible; improve lighting.
* `posture.unknown_proba_threshold` / `posture.min_visible_keypoints` control this.

**False fall alarms when someone lies down quickly or does push-ups**
* Known limitation (see `evaluation/fall/error_analysis.csv`).
* Options: raise `fall.model_threshold` (e.g. 0.7) — fewer false alarms but more missed falls; or collect your own negative examples and retrain.

## Missing files

**`FileNotFoundError: Model file not found: artifacts/pose_estimation/yolo11n-pose.onnx`**
* Fix: `python training/download_datasets.py --models-only`.

**`data/annotations/video_index.csv missing`**
* Fix: run `python training/prepare_dataset.py` first (after downloading the dataset).

## Robot link (Phase 2)

See also `docs/NETWORK.md` (network problems) and `docs/ESP32_SETUP.md` (upload problems).

**Upload stops at `Connecting........`**
* Hold the BOOT button on the ESP32 until the upload starts; use a data USB cable; install the CP210x/CH340 driver.

**Dashboard robot panel: `NOT CONNECTED ... cannot connect ... timed out`**
* Meaning: the laptop cannot open TCP port 3333 on the robot.
* Check the robot IP on the Serial Monitor equals `robot.host`; laptop and robot on the same network; run `python -m robot.network_check`.

**`robot speaks protocol v2, laptop v3` / the link drops right after connecting**
* Firmware and laptop software are from different versions — upload the firmware from this project folder again.

**Many `timeouts` / reconnects in the robot panel**
* Weak Wi-Fi or a busy hotspot. The robot stops by itself when replies are missing (watchdog), which is the safe behaviour. Move closer or use a less busy network.

**Robot stays in EMERGENCY after a power cycle**
* Intended: the laptop re-sends `emergency_stop` while its emergency latch is set (also after the robot rebooted). Use **Clear emergency…** on the dashboard (or `clear` in `python -m robot.robot_console`).

**Following does not start: `robot not connected` / `link not healthy` / `emergency active`**
* The robot must be connected and replying, and no emergency may be latched (reset it first).

**Following stops with `obstacle ahead` although the path looks free**
* The camera check treats objects low in the middle of the image as obstacles. Point the phone slightly upward, or tune `robot.follow.obstacle` (`corridor_width`, `bottom_frac`, `min_area_frac`, `classes`) in `config/settings.yaml`.

**FPS drops while following**
* Object detection runs every 2nd frame while following (`robot.follow.obstacle.object_every_n_frames`). Raise it to 3-4 on a slow laptop.
