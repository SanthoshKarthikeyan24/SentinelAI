# Training and evaluation guide

All commands are run from the project folder with the virtual environment
active and the training requirements installed:

```powershell
pip install -r requirements-train.txt
```

## 1. The full pipeline (what was run to produce the shipped models)

| Step | Command | Time (2-core CPU) | Output |
|---|---|---|---|
| 1. Download models + GMDCSA-24 | `python training/download_datasets.py` | depends on internet (~2 GB) | `artifacts/*/*.onnx`, `data/raw/public_datasets/GMDCSA24/` |
| 2. Convert annotations | `python training/prepare_dataset.py` | seconds | `data/annotations/…` + `prepare_report.json` |
| 3. Inspect / validate | `python training/inspect_dataset.py` | seconds | `evaluation/dataset_inspection.json` |
| 4. Subject-level splits | `python training/split_dataset.py` | seconds | `data/splits/splits.json` |
| 5. Pose extraction | `python training/extract_pose_features.py` | ~22 min | `data/processed/pose_features/*.npz` |
| 6. Posture classifier | `python training/train_posture_classifier.py` | ~2 min | `artifacts/posture_classifier/` + `evaluation/posture/` |
| 7. (optional) design comparison | `python training/tune_fall_settings.py` | ~8 min | `evaluation/fall/tuning.json` |
| 8. Fall classifier | `python training/train_fall_classifier.py` | ~15 min | `artifacts/fall_classifier/` + `evaluation/fall/` |
| 9. Error analysis | `python training/analyse_errors.py` | ~2 min | `evaluation/fall/error_analysis.csv` |
| 10. Summary + speed | `python training/evaluate_models.py` | ~1 min | `evaluation/SUMMARY.md` |

Steps 5–8 are cached: if `data/processed/pose_features/` already exists (it is
shipped with the project), you can retrain the classifiers **without the
videos** in a few minutes — useful if your scikit-learn version differs.

## 2. How labels become model inputs

**Posture (one sample per frame)**
* For every sampled frame of the labelled person, find the annotated interval
  that contains the frame time.
* `Standing` and `Walking` → STANDING, `Sitting` → SITTING, `Sleeping` → LYING.
* Skipped: frames inside a fall interval, inside *Exercising* (e.g. yoga bends,
  push-ups — not a normal posture), in gaps between intervals, within 0.15 s of
  an interval edge, with two different posture labels, or with fewer than 7
  visible keypoints.
* Features: `FRAME_FEATURES` in `src/analysis/features.py` (torso angle, joint
  angles, normalised vertical offsets, limb ratios, box shape, keypoint visibility).

**Fall windows (one sample per frame, describing the last `window_s` seconds)**
* Positive (1): the window ends between 0.4 s and 2.0 s after a labelled fall start.
* Ignored: the window ends just before the fall start (boundary) or more than
  2.0 s after it (the person is already down — that is the state machine's job).
* Negative (0): everything else, including all ordinary activities and the
  pre-fall part of fall videos, and windows from any extra (non-labelled) tracks.
* Features: `WINDOW_FEATURES` (descent speeds and drops of hips/head/box,
  torso rotation, box-shape change, end posture, motion before/after, …).

## 3. Avoiding data leakage

* Splits are by **subject**: a person is never in both training and test data.
* Reported numbers use **leave-one-subject-out** (LOSO): train on 3 people,
  test on the 4th, repeat for each person, pool the results.
* Model type and decision threshold are chosen **inside** each training fold
  (nested LOSO), never on the test person.
* Design options (feature set, window length) were compared on subjects S1–S3
  only (`tune_fall_settings.py`), keeping S4 unseen for that decision.
* Honest note: after a first full evaluation, the error list (`analyse_errors.py`,
  all subjects) showed falls being lost when the tracker gave the fallen person a
  new ID. Three general fixes were then made to the state machine (fall-state
  hand-over to the new ID, "cannot tell" no longer counts as standing up, and
  detection gaps add at most 0.25 s to timers). No threshold was tuned on the
  test subjects, but because the problem was found by looking at all subjects,
  the reported numbers may be slightly optimistic. New data (your own
  recordings) is the real test.

## 4. Event-level evaluation (how a "detected fall" is counted)

1. The cached detections of each test video are replayed through the **same**
   PostureClassifier and FallDetector state machine used live.
2. An alarm (FALL_SUSPECTED or FALL_CONFIRMED) is a **true positive** if it occurs
   between 0.5 s before the labelled fall start and 1.0 s after the labelled end.
3. Each labelled fall can be matched once; extra alarms inside the same fall are
   counted as *duplicates*; alarms outside every fall are **false positives**.
4. Precision = TP / (TP + FP), Recall = TP / (TP + FN), F1 = harmonic mean.
5. False alarms per hour = FP / (hours of non-fall video).
6. Detection delay = alarm time − labelled fall start.

Frames are **not** counted as separate fall events.

## 5. Adding your own data (needed for BENDING / KNEELING and robot-height views)

```powershell
# record (asks for consent; use a pseudonym)
python training/collect_data.py --source http://PHONE_IP:8080/video --person P01 --activity BENDING
# label intervals (interactive window) or from the command line
python training/annotate_videos.py data/raw/collected_videos/<file>.mp4
python training/annotate_videos.py data/raw/collected_videos/<file>.mp4 --add BENDING 1.2 3.4
# rebuild everything
python training/prepare_dataset.py
python training/inspect_dataset.py
python training/split_dataset.py
python training/extract_pose_features.py
python training/train_posture_classifier.py
python training/train_fall_classifier.py
python training/evaluate_models.py
```

Tips: record each person in several sessions, vary rooms and lighting, record
**ordinary activities that look like falls** (sitting down fast, lying on a sofa,
picking things up, push-ups) — they are the most useful negatives. Fall-like
clips must be safe staged demonstrations onto a mattress, never real falls.
Give every person a different `--person` id so the subject-level split works.

## 6. Model cards

Each trained model folder contains `model_card.json` with: model type,
hyper-parameters, training data (datasets, subjects, sample counts), feature
list, label mapping, evaluation results, software versions and limitations.
