# SentinelAI - measured results

Generated 2026-10-29T19:27:11+00:00 by `training/evaluate_models.py`.
All numbers below were produced by scripts in this repository; nothing is estimated by hand.

## Pose model coverage on GMDCSA-24

- Sampled frames: 19178 from 159 videos
- Frames with at least one detected person: 92.7%
- Labelled person: mean visible keypoints 15.14 / 17; frames with >= 7 keypoints: 99.0%
- Videos where a person was found in < 80% of frames: 25
- No keypoint ground truth exists for GMDCSA-24, so this is detection coverage, not keypoint accuracy.

## Posture classifier

Protocol: leave-one-subject-out over 4 subjects, 10130 labelled frames from 145 videos.

| Model | Accuracy | Macro-F1 | LYING F1 | SITTING F1 | STANDING F1 |
|---|---|---|---|---|---|
| Rule baseline | 0.799 | 0.833 | 0.849 | 0.828 | 0.823 |
| Random Forest | 0.976 | 0.965 | 0.936 | 0.979 | 0.979 |
| Hist Gradient Boosting | 0.970 | 0.949 | 0.898 | 0.972 | 0.978 |

Selected model: **Random Forest** (retrained on all subjects).

UNKNOWN threshold (selected model, out-of-fold):

| threshold | frames kept | accuracy on kept |
|---|---|---|
| 0.0 | 100.0% | 0.9762 |
| 0.4 | 100.0% | 0.9766 |
| 0.5 | 99.7% | 0.978 |
| 0.6 | 97.6% | 0.9853 |
| 0.7 | 95.8% | 0.9895 |
| 0.8 | 93.3% | 0.9942 |


## Fall detection

Protocol: nested leave-one-subject-out. Every fall/ADL video is evaluated by a system whose models and threshold were chosen without that person.

| System | Alarm level | Precision | Recall | F1 | TP | FN | FP | False alarms / hour | Median delay (s) |
|---|---|---|---|---|---|---|---|---|---|
| Trained | FALL_SUSPECTED | 0.859 | 0.924 | 0.890 | 73 | 6 | 12 | 44.19 | 0.928 |
| Trained | FALL_CONFIRMED | 0.871 | 0.772 | 0.819 | 61 | 18 | 9 | 33.14 | 2.548 |
| Rule baseline | FALL_SUSPECTED | 0.753 | 0.848 | 0.798 | 67 | 12 | 22 | 81.02 | 0.916 |
| Rule baseline | FALL_CONFIRMED | 0.838 | 0.722 | 0.775 | 57 | 22 | 11 | 40.51 | 2.49 |

Window-level (out-of-fold): ROC-AUC 0.9838, average precision 0.8669 on 15258 windows.

FALL_CONFIRMED needs the person to stay down; only 26 labelled falls continue for at least 4.4 s after onset. Confirmed recall on those: 0.8077.

| Test subject | Selected model | Threshold | Trained F1 (suspected) | Baseline F1 (suspected) |
|---|---|---|---|---|
| S1 | Hist Gradient Boosting | 0.8 | 1.000 | 0.909 |
| S2 | Hist Gradient Boosting | 0.8 | 0.862 | 0.737 |
| S3 | Random Forest | 0.4 | 0.857 | 0.762 |
| S4 | Random Forest | 0.6 | 0.875 | 0.833 |

Deployed model: Hist Gradient Boosting, threshold 0.8.

Limitations:

- Staged falls by 4 young adult actors; real falls of elderly people can be slower or partial.
- Videos are short (2-70 s); false alarms per hour are extrapolated from about 16 minutes of non-fall video.
- Many clips end 1-3 s after the fall, so FALL_CONFIRMED (needs the person to stay down) is limited by clip length.
- Camera heights 64-135 cm; a floor-level robot camera sees people from a different angle.
- Falls onto a bed are labelled as falls in this dataset.


## Speed on the build machine

- CPU: x86_64; ONNX Runtime providers: ['CPUExecutionProvider']
- Pose model: 53.5 ms/frame (p95 65.9)
- Object model: 50.1 ms/frame (p95 66.9)
- posture model ms per call: 9.76 ms
- fall model ms per call: 2.33 ms

Your laptop's speed will differ; the dashboard shows live FPS and inference time.

## Software

```
{
  "python": "3.13.18",
  "platform": "Linux-6.18.44-fc-v185-x86_64-with-glibc2.39",
  "numpy": "2.5.3",
  "scipy": "1.18.1",
  "sklearn": "1.9.1",
  "cv2": "5.0.0",
  "onnxruntime": "1.29.0",
  "pandas": "3.0.5",
  "joblib": "1.6.0"
}
```
