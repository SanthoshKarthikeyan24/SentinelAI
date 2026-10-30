# SentinelAI - AI experiments (A-H)

Generated 2026-10-30T09:43:42+00:00 by `training/experiments.py`. Data: GMDCSA-24 (4 subjects). Settings are chosen on the development subjects S1-S3 (leave-one-subject-out inside them); S4 is a held-out check that was not used for any choice. Posture classes: LYING, SITTING, STANDING (frame level).

**How to read this:** frames from one video are strongly correlated, so differences below about one percentage point are within noise. No experiment here is evidence about elderly people or real falls.

## A - camera resolution

the ONNX model input stays 640x640 (fixed export); this measures the effect of the CAMERA resolution (e.g. IP Webcam 640x480 or lower) on detection and posture.

| camera input | person found | mean visible keypoints | posture usable | posture macro-F1 | pose ms |
|---|---|---|---|---|---|
| 1280x720 | 100.0% | 15.67 | 100.0% | 0.9756 | 69.8 |
| 640x360 | 100.0% | 15.67 | 100.0% | 0.9756 | 70.3 |
| 320x180 | 99.8% | 15.59 | 99.8% | 0.9684 | 70.5 |

Videos: gmdcsa24_S1_ADL_05, gmdcsa24_S1_ADL_11, gmdcsa24_S1_ADL_07, gmdcsa24_S2_ADL_02, gmdcsa24_S2_ADL_18, gmdcsa24_S2_ADL_15, gmdcsa24_S3_ADL_17, gmdcsa24_S3_ADL_10, gmdcsa24_S3_ADL_11, gmdcsa24_S4_ADL_04, gmdcsa24_S4_ADL_01, gmdcsa24_S4_ADL_09 (every 4th pose frame).

## B - keypoint classifier vs image-based classifier

Same 3377 frames (every 3rd labelled frame). Image model: HOG descriptor of a 96x96 crop around the person box + logistic regression.

| model | dev S1-S3 macro-F1 | held-out S4 macro-F1 | all-LOSO macro-F1 | LYING F1 (all) |
|---|---|---|---|---|
| Keypoint features + Random Forest (deployed type) | 0.9632 | 0.9891 | 0.9744 | 0.9569 |
| Image HOG + logistic regression | 0.7244 | 0.5305 | 0.7473 | 0.4912 |

Image model cost per person: HOG 0.182 ms + classifier 0.233 ms (on top of the person detector).

## C - keypoint confidence threshold (chosen on S1-S3)

| keypoint_conf | labelled frames kept | dev macro-F1 | S4 macro-F1 |
|---|---|---|---|
| 0.2 | 10139 | 0.9585 | 0.9905 |
| 0.3 | 10132 | 0.9588 | 0.9932 |
| 0.35 | 10130 | 0.959 | 0.9922 |
| 0.4 | 10123 | 0.9593 | 0.9913 |
| 0.5 | 10110 | 0.9602 | 0.9921 |
| 0.6 | 10100 | 0.9589 | 0.9872 |

Chosen on development subjects: **0.5** (current setting 0.35). frames = labelled frames with enough visible keypoints; a stricter threshold drops frames.

## D - training augmentation

| augmentation | dev macro-F1 | dev LYING F1 | S4 macro-F1 |
|---|---|---|---|
| none | 0.959 | 0.9374 | 0.9922 |
| mirror | 0.9583 | 0.9395 | 0.9879 |
| jitter+dropout | 0.956 | 0.9421 | 0.9916 |
| mirror+jitter+dropout | 0.9543 | 0.937 | 0.9874 |

Occlusion test (3 visible keypoints removed from every test frame, all-LOSO):

| trained with | macro-F1 | LYING F1 |
|---|---|---|
| none | 0.9016 | 0.8074 |
| mirror+jitter+dropout | 0.9603 | 0.9423 |

## E - class imbalance

Class counts: {'SITTING': 5534, 'LYING': 632, 'STANDING': 3964}

| method | dev macro-F1 | dev LYING F1 | S4 macro-F1 | S4 LYING F1 |
|---|---|---|---|---|
| no weighting | 0.9488 | 0.9144 | 0.9903 | 0.982 |
| balanced_subsample (current) | 0.959 | 0.9374 | 0.9922 | 0.982 |
| oversample minority | 0.9542 | 0.9287 | 0.984 | 0.982 |

## F - fine-tuning the pose model

Not run. Reasons: (1) GMDCSA-24 has no keypoint ground truth, so a fine-tuned pose model could not be measured; (2) the pose model already finds the labelled person in 92.7% of frames with 15.1/17 keypoints on average (evaluation/pose/pose_coverage.json); (3) training YOLO needs the Ultralytics package and a GPU, neither available in this build environment. Fine-tuning would be justified once robot-height recordings with keypoint labels exist.

## G - temporal settings

Posture smoothing (causal majority vote over the last N pose frames, 15 pose frames/s; ground truth changes 2.97 times/min):

| window N | accuracy | macro-F1 | label changes / min | added delay (s) |
|---|---|---|---|---|
| 1 | 0.9781 | 0.9666 | 11.63 | 0.0 |
| 3 | 0.976 | 0.9631 | 5.94 | 0.067 |
| 5 | 0.9733 | 0.9591 | 5.12 | 0.133 |
| 9 | 0.9672 | 0.9487 | 4.62 | 0.267 |
| 15 | 0.9593 | 0.9381 | 4.13 | 0.467 |

Current smoothing_window: 5.

Fall timing (window length, time down before FALL_SUSPECTED) was tuned on ['S1', 'S2', 'S3'] with S4 held out (8 settings; evaluation/fall/tuning.json). Best on development subjects: {"window_s": 1.5, "features": "extended", "suspect_min_down_s": 0.4, "model": "rf", "threshold": 0.6, "dev_f1": 0.9291, "dev_precision": 0.9219, "dev_recall": 0.9365}.

## H - environment (lighting)

lighting from the dataset's 'Time of Recording' column; groups are small - differences of a few points are not significant.

| lighting | videos | posture frames | posture macro-F1 | accuracy |
|---|---|---|---|---|
| day_dim | 16 | 1487 | 0.9869 | 0.9953 |
| day_lit | 100 | 6216 | 0.9754 | 0.9791 |
| night | 29 | 2427 | 0.817 | 0.965 |

Fall detection (deployed system, nested LOSO):

| lighting | videos | falls | minutes | SUSPECTED recall | SUSPECTED false alarms | CONFIRMED recall |
|---|---|---|---|---|---|---|
| day_dim | 18 | 6 | 2.6 | 1.0 | 3 | 0.833 |
| day_lit | 112 | 61 | 14.0 | 0.967 | 7 | 0.82 |
| night | 29 | 12 | 4.7 | 0.75 | 3 | 0.667 |

## What changed in the deployed system because of these experiments

See the 'Decisions' section in reports/error_analysis.md. Settings are only changed when the development-subject result improves and the held-out check does not get worse.
