# Error analysis

Generated 2026-10-30T09:44:53+00:00 by `training/error_report.py`. Posture: out-of-fold predictions of the deployed model type (Random Forest on keypoint features), leave-one-subject-out over S1-S4. Fall: errors of the deployed fall model type and threshold under leave-one-subject-out (`evaluation/fall/error_analysis.csv`).

## 1. Posture errors

222 wrong of 10130 labelled frames (2.2 %).

All frames:

| true \ predicted | LYING | SITTING | STANDING |
|---|---|---|---|
| LYING | 592 | 30 | 10 |
| SITTING | 38 | 5405 | 91 |
| STANDING | 0 | 53 | 3911 |

Most common mistakes:

| true | predicted | frames | share of all errors |
|---|---|---|---|
| SITTING | STANDING | 91 | 41 % |
| STANDING | SITTING | 53 | 24 % |
| SITTING | LYING | 38 | 17 % |
| LYING | SITTING | 30 | 14 % |
| LYING | STANDING | 10 | 5 % |

### By lighting (dataset 'Time of Recording')

| lighting | frames | errors | error rate | LYING frames | LYING recall |
|---|---|---|---|---|---|
| day_dim | 1487 | 7 | 0.5 % | 100 | 0.930 |
| day_lit | 6216 | 130 | 2.1 % | 513 | 0.936 |
| night | 2427 | 85 | 3.5 % | 19 | 1.000 |

Night frames only:

| true \ predicted | LYING | SITTING | STANDING |
|---|---|---|---|
| LYING | 19 | 0 | 0 |
| SITTING | 37 | 1475 | 26 |
| STANDING | 0 | 22 | 848 |

### By number of visible keypoints

| visible keypoints | frames | error rate |
|---|---|---|
| 7-10 | 49 | 6.1 % |
| 11-13 | 985 | 5.3 % |
| 14-15 | 2759 | 2.7 % |
| 16-17 | 6337 | 1.5 % |

### By subject (each tested by a model trained on the other three)

| subject | frames | error rate |
|---|---|---|
| S1 | 1516 | 1.3 % |
| S2 | 3238 | 4.5 % |
| S3 | 2800 | 1.6 % |
| S4 | 2576 | 0.4 % |

### Videos with the most errors

| video | errors | frames | lighting | description |
|---|---|---|---|---|
| gmdcsa24_S2_ADL_02 | 28 | 162 | day_lit | Picking up a water bottle from the ground and drinking while sitting on the bed |
| gmdcsa24_S2_ADL_03 | 25 | 130 | day_lit | Walking; removing dust from the bed using a pillow; and sleeping on the bed |
| gmdcsa24_S2_ADL_14 | 24 | 143 | night | Walking and reading something while sitting on the bed |
| gmdcsa24_S2_ADL_15 | 13 | 155 | night | Reading a newspaper while sitting on the bed and placing a pillow on the lap |
| gmdcsa24_S2_Fall_24 | 11 | 37 | day_lit | Standing from the bed and then falling sideways on the ground |
| gmdcsa24_S3_ADL_15 | 11 | 101 | night | Reading newspaper while sitting on the floor |
| gmdcsa24_S2_ADL_12 | 10 | 57 | day_lit | Going to bed and sleeping (belly facing downwards) |
| gmdcsa24_S1_Fall_03 | 9 | 35 | day_lit | Falling from sitting on the chair to the ground. Left side fall |

Example frames (red box = tracked person, yellow = visible keypoints):

- `reports/error_examples/gmdcsa24_S2_ADL_02_6.7s.jpg` – true SITTING, predicted STANDING (28 wrong frames in this video)
- `reports/error_examples/gmdcsa24_S2_ADL_03_8.8s.jpg` – true LYING, predicted SITTING (25 wrong frames in this video)
- `reports/error_examples/gmdcsa24_S2_ADL_14_8.1s.jpg` – true SITTING, predicted LYING (24 wrong frames in this video)
- `reports/error_examples/gmdcsa24_S2_ADL_15_3.9s.jpg` – true SITTING, predicted LYING (13 wrong frames in this video)
- `reports/error_examples/gmdcsa24_S2_Fall_24_2.7s.jpg` – true STANDING, predicted SITTING (11 wrong frames in this video)
- `reports/error_examples/gmdcsa24_S3_ADL_15_5.4s.jpg` – true SITTING, predicted LYING (11 wrong frames in this video)

## 2. Fall detection errors (deployed model type and threshold, leave-one-subject-out)

This list comes from `training/analyse_errors.py`, which uses the deployed model type and threshold in every fold. The headline numbers in evaluation/SUMMARY.md use *nested* selection (each fold chooses its own model and threshold), so the counts can differ by one or two.

| error | level | count |
|---|---|---|
| FALSE_ALARM | FALL_CONFIRMED | 10 |
| FALSE_ALARM | FALL_SUSPECTED | 13 |
| MISSED | FALL_CONFIRMED | 16 |
| MISSED | FALL_SUSPECTED | 5 |

Grouped by what the video description says (descriptions come from the dataset authors):

| error | level | theme | count |
|---|---|---|---|
| FALSE_ALARM | FALL_CONFIRMED | exercise | 3 |
| FALSE_ALARM | FALL_CONFIRMED | from / near a chair | 5 |
| FALSE_ALARM | FALL_CONFIRMED | on / onto a bed | 2 |
| FALSE_ALARM | FALL_SUSPECTED | exercise | 4 |
| FALSE_ALARM | FALL_SUSPECTED | from / near a chair | 5 |
| FALSE_ALARM | FALL_SUSPECTED | on / onto a bed | 4 |
| MISSED | FALL_CONFIRMED | body partly out of view | 2 |
| MISSED | FALL_CONFIRMED | from / near a chair | 1 |
| MISSED | FALL_CONFIRMED | on / onto a bed | 2 |
| MISSED | FALL_CONFIRMED | other | 11 |
| MISSED | FALL_SUSPECTED | from / near a chair | 1 |
| MISSED | FALL_SUSPECTED | other | 4 |

Every missed FALL_SUSPECTED (the first alarm level):

| video | description |
|---|---|
| gmdcsa24_S2_Fall_10 | Right side fall then backward |
| gmdcsa24_S3_Fall_14 | Falling backward on the floor while removing the shirt and trying to hang it on the hangar |
| gmdcsa24_S3_Fall_15 | Trying to drink water from the water bottle and then falling on the floor |
| gmdcsa24_S3_Fall_16 | Walking and then falling left side on the floor |
| gmdcsa24_S4_Fall_10 | Reading a book while sitting on the chair then falling (FW) on the ground |

Every FALL_SUSPECTED false alarm:

| video | time (s) | description |
|---|---|---|
| gmdcsa24_S2_ADL_08 | 7.1 | Exercising (Stretching thigh and trying to touch the ground:Pyramid Pose) |
| gmdcsa24_S2_ADL_09 | 4.3 | Standing up from a sitting position on the bed and then sitting on the ground |
| gmdcsa24_S2_ADL_12 | 3.9 | Going to bed and sleeping (belly facing downwards) |
| gmdcsa24_S2_ADL_13 | 1.7 | Sitting on the ground and reading a book |
| gmdcsa24_S2_ADL_19 | 1.9 | Sitting on the ground and writing in a notebook |
| gmdcsa24_S2_ADL_20 | 4.6 | Standing up; picking up a water bottle; and drinking water while sitting on the ground |
| gmdcsa24_S2_ADL_22 | 1.4 | Exercising (Push-up) and sitting |
| gmdcsa24_S3_ADL_04 | 1.1 | Picking a book from the floor and putting it on the bed; sitting on the bed |
| gmdcsa24_S3_ADL_05 | 2.5 | Exercising (hand down & up with body bending) |
| gmdcsa24_S3_ADL_08 | 2.4 | Sitting to sleeping on the floor |
| gmdcsa24_S4_ADL_05 | 2.9 | Doing exercise (push-up); view 1 |
| gmdcsa24_S4_ADL_06 | 2.3 | Doing exercise (push-up); view 2 |
| gmdcsa24_S4_ADL_10 | 6.6 | Walking then sleeping on the bed after removing the goggles |

## 3. Decisions taken from the experiments (reports/experiments.md)

Rule: change a deployed setting only if the development subjects (S1-S3) improve by more than the noise level (about 1 point macro-F1) **and** the held-out subject S4 does not get worse.

- **keypoint_conf**: best on S1-S3 is 0.5 (+0.12 points), S4 0.9921 vs 0.9922 -> **kept at 0.35** (difference within noise).
- **augmentation**: no variant beats 'none' on S1-S3 (0.959); kept none.
- **class imbalance**: best on S1-S3 is 'balanced_subsample (current)' -> kept (already deployed).
- **posture smoothing**: window 1 gives the best frame accuracy (0.9781) but 11.63 label changes/min (ground truth: 2.97); window 5 gives 0.9733 with 5.12 changes/min. **Kept 5**: a steadier posture matters more to the fall state machine than half a point of frame accuracy.
- **keypoint vs image classifier**: keypoint RF 0.9744 vs image HOG 0.7473 macro-F1 (same frames) -> **kept the keypoint classifier**.
- **camera resolution**: 1280x720: person found 100.0 %, posture F1 0.9756; 640x360: person found 100.0 %, posture F1 0.9756; 320x180: person found 99.8 %, posture F1 0.9684. Recommendation: set IP Webcam to at least 640×480; 1280×720 if the laptop keeps up.
- **lighting**: night posture macro-F1 0.817 vs daylight 0.9754 -> no setting change; the fix is more night-time training data (record your own with training/collect_data.py, with consent).

## 4. What would most improve accuracy

1. Recordings from the robot's own camera height in the target home (with consent), labelled with training/annotate_videos.py - the biggest gap between the dataset and real use.
2. More night / dim-light examples, especially lying on a bed and on the floor.
3. Longer clips that continue after the fall, so FALL_CONFIRMED can be measured properly.
4. A second public dataset (UR Fall, Le2i, CAUCAFall) when it can be downloaded, as an external test.
