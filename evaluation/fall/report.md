# Fall detection evaluation (event level)

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
