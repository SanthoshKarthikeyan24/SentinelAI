# Posture classifier evaluation

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
