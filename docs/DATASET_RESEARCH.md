# Dataset research

Status key: **USED** = downloaded, inspected and used for training in this project;
**NOT USED** = researched only. Facts marked *(reported)* come from the dataset
authors' publications/pages and were **not** re-verified by downloading the data.

## 1. Datasets per task

| Task | Labels needed | Dataset(s) | Status |
|---|---|---|---|
| Person + object detection | boxes + classes | COCO 2017 (via pretrained YOLO11n) | pretrained model only |
| Human pose estimation | 17 body keypoints | COCO-Keypoints (via pretrained YOLO11n-pose) | pretrained model only |
| Posture classification | posture per frame/interval | GMDCSA-24 intervals (Standing, Walking, Sitting, Sleeping) | **USED** |
| Fall-event detection | fall start/end in time | GMDCSA-24 fall intervals | **USED** |
| Ordinary activities (hard negatives) | ADL intervals | GMDCSA-24 ADL videos | **USED** |
| BENDING / KNEELING / floor-lying | intervals | none suitable found that is downloadable here; collect your own (see TRAINING.md) | pending |

An image dataset (e.g. COCO) can teach "where is the person" but cannot teach
"did this person fall 2 seconds ago" — fall detection needs **video with time
labels**, which is why GMDCSA-24 was chosen for training.

## 2. GMDCSA-24 — USED

* **Official source:** https://github.com/ekramalam/GMDCSA24-A-Dataset-for-Human-Fall-Detection-in-Videos
  (archived on Zenodo, DOI 10.5281/zenodo.12921216)
* **Paper:** E. Alam, A. Sufian, P. Dutta, M. Leo, I. A. Hameed, "GMDCSA-24: A dataset for human
  fall detection in videos", *Data in Brief*, 2024. https://pmc.ncbi.nlm.nih.gov/articles/PMC11416611/
* **Licence:** MIT (LICENSE file in the repository, checked). Cite the paper and the Zenodo record.
* **Access:** public, no registration. Downloaded with `python training/download_datasets.py`.
* **What we verified by inspecting the files:**
  * 160 MP4 videos, 4 subjects (S1–S4), ADL and Fall folders per subject, 21.5 minutes total.
  * Resolutions 1280×720 (142 videos) and 640×480 (18); frame rates 15 or ~30 fps.
  * Annotation CSV per subject and category with columns *File Name, Length, Time of
    Recording, Attire, Description, Classes*. `Classes` holds time intervals, e.g.
    `Falling (SW)[3.4 to 6]; Sitting[0 to 3.4]`.
  * Labels found: Falling (FW/BW/SW), Standing, Walking, Sitting, Sleeping, Exercising,
    Drinking, Eating, Reading, Writing.
* **Annotation problems found and how they were handled** (all logged in
  `data/annotations/prepare_report.json`, nothing silently dropped):

  | Video | Problem | Action |
  |---|---|---|
  | S4 Fall 15 | `Falling (FW);` has no time interval | **video excluded** (otherwise a real fall would become "no fall" data) |
  | S2 Fall 19, S4 Fall 09 | interval written as `[0.9 4.6]` without "to" | parsed as interval, noted |
  | S3 ADL 04 | missing `]` between intervals | repaired, noted |
  | S2 ADL 06 | `Sitting [9.7 to 3.0]` (end before start) | interval ignored, noted |
  | S3 Fall 16 | interval ends after the video | clipped to video length, noted |

  Result: **159 videos, 79 fall events, 356 labelled intervals** used.
* **Limitations:** 4 young adult actors; staged falls (several onto a bed); camera 64–135 cm high
  (a robot camera may be lower); many fall clips end 1–3 s after the fall.

## 3. Other datasets researched — NOT USED

| Dataset | Content *(reported)* | Official source | Why not used here |
|---|---|---|---|
| UR Fall Detection (Kwolek & Kępski, 2014) | 70 sequences (30 falls, 40 ADL), Kinect depth + RGB, per-frame labels | http://fenix.ur.edu.pl/~mkepski/ds/uf.html | host not reachable from the build environment; licence terms must be read on the page before use |
| Le2i Fall Detection | ~190 videos in 4 rooms (home, coffee room, office, lecture room), fall start/end frames | ImViA lab, Université de Bourgogne (https://imvia.ube.fr/?p=346) | host not reachable; access conditions to be checked on the page |
| CAUCAFall | 10 subjects, 5 fall types + 5 ADL (incl. sitting, kneeling, picking up), uncontrolled home | Mendeley Data, https://data.mendeley.com/datasets/7w7fccy7ky/4 | host not reachable; would add KNEELING/BENDING-like activities |
| UP-Fall (HAR-UP) | 17 subjects, 11 activities incl. 5 fall types, cameras + wearables | https://sites.google.com/up.edu.mx/har-up/ | Google Drive downloads not reachable |
| COCO 2017 Keypoints | ~250k people with 17 keypoints | https://cocodataset.org | used only through the pretrained pose model; no fine-tuning needed (see below) |

To add one of these later: download it manually following its page, accept its
licence, place it under `data/raw/public_datasets/<NAME>/`, write a converter
that outputs the standard interval format (see `data/README.md`), then re-run
the pipeline in `docs/TRAINING.md`. Do **not** claim results on a dataset until
this has been done.

## 4. Do we need to fine-tune the detector or pose model?

* **Objects:** the pretrained COCO model already includes `chair`, `couch`, `bed`,
  `dining table`, `toilet`, `tv`, `laptop`, `bottle`, `cup`. On a dataset frame it found
  the person (0.89) and the bed (0.64). No project-specific object class is missing, so
  **no fine-tuning was done** (as the brief recommends). No object-detection mAP was
  measured on our data because we have no box labels for it; Ultralytics publishes
  COCO mAP50-95 ≈ 39.5 for YOLO11n *(reported)*.
* **Pose:** YOLO11n-pose found a person in 92.7 % of the 19,178 sampled dataset frames, and
  in 99.0 % of the frames where it found the labelled person at least 7 of 17 keypoints were
  reliable (mean 15.1). 25 of 159 videos had coverage below 80 %; these include clips where the
  person falls partly out of the picture or is in a dark room (not separately counted).
  See `evaluation/pose/pose_coverage.json`. The trained classifiers reach the accuracy reported
  in `evaluation/` with these keypoints, so **no pose fine-tuning was done**. GMDCSA-24 has no
  keypoint labels, so keypoint accuracy itself could not be measured here; Ultralytics reports
  COCO pose mAP50-95 ≈ 50.0 for YOLO11n-pose *(reported)*.

| Model | Pretrained | Fine-tuned | Trained from scratch |
|---|---|---|---|
| YOLO11n (objects) | yes (Ultralytics, COCO) | no | no |
| YOLO11n-pose | yes (Ultralytics, COCO-Keypoints) | no | no |
| Posture classifier | – | – | **yes** (this project, GMDCSA-24) |
| Fall-window classifier | – | – | **yes** (this project, GMDCSA-24) |
