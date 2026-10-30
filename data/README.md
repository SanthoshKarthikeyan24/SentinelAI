# Data folder

```text
data/
├── raw/
│   ├── public_datasets/GMDCSA24/   downloaded by training/download_datasets.py (not edited)
│   └── collected_videos/           your own clips (training/collect_data.py) + .json sidecars
├── annotations/
│   ├── video_index.csv             one row per usable video (generated)
│   ├── prepare_report.json         every repaired / excluded annotation (generated)
│   ├── video_events/events.csv     all labelled intervals in the standard format (generated)
│   └── video_events/collected_events.csv  your interval labels (training/annotate_videos.py)
├── processed/
│   ├── pose_features/*.npz         keypoints + track ids per video (training/extract_pose_features.py)
│   └── temporal_sequences/         cached frame/window features (regenerated automatically)
├── splits/                         subject-level splits (training/split_dataset.py)
├── dataset_inventory.csv           one row per video: lighting, labels, pose coverage, split, licence
│                                   (training/build_inventory.py)
└── dataset_inventory_summary.json  totals by subject / category / lighting
```

## Standard annotation format

`video_index.csv`: `video_id, dataset, path, subject, session, category, duration_s, fps, width, height, description`

`video_events/events.csv`: `video_id, label, start_s, end_s, raw_label, person_id, uncertainty, notes`

Labels: `STANDING, WALKING, SITTING, BENDING, KNEELING, LYING, FALL (= FALL_LIKE_EVENT), RECOVERY, OCCLUDED, UNKNOWN`
and overlay activities `ACT_*` (e.g. `ACT_EXERCISING`) that are not postures.

A fall is labelled as an **interval** from the loss of balance to the end of the
clip/rest, never as "the whole video is a fall".

## Privacy

Recordings of people are personal data. Only record with consent, use
pseudonyms (P01…), keep raw videos out of shared folders/Git (see `.gitignore`),
and delete them when the project ends.
