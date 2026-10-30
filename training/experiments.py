"""Controlled AI experiments (master prompt Part 9, experiments A-H) on GMDCSA-24.

Every number is produced here, from the cached pose files and the videos; nothing is typed in.
Protocol: leave-one-subject-out (LOSO) unless stated. Settings are CHOSEN on the development
subjects S1-S3 only (LOSO inside them); S4 is reported afterwards as a held-out check.

  A  camera resolution   pose model on frames downscaled to 1/1, 1/2, 1/4 (subset of videos)
  B  model selection     keypoint Random Forest vs an image-based classifier (HOG + linear model)
  C  thresholds          keypoint confidence threshold chosen on S1-S3, checked on S4
  D  augmentation        mirrored / jittered / keypoint-dropout training copies
  E  class imbalance     no weighting / balanced weights / oversampling the rare LYING class
  F  fine-tuning         not run - reasons written into the report
  G  temporal            posture smoothing window (accuracy vs stability); fall timing from tuning.json
  H  environment         results per lighting group (from the dataset's "Time of Recording")

Run:  python training/experiments.py [--only A B ...] [--quick]
Output: evaluation/experiments/<id>.json and reports/experiments.md
"""
from __future__ import annotations

import argparse
import csv
import json
import time
from collections import Counter, defaultdict

import cv2
import numpy as np
from sklearn.ensemble import RandomForestClassifier
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import accuracy_score, f1_score
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler

from common import DATA, EVALUATION, ROOT, get_logger, now_iso, software_versions, write_json
from dataset_lib import interval_label_at, load_events, load_index, load_pose
from src.analysis.features import FRAME_FEATURES, frame_features, visible_keypoints
from src.detection.yolo_onnx import KEYPOINT_NAMES
from src.config import load_config

log = get_logger("experiments")
OUT = EVALUATION / "experiments"
REPORT = ROOT / "reports" / "experiments.md"
SEED = 42
DEV = ["S1", "S2", "S3"]
HELD_OUT = "S4"
LABELS = ["LYING", "SITTING", "STANDING"]
FLIP = [KEYPOINT_NAMES.index(n.replace("left_", "TMP").replace("right_", "left_").replace("TMP", "right_"))
        for n in KEYPOINT_NAMES]


def rf(class_weight="balanced_subsample"):
    return RandomForestClassifier(n_estimators=200, min_samples_leaf=3, max_features="sqrt",
                                  class_weight=class_weight, n_jobs=-1, random_state=SEED)


# ----------------------------------------------------------------- samples
def collect(kpt_thr: float, min_visible: int):
    """Frame samples of the labelled (primary) person, with raw keypoints for augmentation."""
    index = {v["video_id"]: v for v in load_index()}
    events = load_events()
    S = defaultdict(list)
    for vid, v in index.items():
        try:
            pose = load_pose(vid)
        except FileNotFoundError:
            continue
        tids = pose["track_id"]
        if len(tids) == 0:
            continue
        vals, counts = np.unique(tids, return_counts=True)
        primary = vals[counts.argmax()]
        for i in np.where(tids == primary)[0]:
            t = float(pose["t"][i])
            lab = interval_label_at(events.get(vid, []), t)
            if lab not in LABELS:
                continue
            k, box = pose["kpts"][i], pose["box"][i]
            if visible_keypoints(k, kpt_thr) < min_visible:
                continue
            f = frame_features(k, box, kpt_thr)
            S["X"].append([f[n] for n in FRAME_FEATURES])
            S["y"].append(lab)
            S["g"].append(v["subject"])
            S["vid"].append(vid)
            S["t"].append(t)
            S["frame"].append(int(pose["frame"][i]))
            S["kpts"].append(k)
            S["box"].append(box)
    out = {k: np.array(v) for k, v in S.items()}
    out["X"] = out["X"].astype(np.float64)
    return out


def subset(S, mask):
    return {k: v[mask] for k, v in S.items()}


def scores(y, p) -> dict:
    """Accuracy, macro-F1 over the classes present in the ground truth, per-class F1 (None if absent)."""
    y, p = np.asarray(y, dtype=object), np.asarray(p, dtype=object)
    present = [l for l in LABELS if (y == l).any()]
    per = f1_score(y, p, labels=LABELS, average=None, zero_division=0)
    return {"accuracy": round(float(accuracy_score(y, p)), 4),
            "macro_f1": round(float(f1_score(y, p, labels=present, average="macro", zero_division=0)), 4),
            **{f"f1_{l}": (round(float(x), 4) if l in present else None) for l, x in zip(LABELS, per)},
            "classes_present": present, "n": int(len(y))}


def loso(S, subjects, fit_predict):
    """Pooled out-of-fold predictions over the given subjects."""
    pred = np.empty(len(S["y"]), dtype=object)
    used = np.zeros(len(S["y"]), bool)
    for s in subjects:
        te = S["g"] == s
        tr = np.isin(S["g"], subjects) & ~te
        if te.sum() == 0 or tr.sum() == 0:
            continue
        pred[te] = fit_predict(subset(S, tr), subset(S, te))
        used |= te
    return pred, used


def rf_fit_predict(class_weight="balanced_subsample", augment=None, oversample=False, seed=SEED):
    def fp(tr, te):
        X, y = tr["X"], tr["y"]
        if augment is not None:
            Xa, ya = augment(tr, np.random.default_rng(seed))
            X, y = np.vstack([X, Xa]), np.concatenate([y, ya])
        if oversample:
            rng = np.random.default_rng(seed)
            n_max = max(Counter(y).values())
            idx = np.concatenate([rng.choice(np.where(y == c)[0], n_max, replace=True) if (y == c).sum() < n_max
                                  else np.where(y == c)[0] for c in LABELS if (y == c).any()])
            X, y = X[idx], y[idx]
        m = rf(class_weight).fit(np.nan_to_num(X, nan=-9.0), y)
        return m.predict(np.nan_to_num(te["X"], nan=-9.0))
    return fp


def dev_and_heldout(S, fit_predict) -> dict:
    p_dev, u_dev = loso(S, DEV, fit_predict)
    p_all, u_all = loso(S, DEV + [HELD_OUT], fit_predict)
    s4 = S["g"] == HELD_OUT
    return {"dev_loso_S1-S3": scores(S["y"][u_dev], p_dev[u_dev]),
            "heldout_S4": scores(S["y"][s4], p_all[s4]),
            "all_loso": scores(S["y"][u_all], p_all[u_all])}


# --------------------------------------------------------------- experiments
def exp_C(cfg, quick):
    """Keypoint confidence threshold: chosen on S1-S3, checked on S4."""
    res = []
    for thr in ([0.2, 0.35, 0.5] if quick else [0.2, 0.3, 0.35, 0.4, 0.5, 0.6]):
        S = collect(thr, cfg.posture.min_visible_keypoints)
        r = dev_and_heldout(S, rf_fit_predict())
        r.update(keypoint_conf=thr, frames=int(len(S["y"])))
        res.append(r)
        log.info("C kpt_thr %.2f: dev F1 %.4f  S4 F1 %.4f  frames %d", thr, r["dev_loso_S1-S3"]["macro_f1"],
                 r["heldout_S4"]["macro_f1"], r["frames"])
    best = max(res, key=lambda r: r["dev_loso_S1-S3"]["macro_f1"])
    return {"results": res, "chosen_on_dev": best["keypoint_conf"], "current_setting": cfg.detection.keypoint_conf,
            "note": "frames = labelled frames with enough visible keypoints; a stricter threshold drops frames"}


def _mirror(tr, rng, thr):
    out = []
    for k, b in zip(tr["kpts"], tr["box"]):
        k2 = k[FLIP].copy()
        k2[:, 0] = -k2[:, 0]
        b2 = np.array([-b[2], b[1], -b[0], b[3]], dtype=np.float32)
        f = frame_features(k2, b2, thr)
        out.append([f[n] for n in FRAME_FEATURES])
    return np.array(out, dtype=np.float64), tr["y"].copy()


def _jitter(tr, rng, thr, sigma=0.02, drop=2):
    out = []
    for k, b in zip(tr["kpts"], tr["box"]):
        k2 = k.copy()
        h = max(1.0, float(b[3] - b[1]))
        k2[:, :2] += rng.normal(0, sigma * h, size=(17, 2)).astype(np.float32)
        vis = np.where(k2[:, 2] >= thr)[0]
        if len(vis) > drop:
            k2[rng.choice(vis, drop, replace=False), 2] = 0.0
        f = frame_features(k2, b, thr)
        out.append([f[n] for n in FRAME_FEATURES])
    return np.array(out, dtype=np.float64), tr["y"].copy()


def exp_D(cfg, quick):
    thr = cfg.detection.keypoint_conf
    S = collect(thr, cfg.posture.min_visible_keypoints)
    variants = {
        "none": None,
        "mirror": lambda tr, rng: _mirror(tr, rng, thr),
        "jitter+dropout": lambda tr, rng: _jitter(tr, rng, thr),
        "mirror+jitter+dropout": lambda tr, rng: tuple(np.concatenate(z) for z in zip(_mirror(tr, rng, thr),
                                                                                   _jitter(tr, rng, thr))),
    }
    res = {}
    for name, aug in variants.items():
        res[name] = dev_and_heldout(S, rf_fit_predict(augment=aug))
        log.info("D %-22s dev F1 %.4f  S4 F1 %.4f", name, res[name]["dev_loso_S1-S3"]["macro_f1"],
                 res[name]["heldout_S4"]["macro_f1"])
    # robustness check: test frames with 2 keypoints removed (simulates occlusion)
    rob = {}
    rng = np.random.default_rng(1)
    Xd, _ = _jitter(S, rng, thr, sigma=0.0, drop=3)        # same frames, 3 visible keypoints removed
    for name in ("none", "mirror+jitter+dropout"):
        aug = variants[name]
        p, u = loso(S, DEV + [HELD_OUT], lambda tr, te, aug=aug: rf_fit_predict(augment=aug)(
            tr, dict(te, X=Xd[_rows(S, te)])))
        rob[name] = scores(S["y"][u], p[u])
    return {"results": res, "occlusion_test_3_keypoints_removed": rob}


def _rows(S, part):
    """Row numbers of `part` (a subset of S) inside S."""
    key = {(v, t): i for i, (v, t) in enumerate(zip(S["vid"], S["t"]))}
    return np.array([key[(v, t)] for v, t in zip(part["vid"], part["t"])])


def exp_E(cfg, quick):
    S = collect(cfg.detection.keypoint_conf, cfg.posture.min_visible_keypoints)
    res = {"class_counts": dict(Counter(S["y"].tolist()))}
    for name, fp in {"no weighting": rf_fit_predict(class_weight=None),
                     "balanced_subsample (current)": rf_fit_predict(),
                     "oversample minority": rf_fit_predict(class_weight=None, oversample=True)}.items():
        res[name] = dev_and_heldout(S, fp)
        log.info("E %-28s dev F1 %.4f LYING %.4f | S4 F1 %.4f", name, res[name]["dev_loso_S1-S3"]["macro_f1"],
                 res[name]["dev_loso_S1-S3"]["f1_LYING"], res[name]["heldout_S4"]["macro_f1"])
    return res


def exp_G(cfg, quick):
    S = collect(cfg.detection.keypoint_conf, cfg.posture.min_visible_keypoints)
    p, u = loso(S, DEV + [HELD_OUT], rf_fit_predict())
    res = []
    for n in (1, 3, 5, 9, 15):
        sm = p.copy()
        flips, minutes = 0, 0.0
        for vid in np.unique(S["vid"]):
            idx = np.where(S["vid"] == vid)[0]
            idx = idx[np.argsort(S["t"][idx])]
            seq = p[idx]
            out = []
            for j in range(len(seq)):
                w = seq[max(0, j - n + 1): j + 1]          # causal window, like the live app
                out.append(Counter(w).most_common(1)[0][0])
            sm[idx] = out
            flips += sum(1 for a, b in zip(out, out[1:]) if a != b)
            minutes += (S["t"][idx].max() - S["t"][idx].min()) / 60 if len(idx) > 1 else 0
        r = scores(S["y"], sm)
        r.update(window=n, label_changes_per_min=round(flips / max(minutes, 1e-9), 2),
                 delay_s_at_15fps=round((n - 1) / 2 / 15, 3))
        res.append(r)
        log.info("G smoothing %2d: acc %.4f F1 %.4f changes/min %.1f", n, r["accuracy"], r["macro_f1"],
                 r["label_changes_per_min"])
    true_changes = 0
    minutes = 0.0
    for vid in np.unique(S["vid"]):
        idx = np.where(S["vid"] == vid)[0]
        idx = idx[np.argsort(S["t"][idx])]
        y = S["y"][idx]
        true_changes += sum(1 for a, b in zip(y, y[1:]) if a != b)
        minutes += (S["t"][idx].max() - S["t"][idx].min()) / 60 if len(idx) > 1 else 0
    fall_tuning = {}
    tp = EVALUATION / "fall" / "tuning.json"
    if tp.exists():
        t = json.loads(tp.read_text(encoding="utf-8"))
        fall_tuning = {"development_subjects": t.get("development_subjects"), "held_out": t.get("held_out_subject"),
                       "best": t.get("best"), "grid_size": len(t.get("results", []))}
    return {"posture_smoothing": res, "ground_truth_label_changes_per_min": round(true_changes / max(minutes, 1e-9), 2),
            "current_setting": cfg.posture.smoothing_window, "fall_timing_tuning": fall_tuning}


def lighting_of() -> dict:
    out = {}
    p = DATA / "dataset_inventory.csv"
    if p.exists():
        with open(p, newline="", encoding="utf-8") as fh:
            for r in csv.DictReader(fh):
                out[r["video_id"]] = r.get("lighting_group", "unknown")
    return out


def exp_H(cfg, quick):
    S = collect(cfg.detection.keypoint_conf, cfg.posture.min_visible_keypoints)
    light = lighting_of()
    p, u = loso(S, DEV + [HELD_OUT], rf_fit_predict())
    groups = np.array([light.get(v, "unknown") for v in S["vid"]])
    posture = {}
    for g in sorted(set(groups)):
        m = groups == g
        posture[g] = scores(S["y"][m], p[m])
        posture[g]["videos"] = int(len(set(S["vid"][m])))
    # fall detection per lighting group from the nested-LOSO error list of the deployed system
    fall = {}
    err = EVALUATION / "fall" / "error_analysis.csv"
    if err.exists():
        rows = list(csv.DictReader(open(err, newline="", encoding="utf-8")))
        inv = list(csv.DictReader(open(DATA / "dataset_inventory.csv", newline="", encoding="utf-8")))
        for g in sorted({r["lighting_group"] for r in inv if r["status"] == "used"}):
            vids = {r["video_id"] for r in inv if r["lighting_group"] == g and r["status"] == "used"}
            n_falls = sum(int(r["fall_intervals"] or 0) for r in inv if r["video_id"] in vids)
            minutes = sum(float(r["duration_s"] or 0) for r in inv if r["video_id"] in vids) / 60
            d = {"videos": len(vids), "fall_events": n_falls, "minutes": round(minutes, 1)}
            for lvl in ("FALL_SUSPECTED", "FALL_CONFIRMED"):
                missed = sum(1 for r in rows if r["video_id"] in vids and r["error"] == "MISSED" and r["level"] == lvl)
                fa = sum(1 for r in rows if r["video_id"] in vids and r["error"] == "FALSE_ALARM" and r["level"] == lvl)
                d[lvl] = {"missed": missed, "recall": round(1 - missed / n_falls, 3) if n_falls else None,
                          "false_alarms": fa}
            fall[g] = d
    for g, r in posture.items():
        log.info("H %-8s posture F1 %.4f acc %.4f (%d frames, %d videos)", g, r["macro_f1"], r["accuracy"], r["n"],
                 r["videos"])
    return {"posture_by_lighting": posture, "fall_by_lighting": fall,
            "note": "lighting from the dataset's 'Time of Recording' column; groups are small - differences of a few "
                    "points are not significant"}


# ---------------------------------------------- B: image-based vs keypoint classifier
def _crop(img, box, size=96):
    x1, y1, x2, y2 = [float(v) for v in box]
    cx, cy, s = (x1 + x2) / 2, (y1 + y2) / 2, max(x2 - x1, y2 - y1) * 1.1
    M = np.float32([[size / s, 0, size / 2 - cx * size / s], [0, size / s, size / 2 - cy * size / s]])
    return cv2.warpAffine(img, M, (size, size), borderMode=cv2.BORDER_CONSTANT)


def exp_B(cfg, quick):
    S = collect(cfg.detection.keypoint_conf, cfg.posture.min_visible_keypoints)
    step = 6 if quick else 3
    keep = np.zeros(len(S["y"]), bool)
    keep[::step] = True
    S = subset(S, keep)
    index = {v["video_id"]: v for v in load_index()}
    hog = cv2.HOGDescriptor((96, 96), (16, 16), (8, 8), (8, 8), 9)
    feats = np.zeros((len(S["y"]), hog.getDescriptorSize()), np.float32)
    t0 = time.time()
    for vid in np.unique(S["vid"]):
        rows = np.where(S["vid"] == vid)[0]
        want = {int(S["frame"][i]): [] for i in rows}
        for i in rows:
            want[int(S["frame"][i])].append(i)
        cap = cv2.VideoCapture(str(ROOT / index[vid]["path"]))
        fno = 0
        last = max(want)
        while fno <= last:
            ok, img = cap.read()
            if not ok:
                break
            if fno in want:
                g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                for i in want[fno]:
                    feats[i] = hog.compute(_crop(g, S["box"][i])).ravel()
            fno += 1
        cap.release()
    log.info("B: HOG features for %d frames in %.0f s", len(S["y"]), time.time() - t0)
    S_img = dict(S, X=feats.astype(np.float64))

    def img_fp(tr, te):
        m = make_pipeline(StandardScaler(), LogisticRegression(C=0.05, max_iter=2000, class_weight="balanced"))
        return m.fit(tr["X"], tr["y"]).predict(te["X"])

    res = {"frames": int(len(S["y"])), "subsample_every": step,
           "keypoint_random_forest": dev_and_heldout(S, rf_fit_predict()),
           "image_hog_logistic": dev_and_heldout(S_img, img_fp)}
    # speed per sample
    m = make_pipeline(StandardScaler(), LogisticRegression(C=0.05, max_iter=2000, class_weight="balanced")).fit(
        S_img["X"], S["y"])
    t = time.perf_counter()
    for i in range(200):
        m.predict(S_img["X"][i % len(S["y"])][None])
    res["image_ms_per_sample_classifier_only"] = round((time.perf_counter() - t) / 200 * 1000, 3)
    g = cv2.cvtColor(np.zeros((720, 1280, 3), np.uint8), cv2.COLOR_BGR2GRAY)
    t = time.perf_counter()
    for i in range(200):
        hog.compute(_crop(g, S["box"][i % len(S["y"])]))
    res["image_ms_per_sample_hog"] = round((time.perf_counter() - t) / 200 * 1000, 3)
    for k in ("keypoint_random_forest", "image_hog_logistic"):
        log.info("B %-24s dev F1 %.4f  S4 F1 %.4f  all F1 %.4f", k, res[k]["dev_loso_S1-S3"]["macro_f1"],
                 res[k]["heldout_S4"]["macro_f1"], res[k]["all_loso"]["macro_f1"])
    return res


# ---------------------------------------------------------- A: camera resolution
def exp_A(cfg, quick):
    from src.detection.yolo_onnx import PoseDetector
    thr = cfg.detection.keypoint_conf
    S = collect(thr, 0)
    index = {v["video_id"]: v for v in load_index()}
    det = PoseDetector(ROOT / cfg.models.pose_model, cfg.detection.pose_imgsz, cfg.detection.person_conf,
                       cfg.detection.nms_iou, ["CPUExecutionProvider"])
    # 3 videos per subject with the most labelled frames, only 1280x720 sources
    pick = []
    for s in DEV + [HELD_OUT]:
        c = Counter(v for v, g in zip(S["vid"], S["g"]) if g == s and index[v]["width"] == "1280")
        pick += [v for v, _ in c.most_common(2 if quick else 3)]
    step = 4
    models = {}
    for s in DEV + [HELD_OUT]:          # fold models trained on the OTHER subjects (full resolution)
        tr = S["g"] != s
        Sv = subset(S, tr & (np.array([visible_keypoints(k, thr) for k in S["kpts"]]) >= cfg.posture.min_visible_keypoints))
        models[s] = rf().fit(np.nan_to_num(Sv["X"], nan=-9.0), Sv["y"])
    res = {}
    for scale in (1.0, 0.5, 0.25):
        found = n = 0
        vis, yt, yp, ms = [], [], [], []
        for vid in pick:
            rows = np.where(S["vid"] == vid)[0][::step]
            want = {int(S["frame"][i]): i for i in rows}
            cap = cv2.VideoCapture(str(ROOT / index[vid]["path"]))
            fno, last = 0, max(want)
            while fno <= last:
                ok, img = cap.read()
                if not ok:
                    break
                if fno in want:
                    i = want[fno]
                    small = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA) if scale != 1 else img
                    t = time.perf_counter()
                    dets = det.detect(small)
                    ms.append((time.perf_counter() - t) * 1000)
                    n += 1
                    ref = S["box"][i] * scale
                    best, biou = None, 0.0
                    for d in dets:
                        b = d.box
                        iw = max(0, min(b[2], ref[2]) - max(b[0], ref[0]))
                        ih = max(0, min(b[3], ref[3]) - max(b[1], ref[1]))
                        inter = iw * ih
                        u = (b[2] - b[0]) * (b[3] - b[1]) + (ref[2] - ref[0]) * (ref[3] - ref[1]) - inter
                        if u > 0 and inter / u > biou:
                            best, biou = d, inter / u
                    if best is not None and biou >= 0.3:
                        found += 1
                        k = best.keypoints.copy()
                        vis.append(visible_keypoints(k, thr))
                        if visible_keypoints(k, thr) >= cfg.posture.min_visible_keypoints:
                            f = frame_features(k, best.box, thr)
                            yp.append(models[index[vid]["subject"]].predict(
                                np.nan_to_num(np.array([[f[x] for x in FRAME_FEATURES]]), nan=-9.0))[0])
                            yt.append(S["y"][i])
                fno += 1
            cap.release()
        r = {"input": f"{int(1280 * scale)}x{int(720 * scale)}", "frames": n,
             "person_found_pct": round(100 * found / max(n, 1), 1),
             "mean_visible_keypoints": round(float(np.mean(vis)), 2) if vis else None,
             "posture_usable_pct": round(100 * len(yt) / max(n, 1), 1),
             "posture": scores(np.array(yt), np.array(yp)) if yt else None,
             "pose_ms_median": round(float(np.median(ms)), 1)}
        res[str(scale)] = r
        log.info("A scale %.2f (%s): found %.1f%% kpts %.2f usable %.1f%% F1 %s", scale, r["input"],
                 r["person_found_pct"], r["mean_visible_keypoints"] or 0, r["posture_usable_pct"],
                 r["posture"]["macro_f1"] if r["posture"] else None)
    return {"videos": pick, "every_nth_pose_frame": step, "results": res,
            "note": "the ONNX model input stays 640x640 (fixed export); this measures the effect of the CAMERA "
                    "resolution (e.g. IP Webcam 640x480 or lower) on detection and posture"}


EXPERIMENTS = {"C": exp_C, "D": exp_D, "E": exp_E, "G": exp_G, "H": exp_H, "B": exp_B, "A": exp_A}


# -------------------------------------------------------------------- report
def write_report(all_res: dict) -> None:
    L = ["# SentinelAI - AI experiments (A-H)", "",
         f"Generated {now_iso()} by `training/experiments.py`. Data: GMDCSA-24 (4 subjects). Settings are chosen on "
         "the development subjects S1-S3 (leave-one-subject-out inside them); S4 is a held-out check that was not "
         "used for any choice. Posture classes: LYING, SITTING, STANDING (frame level).", "",
         "**How to read this:** frames from one video are strongly correlated, so differences below about one "
         "percentage point are within noise. No experiment here is evidence about elderly people or real falls.", ""]
    if "A" in all_res:
        a = all_res["A"]
        L += ["## A - camera resolution", "", a["note"] + ".", "",
              "| camera input | person found | mean visible keypoints | posture usable | posture macro-F1 | pose ms |",
              "|---|---|---|---|---|---|"]
        for r in a["results"].values():
            L.append(f"| {r['input']} | {r['person_found_pct']}% | {r['mean_visible_keypoints']} | "
                     f"{r['posture_usable_pct']}% | {r['posture']['macro_f1'] if r['posture'] else '-'} | "
                     f"{r['pose_ms_median']} |")
        L += ["", f"Videos: {', '.join(a['videos'])} (every {a['every_nth_pose_frame']}th pose frame).", ""]
    if "B" in all_res:
        b = all_res["B"]
        L += ["## B - keypoint classifier vs image-based classifier", "",
              f"Same {b['frames']} frames (every {b['subsample_every']}rd labelled frame). Image model: HOG "
              "descriptor of a 96x96 crop around the person box + logistic regression.", "",
              "| model | dev S1-S3 macro-F1 | held-out S4 macro-F1 | all-LOSO macro-F1 | LYING F1 (all) |",
              "|---|---|---|---|---|"]
        for k, name in (("keypoint_random_forest", "Keypoint features + Random Forest (deployed type)"),
                        ("image_hog_logistic", "Image HOG + logistic regression")):
            r = b[k]
            L.append(f"| {name} | {r['dev_loso_S1-S3']['macro_f1']} | {r['heldout_S4']['macro_f1']} | "
                     f"{r['all_loso']['macro_f1']} | {r['all_loso']['f1_LYING']} |")
        L += ["", f"Image model cost per person: HOG {b['image_ms_per_sample_hog']} ms + classifier "
                  f"{b['image_ms_per_sample_classifier_only']} ms (on top of the person detector).", ""]
    if "C" in all_res:
        c = all_res["C"]
        L += ["## C - keypoint confidence threshold (chosen on S1-S3)", "",
              "| keypoint_conf | labelled frames kept | dev macro-F1 | S4 macro-F1 |", "|---|---|---|---|"]
        for r in c["results"]:
            L.append(f"| {r['keypoint_conf']} | {r['frames']} | {r['dev_loso_S1-S3']['macro_f1']} | "
                     f"{r['heldout_S4']['macro_f1']} |")
        L += ["", f"Chosen on development subjects: **{c['chosen_on_dev']}** (current setting "
                  f"{c['current_setting']}). {c['note']}.", ""]
    if "D" in all_res:
        d = all_res["D"]
        L += ["## D - training augmentation", "", "| augmentation | dev macro-F1 | dev LYING F1 | S4 macro-F1 |",
              "|---|---|---|---|"]
        for k, r in d["results"].items():
            L.append(f"| {k} | {r['dev_loso_S1-S3']['macro_f1']} | {r['dev_loso_S1-S3']['f1_LYING']} | "
                     f"{r['heldout_S4']['macro_f1']} |")
        L += ["", "Occlusion test (3 visible keypoints removed from every test frame, all-LOSO):", "",
              "| trained with | macro-F1 | LYING F1 |", "|---|---|---|"]
        for k, r in d["occlusion_test_3_keypoints_removed"].items():
            L.append(f"| {k} | {r['macro_f1']} | {r['f1_LYING']} |")
        L.append("")
    if "E" in all_res:
        e = all_res["E"]
        L += ["## E - class imbalance", "", f"Class counts: {e['class_counts']}", "",
              "| method | dev macro-F1 | dev LYING F1 | S4 macro-F1 | S4 LYING F1 |", "|---|---|---|---|---|"]
        for k, r in e.items():
            if k == "class_counts":
                continue
            L.append(f"| {k} | {r['dev_loso_S1-S3']['macro_f1']} | {r['dev_loso_S1-S3']['f1_LYING']} | "
                     f"{r['heldout_S4']['macro_f1']} | {r['heldout_S4']['f1_LYING']} |")
        L.append("")
    L += ["## F - fine-tuning the pose model", "",
          "Not run. Reasons: (1) GMDCSA-24 has no keypoint ground truth, so a fine-tuned pose model could not be "
          "measured; (2) the pose model already finds the labelled person in 92.7% of frames with 15.1/17 "
          "keypoints on average (evaluation/pose/pose_coverage.json); (3) training YOLO needs the Ultralytics "
          "package and a GPU, neither available in this build environment. Fine-tuning would be justified once "
          "robot-height recordings with keypoint labels exist.", ""]
    if "G" in all_res:
        g = all_res["G"]
        L += ["## G - temporal settings", "", "Posture smoothing (causal majority vote over the last N pose frames, "
              f"15 pose frames/s; ground truth changes {g['ground_truth_label_changes_per_min']} times/min):", "",
              "| window N | accuracy | macro-F1 | label changes / min | added delay (s) |", "|---|---|---|---|---|"]
        for r in g["posture_smoothing"]:
            L.append(f"| {r['window']} | {r['accuracy']} | {r['macro_f1']} | {r['label_changes_per_min']} | "
                     f"{r['delay_s_at_15fps']} |")
        ft = g.get("fall_timing_tuning") or {}
        L += ["", f"Current smoothing_window: {g['current_setting']}.", "",
              f"Fall timing (window length, time down before FALL_SUSPECTED) was tuned on {ft.get('development_subjects')} "
              f"with {ft.get('held_out')} held out ({ft.get('grid_size')} settings; evaluation/fall/tuning.json). "
              f"Best on development subjects: {json.dumps(ft.get('best'))}.", ""]
    if "H" in all_res:
        h = all_res["H"]
        L += ["## H - environment (lighting)", "", h["note"] + ".", "",
              "| lighting | videos | posture frames | posture macro-F1 | accuracy |", "|---|---|---|---|---|"]
        for k, r in h["posture_by_lighting"].items():
            L.append(f"| {k} | {r['videos']} | {r['n']} | {r['macro_f1']} | {r['accuracy']} |")
        if h["fall_by_lighting"]:
            L += ["", "Fall detection (deployed system, nested LOSO):", "",
                  "| lighting | videos | falls | minutes | SUSPECTED recall | SUSPECTED false alarms | CONFIRMED recall |",
                  "|---|---|---|---|---|---|---|"]
            for k, r in h["fall_by_lighting"].items():
                L.append(f"| {k} | {r['videos']} | {r['fall_events']} | {r['minutes']} | "
                         f"{r['FALL_SUSPECTED']['recall']} | {r['FALL_SUSPECTED']['false_alarms']} | "
                         f"{r['FALL_CONFIRMED']['recall']} |")
        L.append("")
    L += ["## What changed in the deployed system because of these experiments", "",
          "See the 'Decisions' section in reports/error_analysis.md. Settings are only changed when the "
          "development-subject result improves and the held-out check does not get worse.", ""]
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text("\n".join(L), encoding="utf-8")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--only", nargs="+", choices=list(EXPERIMENTS), default=list(EXPERIMENTS))
    ap.add_argument("--quick", action="store_true")
    args = ap.parse_args()
    cfg = load_config()
    OUT.mkdir(parents=True, exist_ok=True)
    all_res = {}
    for k in EXPERIMENTS:                      # keep earlier results for the report
        p = OUT / f"{k}.json"
        if p.exists() and k not in args.only:
            all_res[k] = json.loads(p.read_text(encoding="utf-8"))["result"]
    for k in args.only:
        t0 = time.time()
        log.info("=== experiment %s ===", k)
        r = EXPERIMENTS[k](cfg, args.quick)
        all_res[k] = r
        write_json(OUT / f"{k}.json", {"experiment": k, "created": now_iso(), "seconds": round(time.time() - t0, 1),
                                       "quick": args.quick, "software": software_versions(), "result": r})
    write_report(all_res)
    log.info("Report: %s", REPORT.relative_to(ROOT))


if __name__ == "__main__":
    main()
