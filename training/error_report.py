"""Write reports/error_analysis.md: where and why the models fail (all numbers computed here).

Inputs: cached pose files + labels (posture, out-of-fold leave-one-subject-out predictions),
evaluation/fall/error_analysis.csv (nested-LOSO fall errors of the deployed system),
data/dataset_inventory.csv (lighting), evaluation/experiments/*.json (decisions).
Also saves a few example frames of posture errors to reports/error_examples/ (public dataset
actors; no other images are stored).

Run after training/experiments.py:  python training/error_report.py
"""
from __future__ import annotations

import csv
import json
from collections import Counter, defaultdict

import cv2
import numpy as np

from common import DATA, EVALUATION, ROOT, get_logger, now_iso
from dataset_lib import load_index
from experiments import DEV, HELD_OUT, LABELS, collect, lighting_of, loso, rf_fit_predict
from src.analysis.features import visible_keypoints
from src.config import load_config

log = get_logger("error_report")
REPORT = ROOT / "reports" / "error_analysis.md"
EXAMPLES = ROOT / "reports" / "error_examples"


def confusion(y, p) -> list[list[int]]:
    return [[int(((y == a) & (p == b)).sum()) for b in LABELS] for a in LABELS]


def cm_table(cm) -> list[str]:
    L = ["| true \\ predicted | " + " | ".join(LABELS) + " |", "|---|" + "---|" * len(LABELS)]
    for a, row in zip(LABELS, cm):
        L.append(f"| {a} | " + " | ".join(str(x) for x in row) + " |")
    return L


def save_examples(S, p, wrong_idx, index, n=6) -> list[str]:
    EXAMPLES.mkdir(parents=True, exist_ok=True)
    for old in EXAMPLES.glob("*.jpg"):
        old.unlink()
    saved = []
    by_vid = defaultdict(list)
    for i in wrong_idx:
        by_vid[S["vid"][i]].append(i)
    # one example from each of the videos with the most errors
    for vid, rows in sorted(by_vid.items(), key=lambda kv: -len(kv[1]))[:n]:
        i = rows[len(rows) // 2]
        cap = cv2.VideoCapture(str(ROOT / index[vid]["path"]))
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(S["frame"][i]))
        ok, img = cap.read()
        cap.release()
        if not ok:
            continue
        x1, y1, x2, y2 = [int(v) for v in S["box"][i]]
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 3)
        for k in S["kpts"][i]:
            if k[2] >= 0.35:
                cv2.circle(img, (int(k[0]), int(k[1])), 5, (0, 255, 255), -1)
        txt = f"{vid} t={S['t'][i]:.1f}s true {S['y'][i]} -> predicted {p[i]}"
        cv2.putText(img, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
        cv2.putText(img, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        h, w = img.shape[:2]
        img = cv2.resize(img, (640, int(h * 640 / w)))
        name = f"{vid}_{S['t'][i]:.1f}s.jpg"
        cv2.imwrite(str(EXAMPLES / name), img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        saved.append((name, vid, float(S["t"][i]), str(S["y"][i]), str(p[i]), len(rows)))
    return saved


def main() -> None:
    cfg = load_config()
    thr = cfg.detection.keypoint_conf
    index = {v["video_id"]: v for v in load_index()}
    S = collect(thr, cfg.posture.min_visible_keypoints)
    p, u = loso(S, DEV + [HELD_OUT], rf_fit_predict())
    y = S["y"]
    wrong = np.where(p != y)[0]
    light = lighting_of()
    lg = np.array([light.get(v, "unknown") for v in S["vid"]])
    nvis = np.array([visible_keypoints(k, thr) for k in S["kpts"]])
    L = ["# Error analysis", "",
         f"Generated {now_iso()} by `training/error_report.py`. Posture: out-of-fold predictions of the deployed "
         "model type (Random Forest on keypoint features), leave-one-subject-out over S1-S4. Fall: errors of the "
         "deployed fall model type and threshold under leave-one-subject-out (`evaluation/fall/error_analysis.csv`).", "",
         "## 1. Posture errors", "",
         f"{len(wrong)} wrong of {len(y)} labelled frames ({100 * len(wrong) / len(y):.1f} %).", "", "All frames:", ""]
    L += cm_table(confusion(y, p))
    pairs = Counter(zip(y[wrong], p[wrong])).most_common()
    L += ["", "Most common mistakes:", "", "| true | predicted | frames | share of all errors |", "|---|---|---|---|"]
    L += [f"| {a} | {b} | {n} | {100 * n / len(wrong):.0f} % |" for (a, b), n in pairs]
    L += ["", "### By lighting (dataset 'Time of Recording')", "",
          "| lighting | frames | errors | error rate | LYING frames | LYING recall |", "|---|---|---|---|---|---|"]
    for g in sorted(set(lg)):
        m = lg == g
        ly = m & (y == "LYING")
        L.append(f"| {g} | {m.sum()} | {(m & (p != y)).sum()} | {100 * (m & (p != y)).sum() / m.sum():.1f} % | "
                 f"{ly.sum()} | {(p[ly] == 'LYING').mean():.3f} |" if ly.any() else
                 f"| {g} | {m.sum()} | {(m & (p != y)).sum()} | {100 * (m & (p != y)).sum() / m.sum():.1f} % | 0 | - |")
    night = lg == "night"
    if night.any():
        L += ["", "Night frames only:", ""] + cm_table(confusion(y[night], p[night]))
    L += ["", "### By number of visible keypoints", "", "| visible keypoints | frames | error rate |", "|---|---|---|"]
    for lo, hi in ((7, 10), (11, 13), (14, 15), (16, 17)):
        m = (nvis >= lo) & (nvis <= hi)
        if m.any():
            L.append(f"| {lo}-{hi} | {m.sum()} | {100 * (m & (p != y)).sum() / m.sum():.1f} % |")
    L += ["", "### By subject (each tested by a model trained on the other three)", "",
          "| subject | frames | error rate |", "|---|---|---|"]
    for s in DEV + [HELD_OUT]:
        m = S["g"] == s
        L.append(f"| {s} | {m.sum()} | {100 * (m & (p != y)).sum() / m.sum():.1f} % |")
    per_vid = Counter(S["vid"][wrong])
    L += ["", "### Videos with the most errors", "", "| video | errors | frames | lighting | description |",
          "|---|---|---|---|---|"]
    for vid, n in per_vid.most_common(8):
        L.append(f"| {vid} | {n} | {(S['vid'] == vid).sum()} | {light.get(vid, '')} | "
                 f"{index[vid]['description'][:90]} |")
    ex = save_examples(S, p, wrong, index)
    if ex:
        L += ["", "Example frames (red box = tracked person, yellow = visible keypoints):", ""]
        L += [f"- `reports/error_examples/{n}` – true {a}, predicted {b} ({k} wrong frames in this video)"
              for n, vid, t, a, b, k in ex]
    # ---------------------------------------------------------------- falls
    err_p = EVALUATION / "fall" / "error_analysis.csv"
    if err_p.exists():
        rows = list(csv.DictReader(open(err_p, newline="", encoding="utf-8")))
        L += ["", "## 2. Fall detection errors (deployed model type and threshold, leave-one-subject-out)", "",
              "This list comes from `training/analyse_errors.py`, which uses the deployed model type and threshold "
              "in every fold. The headline numbers in evaluation/SUMMARY.md use *nested* selection (each fold "
              "chooses its own model and threshold), so the counts can differ by one or two.", "",
              "| error | level | count |", "|---|---|---|"]
        for (e, lvl), n in sorted(Counter((r["error"], r["level"]) for r in rows).items()):
            L.append(f"| {e} | {lvl} | {n} |")
        def theme(d: str) -> str:
            d = d.lower()
            if "not visible" in d or "partially" in d or "partial" in d:
                return "body partly out of view"
            if "bed" in d:
                return "on / onto a bed"
            if "chair" in d or "sitting" in d:
                return "from / near a chair"
            if "exercis" in d or "yoga" in d or "push" in d:
                return "exercise"
            if "sleep" in d or "lying" in d:
                return "lying down"
            return "other"
        L += ["", "Grouped by what the video description says (descriptions come from the dataset authors):", "",
              "| error | level | theme | count |", "|---|---|---|---|"]
        for (e, lvl, th), n in sorted(Counter((r["error"], r["level"], theme(r["description"])) for r in rows).items()):
            L.append(f"| {e} | {lvl} | {th} | {n} |")
        L += ["", "Every missed FALL_SUSPECTED (the first alarm level):", "", "| video | description |", "|---|---|"]
        L += [f"| {r['video_id']} | {r['description'][:110]} |" for r in rows
              if r["error"] == "MISSED" and r["level"] == "FALL_SUSPECTED"]
        L += ["", "Every FALL_SUSPECTED false alarm:", "", "| video | time (s) | description |", "|---|---|---|"]
        L += [f"| {r['video_id']} | {r['time_s']} | {r['description'][:100]} |" for r in rows
              if r["error"] == "FALSE_ALARM" and r["level"] == "FALL_SUSPECTED"]
    # ------------------------------------------------------------- decisions
    exp = {}
    for k in "ABCDEGH":
        f = EVALUATION / "experiments" / f"{k}.json"
        if f.exists():
            exp[k] = json.loads(f.read_text(encoding="utf-8"))["result"]
    L += ["", "## 3. Decisions taken from the experiments (reports/experiments.md)", "",
          "Rule: change a deployed setting only if the development subjects (S1-S3) improve by more than the noise "
          "level (about 1 point macro-F1) **and** the held-out subject S4 does not get worse.", ""]
    if "C" in exp:
        c = exp["C"]
        cur = next(r for r in c["results"] if abs(r["keypoint_conf"] - c["current_setting"]) < 1e-9)
        best = next(r for r in c["results"] if r["keypoint_conf"] == c["chosen_on_dev"])
        gain = best["dev_loso_S1-S3"]["macro_f1"] - cur["dev_loso_S1-S3"]["macro_f1"]
        keep = gain <= 0.01 or best["heldout_S4"]["macro_f1"] < cur["heldout_S4"]["macro_f1"]
        L.append(f"- **keypoint_conf**: best on S1-S3 is {c['chosen_on_dev']} (+{100 * gain:.2f} points), S4 "
                 f"{best['heldout_S4']['macro_f1']} vs {cur['heldout_S4']['macro_f1']} -> "
                 + ("**kept at %s** (difference within noise)." % c["current_setting"] if keep else "changed."))
    if "D" in exp:
        d = exp["D"]["results"]
        base = d["none"]["dev_loso_S1-S3"]["macro_f1"]
        better = [k for k, r in d.items() if r["dev_loso_S1-S3"]["macro_f1"] > base + 0.01]
        L.append(f"- **augmentation**: no variant beats 'none' on S1-S3 ({base}); " +
                 ("kept none." if not better else f"candidates: {better}."))
    if "E" in exp:
        e = {k: v for k, v in exp["E"].items() if k != "class_counts"}
        best = max(e, key=lambda k: e[k]["dev_loso_S1-S3"]["macro_f1"])
        L.append(f"- **class imbalance**: best on S1-S3 is '{best}' -> " +
                 ("kept (already deployed)." if "current" in best else "consider switching."))
    if "G" in exp:
        g = {r["window"]: r for r in exp["G"]["posture_smoothing"]}
        cur = exp["G"]["current_setting"]
        L.append(f"- **posture smoothing**: window 1 gives the best frame accuracy ({g[1]['accuracy']}) but "
                 f"{g[1]['label_changes_per_min']} label changes/min (ground truth: "
                 f"{exp['G']['ground_truth_label_changes_per_min']}); window {cur} gives {g[cur]['accuracy']} with "
                 f"{g[cur]['label_changes_per_min']} changes/min. **Kept {cur}**: a steadier posture matters more to "
                 "the fall state machine than half a point of frame accuracy.")
    if "B" in exp:
        b = exp["B"]
        k, i = b["keypoint_random_forest"]["all_loso"]["macro_f1"], b["image_hog_logistic"]["all_loso"]["macro_f1"]
        L.append(f"- **keypoint vs image classifier**: keypoint RF {k} vs image HOG {i} macro-F1 (same frames) -> "
                 + ("**kept the keypoint classifier**." if k >= i else "image model is better - investigate."))
    if "A" in exp:
        res = exp["A"]["results"]
        L.append("- **camera resolution**: " + "; ".join(
            f"{r['input']}: person found {r['person_found_pct']} %, posture F1 "
            f"{r['posture']['macro_f1'] if r['posture'] else '-'}" for r in res.values())
            + ". Recommendation: set IP Webcam to at least 640×480; 1280×720 if the laptop keeps up.")
    if "H" in exp:
        h = exp["H"]["posture_by_lighting"]
        L.append(f"- **lighting**: night posture macro-F1 {h.get('night', {}).get('macro_f1')} vs daylight "
                 f"{h.get('day_lit', {}).get('macro_f1')} -> no setting change; the fix is more night-time "
                 "training data (record your own with training/collect_data.py, with consent).")
    L += ["", "## 4. What would most improve accuracy", "",
          "1. Recordings from the robot's own camera height in the target home (with consent), labelled with "
          "training/annotate_videos.py - the biggest gap between the dataset and real use.",
          "2. More night / dim-light examples, especially lying on a bed and on the floor.",
          "3. Longer clips that continue after the fall, so FALL_CONFIRMED can be measured properly.",
          "4. A second public dataset (UR Fall, Le2i, CAUCAFall) when it can be downloaded, as an external test.", ""]
    REPORT.write_text("\n".join(L), encoding="utf-8")
    log.info("Wrote %s (%d posture errors, %d example frames)", REPORT.relative_to(ROOT), len(wrong), len(ex))


if __name__ == "__main__":
    main()
