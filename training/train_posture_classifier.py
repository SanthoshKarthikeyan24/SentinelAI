"""Train and evaluate the posture classifier (frame features -> posture).

Protocol
  1. Build frame samples from the labelled person in every video
     (labels from the annotated intervals; ambiguous frames are skipped).
  2. Leave-one-subject-out (LOSO) evaluation of:
       - the rule-based baseline (no training)
       - Random Forest
       - Histogram Gradient Boosting
     Every test frame is predicted by a model that never saw that person.
  3. Pick the candidate with the best pooled macro-F1.
  4. Retrain it on ALL subjects and save it to artifacts/posture_classifier/.

Outputs
  artifacts/posture_classifier/posture_model.joblib  model + feature list + label map
  artifacts/posture_classifier/model_card.json       dataset, metrics, versions, limitations
  evaluation/posture/metrics.json, confusion_*.png, report.md

Usage:
    python training/train_posture_classifier.py
    python training/train_posture_classifier.py --candidates rf      # only Random Forest
"""
from __future__ import annotations

import argparse
from collections import Counter

import joblib
import numpy as np
from sklearn.ensemble import HistGradientBoostingClassifier, RandomForestClassifier
from sklearn.metrics import accuracy_score, classification_report, confusion_matrix, f1_score

from common import ARTIFACTS, EVALUATION, get_logger, now_iso, software_versions, write_json
from dataset_lib import build_all_caches, load_events, load_index, load_splits, posture_samples
from src.analysis.fall_detector import FallParams
from src.analysis.features import FRAME_FEATURES
from src.analysis.posture import rule_based_posture
from src.config import load_config

log = get_logger("posture")
SEED = 42


def make_candidate(name: str):
    if name == "rf":
        return RandomForestClassifier(n_estimators=200, min_samples_leaf=3, max_features="sqrt",
                                      class_weight="balanced_subsample", n_jobs=-1, random_state=SEED)
    if name == "hgb":
        return HistGradientBoostingClassifier(max_iter=250, learning_rate=0.06, max_leaf_nodes=15,
                                              l2_regularization=1.0, class_weight="balanced",
                                              random_state=SEED)
    raise ValueError(name)


CANDIDATE_NAMES = {"rf": "Random Forest", "hgb": "Hist Gradient Boosting", "rules": "Rule baseline"}


def rules_predict(X: np.ndarray) -> np.ndarray:
    return np.array([rule_based_posture(dict(zip(FRAME_FEATURES, row)))[0] for row in X])


def metrics_for(y_true, y_pred, labels) -> dict:
    rep = classification_report(y_true, y_pred, labels=labels, output_dict=True, zero_division=0)
    return {
        "accuracy": round(float(accuracy_score(y_true, y_pred)), 4),
        "macro_f1": round(float(f1_score(y_true, y_pred, labels=labels, average="macro", zero_division=0)), 4),
        "per_class": {l: {k: round(float(rep[l][k]), 4) for k in ("precision", "recall", "f1-score", "support")}
                      for l in labels},
        "confusion_matrix": {"labels": labels,
                             "matrix": confusion_matrix(y_true, y_pred, labels=labels).tolist()},
        "predicted_other_labels": dict(Counter(p for p in y_pred if p not in labels)),
    }


def plot_confusion(cm: list[list[int]], labels: list[str], title: str, path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    cm = np.array(cm, dtype=float)
    norm = cm / np.maximum(cm.sum(axis=1, keepdims=True), 1)
    fig, ax = plt.subplots(figsize=(4.8, 4.2))
    ax.imshow(norm, cmap="Blues", vmin=0, vmax=1)
    ax.set_xticks(range(len(labels)), labels, rotation=30, ha="right")
    ax.set_yticks(range(len(labels)), labels)
    for i in range(len(labels)):
        for j in range(len(labels)):
            ax.text(j, i, f"{int(cm[i, j])}\n{norm[i, j]:.0%}", ha="center", va="center", fontsize=8,
                    color="white" if norm[i, j] > 0.5 else "black")
    ax.set_xlabel("predicted")
    ax.set_ylabel("true")
    ax.set_title(title, fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None)
    ap.add_argument("--candidates", nargs="+", default=["rf", "hgb"], choices=["rf", "hgb"])
    ap.add_argument("--deploy", action="store_true",
                    help="also make the new model the one the app uses (otherwise it is only registered)")
    args = ap.parse_args()
    cfg = load_config(args.config)
    p = FallParams.from_config(cfg.fall)
    index = {v["video_id"]: v for v in load_index()}
    events = load_events()
    splits = load_splits()
    caches = build_all_caches(list(index), p, cfg.detection.keypoint_conf, cfg.tracking.history_s, log)
    missing = [v for v in index if v not in caches]
    if missing:
        log.warning("%d indexed videos have no pose file (run extract_pose_features.py): %s",
                    len(missing), missing[:5])
    X, y, groups, vids = posture_samples(caches, events, index, cfg.posture.min_visible_keypoints)
    if len(X) == 0:
        raise SystemExit("No labelled posture samples found.")
    labels = sorted(set(y))
    log.info("Posture samples: %d frames from %d videos; classes %s", len(y), len(set(vids)),
             {str(k): v for k, v in Counter(y).items()})

    out_dir = EVALUATION / "posture"
    out_dir.mkdir(parents=True, exist_ok=True)
    pooled = {c: ([], []) for c in ["rules"] + args.candidates}
    fold_results = []
    for fold in splits["loso_folds"]:
        s = fold["test_subject"]
        te, tr = groups == s, groups != s
        if te.sum() == 0:
            continue
        fr = {"test_subject": s, "n_train": int(tr.sum()), "n_test": int(te.sum())}
        pred = rules_predict(X[te])
        pooled["rules"][0].extend(y[te]); pooled["rules"][1].extend(pred)
        fr["rules_macro_f1"] = round(float(f1_score(y[te], pred, labels=labels, average="macro",
                                                    zero_division=0)), 4)
        for c in args.candidates:
            m = make_candidate(c).fit(X[tr], y[tr])
            pred = m.predict(X[te])
            pooled[c][0].extend(y[te]); pooled[c][1].extend(pred)
            fr[f"{c}_macro_f1"] = round(float(f1_score(y[te], pred, labels=labels, average="macro",
                                                       zero_division=0)), 4)
        fold_results.append(fr)
        log.info("  fold test=%s: %s", s, {k: v for k, v in fr.items() if k.endswith("f1")})

    results = {}
    for c, (yt, yp) in pooled.items():
        results[c] = metrics_for(np.array(yt), np.array(yp), labels)
        plot_confusion(results[c]["confusion_matrix"]["matrix"], labels,
                       f"{CANDIDATE_NAMES[c]} (LOSO, {len(yt)} frames)", out_dir / f"confusion_{c}.png")
        log.info("%-24s accuracy %.3f  macro-F1 %.3f", CANDIDATE_NAMES[c], results[c]["accuracy"],
                 results[c]["macro_f1"])
    best = max(args.candidates, key=lambda c: results[c]["macro_f1"])
    log.info("Selected: %s", CANDIDATE_NAMES[best])

    # effect of the UNKNOWN threshold (coverage vs accuracy), out-of-fold
    unknown_curve = []
    oof_proba, oof_true = [], []
    for fold in splits["loso_folds"]:
        te, tr = groups == fold["test_subject"], groups != fold["test_subject"]
        if te.sum() == 0:
            continue
        m = make_candidate(best).fit(X[tr], y[tr])
        pr = m.predict_proba(X[te])
        oof_proba.append(np.array([dict(zip(m.classes_, row)).get(l, 0.0) for row in pr for l in labels])
                         .reshape(-1, len(labels)))
        oof_true.extend(y[te])
    oof_proba = np.vstack(oof_proba)
    oof_true = np.array(oof_true)
    pred = np.array(labels)[oof_proba.argmax(1)]
    conf = oof_proba.max(1)
    for thr in (0.0, 0.4, 0.5, 0.6, 0.7, 0.8):
        keep = conf >= thr
        unknown_curve.append({"threshold": thr, "coverage": round(float(keep.mean()), 4),
                              "accuracy_on_kept": round(float((pred[keep] == oof_true[keep]).mean()), 4)
                              if keep.any() else None})

    final = make_candidate(best).fit(X, y)
    if hasattr(final, "n_jobs"):
        final.set_params(n_jobs=1)  # single-sample predictions are faster without thread start-up
    ARTIFACTS.joinpath("posture_classifier").mkdir(parents=True, exist_ok=True)
    bundle = {"model": final, "classes": list(final.classes_), "feature_names": FRAME_FEATURES,
              "meta": {"model_type": CANDIDATE_NAMES[best], "created": now_iso()}}
    tmp_model = ARTIFACTS / "posture_classifier" / "_new_posture_model.joblib"
    joblib.dump(bundle, tmp_model, compress=3)
    card = {
        "name": "SentinelAI posture classifier", "version": now_iso(), "model_type": CANDIDATE_NAMES[best],
        "hyperparameters": {k: v for k, v in final.get_params().items() if not callable(v)},
        "training_data": {"datasets": sorted({index[v]["dataset"] for v in set(vids)}),
                          "subjects": sorted(set(groups)), "frames": int(len(y)),
                          "videos": int(len(set(vids))), "class_counts": {str(k): v for k, v in Counter(y).items()}},
        "input": {"feature_format": "FRAME_FEATURES from src/analysis/features.py",
                  "features": FRAME_FEATURES, "pose_model": cfg.models.pose_model,
                  "keypoint_conf": cfg.detection.keypoint_conf},
        "label_mapping": {"STANDING": "dataset Standing + Walking", "SITTING": "dataset Sitting",
                          "LYING": "dataset Sleeping (lying on a bed)"},
        "evaluation": {"protocol": "leave-one-subject-out (4 folds), pooled over folds",
                       "selected": results[best], "baseline_rules": results["rules"],
                       "all_candidates": {c: {"accuracy": results[c]["accuracy"], "macro_f1": results[c]["macro_f1"]}
                                          for c in results},
                       "folds": fold_results, "unknown_threshold_curve": unknown_curve},
        "software": software_versions(),
        "limitations": [
            "Only STANDING, SITTING and LYING are learned; BENDING and KNEELING come from the rule baseline.",
            "4 adult subjects in 3 rooms, camera 64-135 cm high; robot-height views were not in the data.",
            "LYING examples are people lying on a bed; lying on the floor was not labelled separately.",
            "Frame-level samples within one video are highly correlated, so the effective sample size is much smaller than the frame count.",
        ],
    }
    import model_registry
    version = model_registry.save_version("posture_classifier", tmp_model, card,
                                          {"loso_accuracy": results[best]["accuracy"],
                                           "loso_macro_f1": results[best]["macro_f1"]})
    tmp_model.unlink()
    if args.deploy:
        model_registry.deploy("posture_classifier", version, "train_posture_classifier.py --deploy")
        log.info("Registered AND deployed posture model %s", version)
    else:
        log.info("Registered posture model %s (NOT deployed - the app still uses %s). To use it: "
                 "python training/model_registry.py deploy posture_classifier %s", version,
                 model_registry.load()["models"]["posture_classifier"]["deployed"], version)
    write_json(out_dir / "metrics.json", card["evaluation"])
    lines = ["# Posture classifier evaluation", "",
             f"Protocol: leave-one-subject-out over {len(set(groups))} subjects, {len(y)} labelled frames "
             f"from {len(set(vids))} videos.", "",
             "| Model | Accuracy | Macro-F1 | " + " | ".join(f"{l} F1" for l in labels) + " |",
             "|---|---|---|" + "---|" * len(labels)]
    for c in results:
        r = results[c]
        lines.append(f"| {CANDIDATE_NAMES[c]} | {r['accuracy']:.3f} | {r['macro_f1']:.3f} | " +
                     " | ".join(f"{r['per_class'][l]['f1-score']:.3f}" for l in labels) + " |")
    lines += ["", f"Selected model: **{CANDIDATE_NAMES[best]}** (retrained on all subjects).", "",
              "UNKNOWN threshold (selected model, out-of-fold):", "",
              "| threshold | frames kept | accuracy on kept |", "|---|---|---|"]
    lines += [f"| {u['threshold']} | {u['coverage']:.1%} | {u['accuracy_on_kept']} |" for u in unknown_curve]
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    log.info("Report written to evaluation/posture/")


if __name__ == "__main__":
    main()
