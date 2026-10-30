"""Train the fall-window classifier and evaluate fall detection at EVENT level.

What is learned
  A window classifier: "did a fall-like movement happen in the last
  window_s seconds?" from WINDOW_FEATURES (hip/head/box-top descent speed,
  drop, torso rotation, box shape change, ...). Positives are windows ending
  0.4-2.0 s after a labelled fall onset; negatives are all windows from
  ordinary activities (sitting down, lying down on a bed, walking, picking
  things up, exercising) and the pre-fall parts of fall videos.

What is evaluated
  The full decision system (posture classifier + fall classifier + state
  machine, exactly as used live) replayed on unseen subjects:
    * event-level precision / recall / F1 for FALL_SUSPECTED and FALL_CONFIRMED
    * false alarms per hour of non-fall video
    * detection delay (alarm time - labelled fall start)
  compared against the rule-based baseline (rule posture + rule fall evidence).

Protocol (nested, no test-subject information used for any choice)
  outer loop: leave one subject out for testing
  inner loop: leave-one-subject-out on the remaining subjects to choose the
              model type (RF / HGB) and the decision threshold that maximises
              event-level F1 of FALL_SUSPECTED.
  final model: same selection on all subjects, retrained on all data.

Usage:
    python training/train_fall_classifier.py
    python training/train_fall_classifier.py --fast     # RF only, smaller threshold grid
"""
from __future__ import annotations

import argparse
from collections import Counter

import joblib
import numpy as np
from sklearn.ensemble import HistGradientBoostingClassifier, RandomForestClassifier
from sklearn.metrics import average_precision_score, roc_auc_score

from common import ARTIFACTS, EVALUATION, get_logger, now_iso, software_versions, write_json
from dataset_lib import (POS_END, POS_START, build_all_caches, fall_samples, gt_falls, load_events,
                         load_index, load_splits, match_events, posture_samples, replay_video,
                         summarise_matches)
from src.analysis.fall_detector import FallDetector, FallParams
from src.analysis.features import WINDOW_FEATURES
from src.analysis.posture import PostureClassifier
from src.config import load_config

log = get_logger("fall")
SEED = 42
NAMES = {"rf": "Random Forest", "hgb": "Hist Gradient Boosting"}


def make_fall_model(name: str):
    if name == "rf":
        return RandomForestClassifier(n_estimators=200, min_samples_leaf=2, max_features="sqrt",
                                      class_weight="balanced_subsample", n_jobs=-1, random_state=SEED)
    if name == "hgb":
        return HistGradientBoostingClassifier(max_iter=300, learning_rate=0.05, max_leaf_nodes=15,
                                              l2_regularization=1.0, class_weight="balanced",
                                              random_state=SEED)
    raise ValueError(name)


def make_posture_model(name: str):
    # same families/hyper-parameters as train_posture_classifier.py
    from train_posture_classifier import make_candidate
    return make_candidate(name)


class Ctx:
    """Everything the replay needs, built once."""

    def __init__(self, cfg, caches, events, index, posture_type, params: FallParams | None = None,
                 feature_names: list[str] | None = None):
        self.cfg, self.caches, self.events, self.index = cfg, caches, events, index
        self.p = params or FallParams.from_config(cfg.fall)
        self.feature_names = list(feature_names or WINDOW_FEATURES)
        self.fidx = np.array([WINDOW_FEATURES.index(f) for f in self.feature_names])
        self.posture_type = posture_type
        self.PX, self.Py, self.Pg, _ = posture_samples(caches, events, index, cfg.posture.min_visible_keypoints)
        self.FX, self.Fy, self.Fg, self.Fv = fall_samples(caches, events, index)

    def videos_of(self, subjects):
        return [v for v in self.caches if self.index[v]["subject"] in subjects]

    def posture_fn(self, subjects):
        if self.posture_type == "rules":
            return None, None
        m = make_posture_model(self.posture_type).fit(self.PX[np.isin(self.Pg, subjects)],
                                                      self.Py[np.isin(self.Pg, subjects)])
        if hasattr(m, "n_jobs"):
            m.set_params(n_jobs=1)
        return m.predict_proba, list(m.classes_)

    def fall_fn(self, model_name, subjects):
        mask = np.isin(self.Fg, subjects)
        m = make_fall_model(model_name).fit(self.FX[mask][:, self.fidx], self.Fy[mask])
        if hasattr(m, "n_jobs"):
            m.set_params(n_jobs=1)   # many small predictions: threads only add overhead
        idx = self.fidx
        return (lambda X: m.predict_proba(X[:, idx])[:, 1]), m

    def replay(self, videos, fall_fn, threshold, posture_fn, posture_classes, use_rules_posture=False):
        """Return per-video alarms for the given decision system."""
        cfg = self.cfg
        out = {}
        for v in videos:
            pc = PostureClassifier(None, use_model=False, kpt_thr=cfg.detection.keypoint_conf,
                                   min_visible=cfg.posture.min_visible_keypoints,
                                   unknown_threshold=cfg.posture.unknown_proba_threshold,
                                   smoothing_window=cfg.posture.smoothing_window)
            params = FallParams(**{**self.p.__dict__, "model_threshold": threshold})
            fd = FallDetector(params, model=_ThresholdOnly() if fall_fn else None,
                              kpt_thr=cfg.detection.keypoint_conf)
            res = replay_video(self.caches[v], pc, fd, fall_score_fn=fall_fn,
                               posture_proba_fn=None if use_rules_posture else posture_fn,
                               posture_classes=None if use_rules_posture else posture_classes,
                               min_hits=cfg.tracking.min_hits, max_missed_s=cfg.tracking.max_missed_s)
            out[v] = res["alarms"]
        return out

    def score(self, alarms_by_video, alarm_type):
        results, neg = [], 0.0
        for v, alarms in alarms_by_video.items():
            gt = gt_falls(self.events.get(v, []))
            results.append(match_events(alarms, gt, alarm_type))
            dur = self.index[v]["duration_s"]
            neg += max(0.0, dur - sum(min(e, dur) - s for s, e in gt))
        return summarise_matches(results, neg)


class _ThresholdOnly:
    """Placeholder so FallDetector reports mode 'model'; scores come from the batch function."""
    recommended_threshold = 0.5

    def score(self, wf):  # pragma: no cover - never called when evidence is precomputed
        raise RuntimeError("batch scores expected")


def select(ctx: Ctx, subjects: list[str], candidates: list[str], grid: list[float]) -> dict:
    """Inner LOSO over `subjects`: choose model type + threshold by event-level F1."""
    table = {c: {t: [] for t in grid} for c in candidates}
    for u in subjects:
        rest = [s for s in subjects if s != u]
        vids = ctx.videos_of([u])
        pfn, pcls = ctx.posture_fn(rest)
        for c in candidates:
            ffn, _ = ctx.fall_fn(c, rest)
            # predict once, reuse for every threshold
            cache = {}

            def cached(X, _f=ffn, _cache=cache):
                key = X.tobytes()
                if key not in _cache:
                    _cache[key] = _f(X)
                return _cache[key]
            for t in grid:
                alarms = ctx.replay(vids, cached, t, pfn, pcls)
                table[c][t].append(alarms)
    best = None
    summary = {}
    for c in candidates:
        for t in grid:
            merged = {k: v for d in table[c][t] for k, v in d.items()}
            s = ctx.score(merged, "FALL_SUSPECTED")
            summary[f"{c}@{t}"] = {"f1": s["f1"], "precision": s["precision"], "recall": s["recall"]}
            key = (s["f1"], s["precision"], t)   # tie-break: precision, then higher threshold
            if best is None or key > best[0]:
                best = (key, c, t)
    return {"model": best[1], "threshold": best[2], "inner_f1": best[0][0], "grid": summary}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None)
    ap.add_argument("--fast", action="store_true", help="RF only and a coarse threshold grid")
    ap.add_argument("--deploy", action="store_true",
                    help="also make the new model the one the app uses (otherwise it is only registered)")
    args = ap.parse_args()
    cfg = load_config(args.config)
    candidates = ["rf"] if args.fast else ["rf", "hgb"]
    grid = [0.3, 0.5, 0.7] if args.fast else [0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]

    index = {v["video_id"]: v for v in load_index()}
    events = load_events()
    splits = load_splits()
    p = FallParams.from_config(cfg.fall)
    caches = build_all_caches(list(index), p, cfg.detection.keypoint_conf, cfg.tracking.history_s, log)
    posture_card = ARTIFACTS / "posture_classifier" / "model_card.json"
    posture_type = "rf"
    if posture_card.exists():
        import json
        mt = json.loads(posture_card.read_text())["model_type"]
        posture_type = {"Random Forest": "rf", "Hist Gradient Boosting": "hgb"}.get(mt, "rf")
    ctx = Ctx(cfg, caches, events, index, posture_type)
    log.info("Fall windows: %d (positives %d) from %d videos; posture model type for replay: %s",
             len(ctx.Fy), int(ctx.Fy.sum()), len(set(ctx.Fv)), posture_type)

    folds_out = []
    pooled = {"trained": {}, "baseline": {}}
    win_true, win_score = [], []
    for fold in splits["loso_folds"]:
        s, train_subj = fold["test_subject"], fold["train_subjects"]
        test_vids = ctx.videos_of([s])
        sel = select(ctx, train_subj, candidates, grid)
        ffn, model = ctx.fall_fn(sel["model"], train_subj)
        pfn, pcls = ctx.posture_fn(train_subj)
        trained = ctx.replay(test_vids, ffn, sel["threshold"], pfn, pcls)
        baseline = ctx.replay(test_vids, None, 0.5, None, None, use_rules_posture=True)
        pooled["trained"].update(trained)
        pooled["baseline"].update(baseline)
        mask = ctx.Fg == s
        if mask.any() and len(set(ctx.Fy[mask])) == 2:
            win_true.extend(ctx.Fy[mask]); win_score.extend(ffn(ctx.FX[mask]))
        fo = {"test_subject": s, "selected_model": NAMES[sel["model"]], "threshold": sel["threshold"],
              "inner_f1": sel["inner_f1"],
              "trained_suspected": ctx.score(trained, "FALL_SUSPECTED"),
              "trained_confirmed": ctx.score(trained, "FALL_CONFIRMED"),
              "baseline_suspected": ctx.score(baseline, "FALL_SUSPECTED"),
              "baseline_confirmed": ctx.score(baseline, "FALL_CONFIRMED")}
        folds_out.append(fo)
        log.info("fold %s: %s thr %.2f | trained suspected F1 %.3f (P %.3f R %.3f) | baseline F1 %.3f",
                 s, fo["selected_model"], sel["threshold"], fo["trained_suspected"]["f1"],
                 fo["trained_suspected"]["precision"], fo["trained_suspected"]["recall"],
                 fo["baseline_suspected"]["f1"])

    summary = {sys_: {lvl: ctx.score(pooled[sys_], lvl) for lvl in ("FALL_SUSPECTED", "FALL_CONFIRMED")}
               for sys_ in pooled}
    win_true, win_score = np.array(win_true), np.array(win_score)
    window_metrics = {"roc_auc": round(float(roc_auc_score(win_true, win_score)), 4),
                      "average_precision": round(float(average_precision_score(win_true, win_score)), 4),
                      "windows": int(len(win_true)), "positive_windows": int(win_true.sum())}

    # confirmation is limited by how long each video continues after the fall starts
    long_enough, conf_hits = 0, 0
    need = 2.5 + p.suspect_min_down_s + p.confirm_down_s
    for v, alarms in pooled["trained"].items():
        dur = index[v]["duration_s"]
        for gs, ge in gt_falls(events.get(v, [])):
            if dur - gs >= need:
                long_enough += 1
                conf_hits += match_events(alarms, [(gs, ge)], "FALL_CONFIRMED")["tp"]
    confirm_note = {"falls_with_at_least_s_after_onset": need, "count": long_enough,
                    "confirmed_recall_on_these": round(conf_hits / long_enough, 4) if long_enough else None}

    # ---- final model: selection on all subjects, retrain on everything
    all_subj = splits["subjects"]
    final_sel = select(ctx, all_subj, candidates, grid)
    _, final_model = ctx.fall_fn(final_sel["model"], all_subj)
    if hasattr(final_model, "n_jobs"):
        final_model.set_params(n_jobs=1)
    out_dir = ARTIFACTS / "fall_classifier"
    out_dir.mkdir(parents=True, exist_ok=True)
    joblib.dump({"model": final_model, "feature_names": WINDOW_FEATURES, "window_s": p.window_s,
                 "threshold": final_sel["threshold"],
                 "meta": {"model_type": NAMES[final_sel["model"]], "created": now_iso()}},
                out_dir / "_new_fall_model.joblib", compress=3)
    card = {
        "name": "SentinelAI fall-window classifier", "version": now_iso(),
        "model_type": NAMES[final_sel["model"]], "decision_threshold": final_sel["threshold"],
        "training_data": {"datasets": sorted({index[v]["dataset"] for v in caches}),
                          "subjects": all_subj, "windows": int(len(ctx.Fy)),
                          "positive_windows": int(ctx.Fy.sum()),
                          "fall_events": int(sum(len(gt_falls(events.get(v, []))) for v in caches)),
                          "videos": len(caches)},
        "labelling": {"window_s": p.window_s,
                      "positive": f"window end in [fall_start+{POS_START}s, fall_start+{POS_END}s] (and before fall end)",
                      "ignored": "boundary windows and post-fall windows (person already down)",
                      "negative": "all other windows, plus all windows of non-target tracks"},
        "input": {"features": WINDOW_FEATURES, "feature_format": "WINDOW_FEATURES from src/analysis/features.py"},
        "state_machine_parameters": p.__dict__,
        "evaluation": {"protocol": "nested leave-one-subject-out; event matching window [start-0.5s, end+1.0s]",
                       "pooled": summary, "window_level": window_metrics, "folds": folds_out,
                       "confirmation_limit": confirm_note,
                       "final_selection_grid": final_sel["grid"]},
        "software": software_versions(),
        "limitations": [
            "Staged falls by 4 young adult actors; real falls of elderly people can be slower or partial.",
            "Videos are short (2-70 s); false alarms per hour are extrapolated from about 16 minutes of non-fall video.",
            "Many clips end 1-3 s after the fall, so FALL_CONFIRMED (needs the person to stay down) is limited by clip length.",
            "Camera heights 64-135 cm; a floor-level robot camera sees people from a different angle.",
            "Falls onto a bed are labelled as falls in this dataset.",
        ],
    }
    import model_registry
    tr = summary.get("trained", {})
    version = model_registry.save_version(
        "fall_classifier", out_dir / "_new_fall_model.joblib", card,
        {lvl: {"f1": d.get("f1"), "precision": d.get("precision"), "recall": d.get("recall")} for lvl, d in tr.items()})
    (out_dir / "_new_fall_model.joblib").unlink()
    if args.deploy:
        model_registry.deploy("fall_classifier", version, "train_fall_classifier.py --deploy")
    log.info("Registered fall model %s (%s)", version, "deployed" if args.deploy else
             "NOT deployed - run: python training/model_registry.py deploy fall_classifier " + version)
    ev_dir = EVALUATION / "fall"
    ev_dir.mkdir(parents=True, exist_ok=True)
    write_json(ev_dir / "metrics.json", card["evaluation"])
    write_report(ev_dir / "report.md", summary, window_metrics, folds_out, confirm_note, card)
    log.info("POOLED (unseen subjects):")
    for sys_, d in summary.items():
        for lvl, s in d.items():
            log.info("  %-8s %-15s P %.3f R %.3f F1 %.3f FA/h %s delay(median) %s", sys_, lvl,
                     s["precision"], s["recall"], s["f1"], s["false_alarms_per_hour"], s["delay_median_s"])
    log.info("Window level: %s", window_metrics)
    log.info("Final model: %s threshold %.2f (registry version %s)", card["model_type"],
             final_sel["threshold"], version)


def write_report(path, summary, window_metrics, folds, confirm_note, card):
    L = ["# Fall detection evaluation (event level)", "",
         "Protocol: nested leave-one-subject-out. Every fall/ADL video is evaluated by a system "
         "whose models and threshold were chosen without that person.", "",
         "| System | Alarm level | Precision | Recall | F1 | TP | FN | FP | False alarms / hour | Median delay (s) |",
         "|---|---|---|---|---|---|---|---|---|---|"]
    for sys_, d in summary.items():
        for lvl, s in d.items():
            L.append(f"| {'Trained' if sys_ == 'trained' else 'Rule baseline'} | {lvl} | {s['precision']:.3f} | "
                     f"{s['recall']:.3f} | {s['f1']:.3f} | {s['true_positives']} | {s['false_negatives (missed)']} | "
                     f"{s['false_positives']} | {s['false_alarms_per_hour']} | {s['delay_median_s']} |")
    L += ["", f"Window-level (out-of-fold): ROC-AUC {window_metrics['roc_auc']}, "
          f"average precision {window_metrics['average_precision']} on {window_metrics['windows']} windows.", "",
          f"FALL_CONFIRMED needs the person to stay down; only {confirm_note['count']} labelled falls continue for "
          f"at least {confirm_note['falls_with_at_least_s_after_onset']:.1f} s after onset. Confirmed recall on those: "
          f"{confirm_note['confirmed_recall_on_these']}.", "",
          "| Test subject | Selected model | Threshold | Trained F1 (suspected) | Baseline F1 (suspected) |",
          "|---|---|---|---|---|"]
    for f in folds:
        L.append(f"| {f['test_subject']} | {f['selected_model']} | {f['threshold']} | "
                 f"{f['trained_suspected']['f1']:.3f} | {f['baseline_suspected']['f1']:.3f} |")
    L += ["", f"Deployed model: {card['model_type']}, threshold {card['decision_threshold']}.", "",
          "Limitations:", ""] + [f"- {x}" for x in card["limitations"]]
    path.write_text("\n".join(L) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
