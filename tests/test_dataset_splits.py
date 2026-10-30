"""Data hygiene: subject-level splits never leak a person between training and testing,
the inventory matches the index, and the model registry matches the deployed files."""
import csv
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
SPLITS = ROOT / "data" / "splits" / "splits.json"
INDEX = ROOT / "data" / "annotations" / "video_index.csv"
INVENTORY = ROOT / "data" / "dataset_inventory.csv"

pytestmark = pytest.mark.skipif(not SPLITS.exists() or not INDEX.exists(), reason="dataset not prepared")


def _index():
    with open(INDEX, newline="", encoding="utf-8") as fh:
        return {r["video_id"]: r for r in csv.DictReader(fh)}


def _splits():
    return json.loads(SPLITS.read_text(encoding="utf-8"))


def test_loso_folds_are_subject_disjoint_and_cover_every_video_once():
    idx, sp = _index(), _splits()
    seen = []
    for fold in sp["loso_folds"]:
        s = fold["test_subject"]
        assert s not in fold["train_subjects"]
        assert {idx[v]["subject"] for v in fold["test"]} == {s}
        seen += fold["test"]
    assert sorted(seen) == sorted(idx)                  # every video is tested exactly once
    assert len(seen) == len(set(seen))


def test_fixed_split_is_subject_disjoint():
    idx, sp = _index(), _splits()
    parts = {k: set(sp["fixed"][k]) for k in ("train", "validation", "test")}
    for a in parts:
        for b in parts:
            if a < b:
                assert not parts[a] & parts[b], (a, b)
    subj = {k: {idx[v]["subject"] for v in vids} for k, vids in parts.items()}
    assert not subj["train"] & subj["test"] and not subj["train"] & subj["validation"]
    assert not subj["validation"] & subj["test"]
    assert set().union(*parts.values()) == set(idx)


def test_excluded_videos_never_appear_in_splits():
    report = json.loads((ROOT / "data" / "annotations" / "prepare_report.json").read_text(encoding="utf-8"))
    excluded = {i["video_id"] for ds in report.values() if isinstance(ds, dict)
                for i in ds.get("issues", []) if i.get("action") == "EXCLUDED"}
    sp = _splits()
    all_split = {v for f in sp["loso_folds"] for v in f["test"]}
    assert excluded and not excluded & all_split


@pytest.mark.skipif(not INVENTORY.exists(), reason="run training/build_inventory.py")
def test_inventory_matches_index():
    idx = _index()
    with open(INVENTORY, newline="", encoding="utf-8") as fh:
        inv = list(csv.DictReader(fh))
    used = {r["video_id"]: r for r in inv if r["status"] == "used"}
    assert set(used) == set(idx)
    for vid, r in used.items():
        assert r["subject"] == idx[vid]["subject"] and r["loso_test_fold"] == idx[vid]["subject"]
        assert r["license"] and r["lighting_group"] in ("day_lit", "day_dim", "night", "unknown")
    assert any(r["status"] == "excluded" for r in inv)  # nothing silently disappears


def test_model_registry_matches_deployed_files():
    import sys
    sys.path.insert(0, str(ROOT / "training"))
    import model_registry
    if not model_registry.REGISTRY.exists():
        pytest.skip("no registry yet")
    assert model_registry.verify() == []
    reg = model_registry.load()
    for kind in ("posture_classifier", "fall_classifier"):
        assert reg["models"][kind]["deployed"] in reg["models"][kind]["versions"]
