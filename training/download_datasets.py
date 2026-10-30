"""Download the public dataset and pretrained models used by SentinelAI.

What it downloads
  * GMDCSA-24 fall dataset (MIT licence, ~2 GB) from the authors' official GitHub repository
    https://github.com/ekramalam/GMDCSA24-A-Dataset-for-Human-Fall-Detection-in-Videos
    -> data/raw/public_datasets/GMDCSA24/
  * Pretrained Ultralytics YOLO11n ONNX models (AGPL-3.0) from the official
    ultralytics/assets GitHub release v8.3.0
    -> artifacts/pose_estimation/yolo11n-pose.onnx, artifacts/object_detection/yolo11n.onnx

Files are verified with SHA-256 (models) / file counts (dataset).
Other datasets (UR Fall, Le2i, CAUCAFall, UP-Fall) need manual download -
see docs/DATASET_RESEARCH.md.

Usage:
    python training/download_datasets.py               # models + GMDCSA-24
    python training/download_datasets.py --models-only
"""
from __future__ import annotations

import argparse
import hashlib
import shutil
import subprocess
import tempfile
import urllib.request
import zipfile
from pathlib import Path

from common import ARTIFACTS, RAW_PUBLIC, get_logger

log = get_logger("download")

GMDCSA_REPO = "https://github.com/ekramalam/GMDCSA24-A-Dataset-for-Human-Fall-Detection-in-Videos"
GMDCSA_ZIP = GMDCSA_REPO + "/archive/refs/heads/main.zip"
MODELS = {
    "pose_estimation/yolo11n-pose.onnx": (
        "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n-pose.onnx",
        "93e2866b0ce678f99b4dd88af0c12e9ea2edf079a361dc2ecbc6226b77ff6408"),
    "object_detection/yolo11n.onnx": (
        "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.onnx",
        "634279b40c07c6391472c51ad45b81ebc48706a9a1fe72dd3396322acd0c053b"),
}


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def download(url: str, dest: Path) -> None:
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(dest.suffix + ".part")
    log.info("Downloading %s", url)
    with urllib.request.urlopen(url, timeout=60) as r, open(tmp, "wb") as fh:
        total = int(r.headers.get("Content-Length") or 0)
        done = 0
        while chunk := r.read(1 << 20):
            fh.write(chunk)
            done += len(chunk)
            if total and done % (50 << 20) < (1 << 20):
                log.info("  %.0f / %.0f MB", done / 1e6, total / 1e6)
    tmp.replace(dest)


def get_models(force: bool = False, root: Path = ARTIFACTS) -> None:
    for rel, (url, digest) in MODELS.items():
        dest = root / rel
        if dest.exists() and not force and sha256(dest) == digest:
            log.info("OK  %s (checksum verified)", rel)
            continue
        download(url, dest)
        got = sha256(dest)
        if got != digest:
            dest.unlink()
            raise SystemExit(f"Checksum mismatch for {rel}: got {got}. File removed; try again.")
        log.info("OK  %s downloaded and verified", rel)


def get_gmdcsa24(force: bool = False) -> None:
    dest = RAW_PUBLIC / "GMDCSA24"
    if dest.exists() and any(dest.glob("Subject */*.csv")) and not force:
        log.info("GMDCSA-24 already present at %s", dest)
        return
    if dest.exists():
        shutil.rmtree(dest)
    RAW_PUBLIC.mkdir(parents=True, exist_ok=True)
    if shutil.which("git"):
        log.info("Cloning GMDCSA-24 with git (about 2 GB, this can take a while)...")
        r = subprocess.run(["git", "-c", "gc.auto=0", "clone", "--depth", "1", GMDCSA_REPO, str(dest)])
        if r.returncode == 0:
            shutil.rmtree(dest / ".git", ignore_errors=True)
            _check_gmdcsa(dest)
            return
        log.warning("git clone failed (exit %d); trying the zip download", r.returncode)
        shutil.rmtree(dest, ignore_errors=True)
    with tempfile.TemporaryDirectory() as td:
        zpath = Path(td) / "gmdcsa24.zip"
        download(GMDCSA_ZIP, zpath)
        with zipfile.ZipFile(zpath) as z:
            z.extractall(td)
        inner = next(p for p in Path(td).iterdir() if p.is_dir() and p.name.startswith("GMDCSA24"))
        shutil.move(str(inner), str(dest))
    _check_gmdcsa(dest)


def _check_gmdcsa(dest: Path) -> None:
    vids = list(dest.glob("Subject */*/*.mp4"))
    csvs = list(dest.glob("Subject */*.csv"))
    log.info("GMDCSA-24: %d videos, %d annotation files in %s", len(vids), len(csvs), dest)
    if len(vids) < 150 or len(csvs) < 8:
        raise SystemExit("GMDCSA-24 looks incomplete (expected 160 videos and 8 csv files)")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--models-only", action="store_true")
    ap.add_argument("--dataset-only", action="store_true")
    ap.add_argument("--force", action="store_true", help="download again even if present")
    args = ap.parse_args()
    if not args.dataset_only:
        get_models(args.force)
    if not args.models_only:
        get_gmdcsa24(args.force)
    log.info("Next: python training/prepare_dataset.py")


if __name__ == "__main__":
    main()
