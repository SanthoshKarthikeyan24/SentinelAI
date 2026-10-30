"""Model registry: every trained model is kept as a numbered version; deploying is explicit.

Layout
  artifacts/<kind>/versions/<version>/<model file> + model_card.json   (never overwritten)
  artifacts/<kind>/<model file> + model_card.json                       (the DEPLOYED copy the app loads)
  artifacts/model_registry.json                                          (versions, sha256, deploy history)

Training scripts call ``save_version`` (always) and ``deploy`` only when run with
``--deploy``. So a new training run can never silently replace the model the app uses.

CLI
  python training/model_registry.py list
  python training/model_registry.py deploy posture_classifier <version>
  python training/model_registry.py verify          # deployed files match their registered sha256
  python training/model_registry.py adopt           # register models that exist but are not registered yet
"""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
ARTIFACTS = ROOT / "artifacts"
REGISTRY = ARTIFACTS / "model_registry.json"
KINDS = {"posture_classifier": "posture_model.joblib", "fall_classifier": "fall_model.joblib",
         "pose_estimation": "yolo11n-pose.onnx", "object_detection": "yolo11n.onnx"}


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def load() -> dict:
    if REGISTRY.exists():
        return json.loads(REGISTRY.read_text(encoding="utf-8"))
    return {"note": "managed by training/model_registry.py - do not edit by hand", "models": {}}


def _save(reg: dict) -> None:
    REGISTRY.write_text(json.dumps(reg, indent=2) + "\n", encoding="utf-8")


def _now() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def new_version_id(kind: str) -> str:
    reg = load()
    n = len(reg["models"].get(kind, {}).get("versions", {})) + 1
    return f"v{n}-{datetime.now(timezone.utc):%Y%m%d-%H%M%S}"


def save_version(kind: str, model_file: Path, card: dict | None, summary: dict | None = None,
                 version: str | None = None) -> str:
    """Copy a freshly written model (and card) into versions/<id>/ and register it. Returns the id."""
    version = version or new_version_id(kind)
    vdir = ARTIFACTS / kind / "versions" / version
    if vdir.exists():
        raise FileExistsError(f"{vdir} already exists - versions are never overwritten")
    vdir.mkdir(parents=True)
    shutil.copy2(model_file, vdir / KINDS[kind])
    if card is not None:
        card = dict(card, registry_version=version)
        (vdir / "model_card.json").write_text(json.dumps(card, indent=2, default=str) + "\n", encoding="utf-8")
    reg = load()
    entry = reg["models"].setdefault(kind, {"file": KINDS[kind], "deployed": None, "versions": {},
                                            "deploy_history": []})
    entry["versions"][version] = {"created": _now(), "sha256": sha256(vdir / KINDS[kind]),
                                  "path": str((vdir / KINDS[kind]).relative_to(ROOT)).replace("\\", "/"),
                                  "summary": summary or {}}
    _save(reg)
    return version


def deploy(kind: str, version: str, reason: str = "") -> None:
    """Make <version> the model the app loads (copies it to artifacts/<kind>/)."""
    reg = load()
    entry = reg["models"].get(kind)
    if not entry or version not in entry["versions"]:
        raise KeyError(f"{kind} version {version} is not registered")
    vdir = ARTIFACTS / kind / "versions" / version
    src = vdir / KINDS[kind]
    if sha256(src) != entry["versions"][version]["sha256"]:
        raise ValueError(f"{src} does not match its registered sha256 - refusing to deploy")
    shutil.copy2(src, ARTIFACTS / kind / KINDS[kind])
    if (vdir / "model_card.json").exists():
        shutil.copy2(vdir / "model_card.json", ARTIFACTS / kind / "model_card.json")
    entry["deploy_history"].append({"version": version, "deployed": _now(), "previous": entry["deployed"],
                                    "reason": reason})
    entry["deployed"] = version
    _save(reg)


def deployed_versions() -> dict:
    """{kind: {"version", "sha256_ok"}} for the dashboard / app status (never raises)."""
    out = {}
    try:
        reg = load()
    except (OSError, ValueError):
        return out
    for kind, e in reg.get("models", {}).items():
        v = e.get("deployed")
        if not v:
            continue
        p = ARTIFACTS / kind / KINDS.get(kind, e.get("file", ""))
        ok = p.exists() and sha256(p) == e["versions"].get(v, {}).get("sha256")
        out[kind] = {"version": v, "sha256_ok": ok}
    return out


def verify() -> list[str]:
    problems = []
    reg = load()
    for kind, e in reg.get("models", {}).items():
        for v, info in e["versions"].items():
            p = ROOT / info["path"]
            if not p.exists():
                problems.append(f"{kind} {v}: file missing ({info['path']})")
            elif sha256(p) != info["sha256"]:
                problems.append(f"{kind} {v}: sha256 mismatch")
        if e.get("deployed"):
            p = ARTIFACTS / kind / e["file"]
            if not p.exists() or sha256(p) != e["versions"][e["deployed"]]["sha256"]:
                problems.append(f"{kind}: deployed file differs from registered version {e['deployed']}")
    return problems


def adopt() -> list[str]:
    """Register deployed models that exist on disk but are not in the registry yet (first run)."""
    done = []
    reg = load()
    for kind, fname in KINDS.items():
        p = ARTIFACTS / kind / fname
        if not p.exists() or reg["models"].get(kind, {}).get("deployed"):
            continue
        card_p = ARTIFACTS / kind / "model_card.json"
        card = json.loads(card_p.read_text(encoding="utf-8")) if card_p.exists() else None
        summary = {}
        if card and "evaluation" in card:
            ev = card["evaluation"]
            if "selected" in ev:
                summary = {"loso_accuracy": ev["selected"].get("accuracy"), "loso_macro_f1": ev["selected"].get("macro_f1")}
            elif "pooled" in ev:
                tr = ev["pooled"].get("trained", {})
                summary = {lvl: {"f1": d.get("f1"), "precision": d.get("precision"), "recall": d.get("recall")}
                           for lvl, d in tr.items()}
        if kind in ("pose_estimation", "object_detection"):
            summary = {"source": "Ultralytics YOLO11n COCO weights exported to ONNX (not trained here)"}
        v = save_version(kind, p, card, summary, version="v1-initial")
        reg = load()
        reg["models"][kind]["deployed"] = v
        reg["models"][kind]["deploy_history"].append({"version": v, "deployed": _now(), "previous": None,
                                                      "reason": "adopted the model trained before the registry existed"})
        _save(reg)
        done.append(f"{kind} -> {v}")
    return done


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list")
    sub.add_parser("verify")
    sub.add_parser("adopt")
    d = sub.add_parser("deploy")
    d.add_argument("kind", choices=list(KINDS))
    d.add_argument("version")
    d.add_argument("--reason", default="manual deploy")
    a = ap.parse_args()
    if a.cmd == "list":
        for kind, e in load()["models"].items():
            print(f"{kind}: deployed {e['deployed']}")
            for v, info in e["versions"].items():
                print(f"   {v}  {info['created']}  sha256 {info['sha256'][:12]}  {info.get('summary', {})}")
    elif a.cmd == "verify":
        problems = verify()
        print("\n".join(problems) if problems else "all registered models verified")
        return 1 if problems else 0
    elif a.cmd == "adopt":
        print("\n".join(adopt()) or "nothing to adopt")
    elif a.cmd == "deploy":
        deploy(a.kind, a.version, a.reason)
        print(f"deployed {a.kind} {a.version}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
