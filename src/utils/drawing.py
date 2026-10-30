"""Overlay drawing for the dashboard / preview window (pure presentation, no AI logic)."""
from __future__ import annotations

import cv2
import numpy as np

from ..detection.yolo_onnx import SKELETON

STATE_COLORS = {  # BGR
    "NORMAL": (80, 200, 80), "POSSIBLE_FALL": (0, 200, 255), "FALL_SUSPECTED": (0, 140, 255),
    "FALL_CONFIRMED": (0, 0, 255), "RECOVERY_OR_RESET": (255, 200, 0),
}


def draw_result(image: np.ndarray, result, kpt_thr: float = 0.35) -> np.ndarray:
    out = image.copy()
    h, w = out.shape[:2]
    scale = max(0.5, min(w, h) / 720.0)
    for obj in result.objects:
        x1, y1, x2, y2 = obj.box.astype(int)
        cv2.rectangle(out, (x1, y1), (x2, y2), (200, 160, 60), 1)
        _label(out, f"{obj.class_name} {obj.score:.2f}", (x1, y1), (200, 160, 60), scale * 0.8)
    for p in result.persons:
        color = STATE_COLORS.get(p.fall_state, (180, 180, 180))
        x1, y1, x2, y2 = p.box.astype(int)
        cv2.rectangle(out, (x1, y1), (x2, y2), color, max(2, int(2 * scale)))
        k = p.keypoints
        for a, b in SKELETON:
            if k[a, 2] >= kpt_thr and k[b, 2] >= kpt_thr:
                cv2.line(out, (int(k[a, 0]), int(k[a, 1])), (int(k[b, 0]), int(k[b, 1])),
                         (255, 255, 255), max(1, int(2 * scale)), cv2.LINE_AA)
        for x, y, s in k:
            if s >= kpt_thr:
                cv2.circle(out, (int(x), int(y)), max(2, int(3 * scale)), (0, 255, 255), -1, cv2.LINE_AA)
        txt = f"#{p.track_id} {p.posture} {p.posture_conf:.2f} | {p.fall_state}"
        _label(out, txt, (x1, y1), color, scale)
    status = (f"FPS {result.fps:.1f}  inference {result.inference_ms:.0f} ms  "
              f"people {len(result.persons)}  camera {result.camera_status}")
    _label(out, status, (5, int(22 * scale) + 4), (40, 40, 40), scale, top=True)
    return out


def _label(img, text, org, color, scale=1.0, top=False):
    font = cv2.FONT_HERSHEY_SIMPLEX
    fs = 0.5 * scale
    th = max(1, int(scale))
    (tw, tht), base = cv2.getTextSize(text, font, fs, th)
    x, y = int(org[0]), int(org[1])
    y = max(y, tht + base + 2)
    cv2.rectangle(img, (x, y - tht - base - 2), (x + tw + 4, y), color, -1)
    cv2.putText(img, text, (x + 2, y - base), font, fs, (255, 255, 255), th, cv2.LINE_AA)
