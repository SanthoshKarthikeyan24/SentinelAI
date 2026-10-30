"""Pretrained YOLO11 person-pose and object detectors running on ONNX Runtime.

Why ONNX Runtime instead of PyTorch?
  * The exported models run on Windows CPU with a small install (no PyTorch).
  * Training-time pose extraction and live inference use *exactly* the same
    code, so the features the classifiers were trained on match runtime.

Models (pretrained by Ultralytics on COCO, not trained by this project):
  * yolo11n-pose.onnx : person boxes + 17 COCO keypoints
  * yolo11n.onnx      : 80 COCO object classes (chair, couch, bed, ...)
Both are AGPL-3.0 licensed by Ultralytics (fine for an academic project).
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

try:
    import onnxruntime as ort
except ImportError as exc:  # pragma: no cover - environment problem
    raise ImportError("onnxruntime is required: pip install onnxruntime") from exc

KEYPOINT_NAMES = [
    "nose", "left_eye", "right_eye", "left_ear", "right_ear",
    "left_shoulder", "right_shoulder", "left_elbow", "right_elbow",
    "left_wrist", "right_wrist", "left_hip", "right_hip",
    "left_knee", "right_knee", "left_ankle", "right_ankle",
]
KP = {name: i for i, name in enumerate(KEYPOINT_NAMES)}

# Lines drawn between keypoints to form the skeleton.
SKELETON = [
    (15, 13), (13, 11), (16, 14), (14, 12), (11, 12), (5, 11), (6, 12),
    (5, 6), (5, 7), (6, 8), (7, 9), (8, 10), (1, 2), (0, 1), (0, 2),
    (1, 3), (2, 4), (3, 5), (4, 6),
]

COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush",
]


@dataclass
class PersonDetection:
    """One detected person."""
    box: np.ndarray            # [x1, y1, x2, y2] in original image pixels
    score: float               # detector confidence score (not a calibrated probability)
    keypoints: np.ndarray      # (17, 3): x, y, keypoint score
    track_id: int | None = None


@dataclass
class ObjectDetection:
    box: np.ndarray
    score: float
    class_id: int
    class_name: str


@dataclass
class InferenceTiming:
    preprocess_ms: float = 0.0
    inference_ms: float = 0.0
    postprocess_ms: float = 0.0
    extra: dict = field(default_factory=dict)

    @property
    def total_ms(self) -> float:
        return self.preprocess_ms + self.inference_ms + self.postprocess_ms


def create_session(model_path: str | Path, providers: list[str] | None = None,
                   num_threads: int = 0) -> "ort.InferenceSession":
    """Create an ONNX Runtime session using the first available providers."""
    model_path = Path(model_path)
    if not model_path.exists():
        raise FileNotFoundError(
            f"Model file not found: {model_path}. See docs/TROUBLESHOOTING.md (missing models).")
    available = set(ort.get_available_providers())
    wanted = [p for p in (providers or ["CPUExecutionProvider"]) if p in available]
    if "CPUExecutionProvider" not in wanted:
        wanted.append("CPUExecutionProvider")
    opts = ort.SessionOptions()
    if num_threads and num_threads > 0:
        opts.intra_op_num_threads = int(num_threads)
    opts.log_severity_level = 3
    return ort.InferenceSession(str(model_path), sess_options=opts, providers=wanted)


def letterbox(image: np.ndarray, size: int) -> tuple[np.ndarray, float, tuple[float, float]]:
    """Resize keeping aspect ratio and pad to a size x size square (grey 114)."""
    h, w = image.shape[:2]
    scale = min(size / h, size / w)
    new_w, new_h = int(round(w * scale)), int(round(h * scale))
    resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    pad_w, pad_h = (size - new_w) / 2.0, (size - new_h) / 2.0
    top, bottom = int(round(pad_h - 0.1)), int(round(pad_h + 0.1))
    left, right = int(round(pad_w - 0.1)), int(round(pad_w + 0.1))
    padded = cv2.copyMakeBorder(resized, top, bottom, left, right, cv2.BORDER_CONSTANT,
                                value=(114, 114, 114))
    return padded, scale, (left, top)


def _to_blob(image: np.ndarray) -> np.ndarray:
    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    return np.ascontiguousarray(rgb.transpose(2, 0, 1)[None].astype(np.float32) / 255.0)


def _nms(boxes_xyxy: np.ndarray, scores: np.ndarray, score_thr: float, iou_thr: float) -> np.ndarray:
    if len(boxes_xyxy) == 0:
        return np.empty(0, dtype=int)
    xywh = np.column_stack([boxes_xyxy[:, 0], boxes_xyxy[:, 1],
                            boxes_xyxy[:, 2] - boxes_xyxy[:, 0], boxes_xyxy[:, 3] - boxes_xyxy[:, 1]])
    idx = cv2.dnn.NMSBoxes(xywh.tolist(), scores.astype(float).tolist(), float(score_thr), float(iou_thr))
    return np.array(idx, dtype=int).reshape(-1)


def _xywh_to_xyxy(b: np.ndarray) -> np.ndarray:
    out = np.empty_like(b)
    out[:, 0] = b[:, 0] - b[:, 2] / 2
    out[:, 1] = b[:, 1] - b[:, 3] / 2
    out[:, 2] = b[:, 0] + b[:, 2] / 2
    out[:, 3] = b[:, 1] + b[:, 3] / 2
    return out


class _YoloOnnxBase:
    def __init__(self, model_path, imgsz=640, providers=None, num_threads=0):
        self.session = create_session(model_path, providers, num_threads)
        inp = self.session.get_inputs()[0]
        self.input_name = inp.name
        shape = inp.shape
        # Exported models have a fixed input size; respect it.
        if isinstance(shape[2], int) and shape[2] > 0:
            imgsz = int(shape[2])
        self.imgsz = int(imgsz)
        self.providers = self.session.get_providers()
        self.last_timing = InferenceTiming()

    def _run(self, image: np.ndarray):
        t0 = time.perf_counter()
        padded, scale, (pad_x, pad_y) = letterbox(image, self.imgsz)
        blob = _to_blob(padded)
        t1 = time.perf_counter()
        out = self.session.run(None, {self.input_name: blob})[0][0].T  # (8400, C)
        t2 = time.perf_counter()
        self.last_timing = InferenceTiming((t1 - t0) * 1000, (t2 - t1) * 1000, 0.0)
        return out, scale, pad_x, pad_y, t2

    @staticmethod
    def _unletterbox(xy: np.ndarray, scale: float, pad_x: float, pad_y: float,
                     w: int, h: int) -> np.ndarray:
        xy = xy.copy()
        xy[..., 0] = np.clip((xy[..., 0] - pad_x) / scale, 0, w - 1)
        xy[..., 1] = np.clip((xy[..., 1] - pad_y) / scale, 0, h - 1)
        return xy


class PoseDetector(_YoloOnnxBase):
    """YOLO11-pose: detects people and their 17 COCO keypoints."""

    def __init__(self, model_path, imgsz=640, conf=0.4, iou=0.5, providers=None, num_threads=0):
        super().__init__(model_path, imgsz, providers, num_threads)
        self.conf = float(conf)
        self.iou = float(iou)
        n_out = self.session.get_outputs()[0].shape[1]
        if n_out != 56:
            raise ValueError(f"{model_path} does not look like a 17-keypoint pose model "
                             f"(output channels={n_out}, expected 56)")

    def detect(self, image: np.ndarray) -> list[PersonDetection]:
        if image is None or image.size == 0:
            return []
        h, w = image.shape[:2]
        out, scale, pad_x, pad_y, t2 = self._run(image)
        scores = out[:, 4]
        keep = scores >= self.conf
        out, scores = out[keep], scores[keep]
        results: list[PersonDetection] = []
        if len(out):
            boxes = _xywh_to_xyxy(out[:, :4])
            idx = _nms(boxes, scores, self.conf, self.iou)
            for i in idx:
                box = self._unletterbox(boxes[i].reshape(2, 2), scale, pad_x, pad_y, w, h).reshape(4)
                kpts = out[i, 5:].reshape(17, 3).copy()
                kpts[:, :2] = self._unletterbox(kpts[:, :2], scale, pad_x, pad_y, w, h)
                results.append(PersonDetection(box=box.astype(np.float32), score=float(scores[i]),
                                               keypoints=kpts.astype(np.float32)))
        results.sort(key=lambda d: d.score, reverse=True)
        self.last_timing.postprocess_ms = (time.perf_counter() - t2) * 1000
        return results


class ObjectDetector(_YoloOnnxBase):
    """YOLO11 COCO object detector, filtered to the classes listed in config."""

    def __init__(self, model_path, imgsz=640, conf=0.4, iou=0.5, classes=None,
                 providers=None, num_threads=0):
        super().__init__(model_path, imgsz, providers, num_threads)
        self.conf = float(conf)
        self.iou = float(iou)
        n_cls = self.session.get_outputs()[0].shape[1] - 4
        if n_cls != len(COCO_CLASSES):
            raise ValueError(f"{model_path}: expected 80 COCO classes, model has {n_cls}")
        unknown = [c for c in (classes or []) if c not in COCO_CLASSES]
        if unknown:
            raise ValueError(f"Unknown object classes in config: {unknown}. "
                             f"Valid names are the 80 COCO classes.")
        self.class_ids = (np.array([COCO_CLASSES.index(c) for c in classes], dtype=int)
                          if classes else np.arange(len(COCO_CLASSES)))

    def detect(self, image: np.ndarray) -> list[ObjectDetection]:
        if image is None or image.size == 0:
            return []
        h, w = image.shape[:2]
        out, scale, pad_x, pad_y, t2 = self._run(image)
        cls_scores = out[:, 4:][:, self.class_ids]
        best = cls_scores.argmax(axis=1)
        scores = cls_scores[np.arange(len(best)), best]
        keep = scores >= self.conf
        results: list[ObjectDetection] = []
        if keep.any():
            boxes = _xywh_to_xyxy(out[keep, :4])
            scores_k, cls_k = scores[keep], self.class_ids[best[keep]]
            # class-aware NMS: offset boxes per class so different classes never suppress each other
            offset = boxes + (cls_k[:, None] * 4096.0)
            for i in _nms(offset, scores_k, self.conf, self.iou):
                box = self._unletterbox(boxes[i].reshape(2, 2), scale, pad_x, pad_y, w, h).reshape(4)
                cid = int(cls_k[i])
                results.append(ObjectDetection(box.astype(np.float32), float(scores_k[i]), cid,
                                               COCO_CLASSES[cid]))
        self.last_timing.postprocess_ms = (time.perf_counter() - t2) * 1000
        return results
