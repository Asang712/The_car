#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO image classification for one image.
Environment: Ubuntu 22 + ROS2 Humble + CPU + Ultralytics .pt model.

Usage design:
- Switch model with model_name, e.g. yolo11s-cls or yolo11s-cls.pt.
- Switch test image with image_name, e.g. dog.jpg.
- model_path / image_path are still supported and have higher priority.
"""

from __future__ import annotations

from pathlib import Path
from typing import Iterable, List, Tuple

import cv2
import rclpy
from rclpy.node import Node
from ultralytics import YOLO


DEFAULT_BASE = "/home/yahboom"
DEFAULT_MODEL_DIR = f"{DEFAULT_BASE}/MODELS/yolo"
DEFAULT_MODEL_NAME = "yolo11n-cls.pt"
DEFAULT_IMAGE_DIR = f"{DEFAULT_BASE}/MODELS/yolo/assets"
DEFAULT_IMAGE_NAME = "dog.jpg"
DEFAULT_OUTPUT_DIR = f"{DEFAULT_BASE}/MODELS/yolo/assets/output"
IMAGE_EXTS = (".jpg", ".jpeg", ".png", ".bmp", ".webp")


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _model_stem(model_path: str) -> str:
    return Path(model_path).stem


def _ensure_suffix(path_or_name: str, suffix: str) -> str:
    p = Path(path_or_name)
    if p.suffix:
        return path_or_name
    return f"{path_or_name}{suffix}"


def _resolve_file(
    base_dir: str,
    name_or_path: str,
    default_name: str,
    default_ext: str | None = None,
) -> str:
    """Resolve either absolute path or filename under base_dir."""
    value = str(name_or_path).strip()
    if not value:
        value = default_name
    if default_ext:
        value = _ensure_suffix(value, default_ext)

    p = Path(value).expanduser()
    # If absolute path or path with subdirectories is given, use it directly.
    if p.is_absolute() or len(p.parts) > 1:
        return str(p)
    return str(Path(base_dir).expanduser() / p)


def _find_first_file(directory: str, exts: Iterable[str]) -> str:
    d = Path(directory).expanduser()
    if not d.exists():
        raise FileNotFoundError(f"Input directory does not exist: {d}")

    files = []
    for ext in exts:
        files.extend(sorted(d.glob(f"*{ext}")))
        files.extend(sorted(d.glob(f"*{ext.upper()}")))

    if not files:
        raise FileNotFoundError(f"No image file found in: {d}")
    return str(files[0])


def _topk_lines(result, topk: int) -> List[Tuple[str, float]]:
    """Return [(class_name, confidence), ...] from an Ultralytics classification result."""
    if result is None or getattr(result, "probs", None) is None:
        return []

    probs = result.probs
    names = getattr(result, "names", {}) or {}
    topk = max(1, int(topk))

    try:
        indices = list(probs.top5[:topk])
        confs = probs.top5conf[:topk]
        if hasattr(confs, "detach"):
            confs = confs.detach().cpu().tolist()
        else:
            confs = list(confs)
    except Exception:  # noqa: BLE001
        data = probs.data
        order = data.argsort(descending=True)[:topk]
        indices = order.detach().cpu().tolist()
        confs = data[order].detach().cpu().tolist()

    lines: List[Tuple[str, float]] = []
    for idx, conf in zip(indices, confs):
        idx = int(idx)
        name = str(names.get(idx, idx)) if isinstance(names, dict) else str(idx)
        lines.append((name, float(conf)))
    return lines


def _draw_classification_overlay(
    frame,
    lines: List[Tuple[str, float]],
    title: str | None = None,
    font_scale: float = 1.1,
    thickness: int = 3,
):
    """Draw top-k classification text on a frame/image."""
    annotated = frame.copy()
    display_lines = []
    if title:
        display_lines.append(title)
    if lines:
        display_lines.extend([f"{name}: {conf:.2f}" for name, conf in lines])
    else:
        display_lines.append("No classification result")

    font_scale = max(0.1, float(font_scale))
    thickness = max(1, int(thickness))
    font = cv2.FONT_HERSHEY_SIMPLEX
    padding = max(12, int(round(12 * font_scale)))
    line_padding = max(8, int(round(8 * font_scale)))

    # Background rectangle for readability.
    max_width = 0
    max_height = 0
    for text in display_lines:
        (tw, th), _ = cv2.getTextSize(text, font, font_scale, thickness)
        max_width = max(max_width, tw)
        max_height = max(max_height, th)
    line_gap = max_height + line_padding
    box_width = min(frame.shape[1], max_width + 2 * padding)
    box_height = min(
        frame.shape[0],
        line_gap * len(display_lines) + 2 * padding,
    )
    cv2.rectangle(annotated, (0, 0), (box_width, box_height), (0, 0, 0), -1)

    for i, text in enumerate(display_lines):
        y = padding + max_height + i * line_gap
        cv2.putText(
            annotated,
            text,
            (padding, y),
            font,
            font_scale,
            (255, 255, 255),
            thickness,
            cv2.LINE_AA,
        )
    return annotated


class YoloClsImageNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_cls_image_node")

        self.declare_parameter("model_dir", DEFAULT_MODEL_DIR)
        self.declare_parameter("model_name", DEFAULT_MODEL_NAME)
        self.declare_parameter("model_path", "")
        self.declare_parameter("raw_image_dir", DEFAULT_IMAGE_DIR)
        self.declare_parameter("image_name", DEFAULT_IMAGE_NAME)
        self.declare_parameter("image_path", "")
        self.declare_parameter("output_dir", DEFAULT_OUTPUT_DIR)
        self.declare_parameter("imgsz", 224)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("topk", 5)
        self.declare_parameter("show", True)
        self.declare_parameter("save", True)
        self.declare_parameter("window_width", 1280)
        self.declare_parameter("window_height", 720)
        self.declare_parameter("overlay_font_scale", 1.1)
        self.declare_parameter("overlay_thickness", 3)

    def _get_param(self, name: str):
        return self.get_parameter(name).value

    def resolve_model_path(self) -> str:
        model_dir = str(self._get_param("model_dir")).strip()
        model_path = str(self._get_param("model_path")).strip()
        model_name = str(self._get_param("model_name")).strip()

        if model_path:
            return _resolve_file(model_dir, model_path, DEFAULT_MODEL_NAME, default_ext=".pt")
        return _resolve_file(model_dir, model_name, DEFAULT_MODEL_NAME, default_ext=".pt")

    def resolve_image_path(self) -> str:
        raw_image_dir = str(self._get_param("raw_image_dir")).strip()
        image_path = str(self._get_param("image_path")).strip()
        image_name = str(self._get_param("image_name")).strip()

        if image_path:
            return _resolve_file(raw_image_dir, image_path, DEFAULT_IMAGE_NAME)
        if image_name:
            return _resolve_file(raw_image_dir, image_name, DEFAULT_IMAGE_NAME)
        return _find_first_file(raw_image_dir, IMAGE_EXTS)

    def run_once(self) -> None:
        model_path = self.resolve_model_path()
        image_path = self.resolve_image_path()
        output_dir = str(self._get_param("output_dir")).strip()
        imgsz = int(self._get_param("imgsz"))
        device = str(self._get_param("device")).strip()
        topk = int(self._get_param("topk"))
        show = _as_bool(self._get_param("show"))
        save = _as_bool(self._get_param("save"))
        window_width = int(self._get_param("window_width"))
        window_height = int(self._get_param("window_height"))
        overlay_font_scale = float(self._get_param("overlay_font_scale"))
        overlay_thickness = int(self._get_param("overlay_thickness"))

        image_file = Path(image_path).expanduser()
        model_file = Path(model_path).expanduser()
        out_dir = Path(output_dir).expanduser()
        out_dir.mkdir(parents=True, exist_ok=True)

        if not model_file.exists():
            raise FileNotFoundError(f"Model file does not exist: {model_file}")
        if not image_file.exists():
            raise FileNotFoundError(f"Image file does not exist: {image_file}")

        model_name = _model_stem(str(model_file))
        output_name = f"{image_file.stem}_{model_name}_cls_imgsz{imgsz}_top{topk}{image_file.suffix}"
        output_path = out_dir / output_name

        self.get_logger().info(f"Loading model: {model_file}")
        model = YOLO(str(model_file))

        self.get_logger().info(
            f"Running image classification: image={image_file}, imgsz={imgsz}, "
            f"topk={topk}, device={device}"
        )

        results = model.predict(source=str(image_file), imgsz=imgsz, device=device, verbose=False)
        if not results:
            self.get_logger().error("No result returned by Ultralytics.")
            return

        result = results[0]
        src = cv2.imread(str(image_file))
        if src is None:
            raise RuntimeError(f"Failed to read image with OpenCV: {image_file}")

        lines = _topk_lines(result, topk)
        title = f"{model_name}  top{topk}"
        source_scale = max(
            1.0,
            min(src.shape[1] / 1280.0, src.shape[0] / 720.0),
        )
        annotated = _draw_classification_overlay(
            src,
            lines,
            title,
            font_scale=overlay_font_scale * source_scale,
            thickness=round(overlay_thickness * source_scale),
        )

        if lines:
            self.get_logger().info("Top classification results: " + ", ".join([f"{n}={c:.3f}" for n, c in lines]))

        if save:
            ok = cv2.imwrite(str(output_path), annotated)
            if not ok:
                raise RuntimeError(f"Failed to save output image: {output_path}")
            self.get_logger().info(f"Saved output image: {output_path}")

        if show:
            display = src
            if window_width > 0 and window_height > 0:
                display = cv2.resize(src, (window_width, window_height))
            display = _draw_classification_overlay(
                display,
                lines,
                title,
                font_scale=overlay_font_scale,
                thickness=overlay_thickness,
            )
            cv2.imshow("YOLO Classification Image", display)
            self.get_logger().info("Press any key in the OpenCV window to close it.")
            cv2.waitKey(0)
            cv2.destroyAllWindows()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloClsImageNode()
    try:
        node.run_once()
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(str(exc))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
