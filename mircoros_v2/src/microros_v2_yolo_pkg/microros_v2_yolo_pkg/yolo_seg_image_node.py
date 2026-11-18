#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO instance segmentation for one image.
Environment: Ubuntu 22 + ROS2 Humble + CPU + Ultralytics .pt model.

参数设计：
- 默认只需要通过 model_name 切换模型，例如 yolo26n-seg.pt 或 yolo26n-seg。
- 默认只需要通过 image_name 切换测试图片，例如 zidane.jpg。
- 仍保留 model_path / image_path，支持绝对路径；如果传入的只是文件名，也会自动拼接默认目录。
"""

from __future__ import annotations

from pathlib import Path
from typing import Iterable

import cv2
import rclpy
from rclpy.node import Node
from ultralytics import YOLO


DEFAULT_BASE = "/home/yahboom"
DEFAULT_MODEL_DIR = f"{DEFAULT_BASE}/MODELS/yolo"
DEFAULT_MODEL_NAME = "yolo11n-seg.pt"
DEFAULT_IMAGE_DIR = f"{DEFAULT_BASE}/MODELS/yolo/assets"
DEFAULT_IMAGE_NAME = "zidane.jpg"
DEFAULT_OUTPUT_DIR = f"{DEFAULT_BASE}/MODELS/yolo/assets/output"
IMAGE_EXTS = (".jpg", ".jpeg", ".png", ".bmp", ".webp")


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _safe_float_text(value: float) -> str:
    return (f"{value:.4f}".rstrip("0").rstrip(".")).replace(".", "p")


def _model_stem(model_path: str) -> str:
    return Path(model_path).stem


def _ensure_suffix(path_or_name: str, suffix: str) -> str:
    p = Path(path_or_name)
    if p.suffix:
        return path_or_name
    return f"{path_or_name}{suffix}"


def _resolve_file(base_dir: str, name_or_path: str, default_name: str, default_ext: str | None = None) -> str:
    """Resolve either absolute path or filename under base_dir."""
    value = str(name_or_path).strip()
    if not value:
        value = default_name
    if default_ext:
        value = _ensure_suffix(value, default_ext)

    p = Path(value).expanduser()
    # 如果是绝对路径，或者带有目录层级，就按路径处理；否则拼接默认目录。
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


class YoloSegImageNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_seg_image_node")

        self.declare_parameter("model_dir", DEFAULT_MODEL_DIR)
        self.declare_parameter("model_name", DEFAULT_MODEL_NAME)
        self.declare_parameter("model_path", "")
        self.declare_parameter("raw_image_dir", DEFAULT_IMAGE_DIR)
        self.declare_parameter("image_name", DEFAULT_IMAGE_NAME)
        self.declare_parameter("image_path", "")
        self.declare_parameter("output_dir", DEFAULT_OUTPUT_DIR)
        self.declare_parameter("imgsz", 960)
        self.declare_parameter("conf", 0.05)
        self.declare_parameter("iou", 0.7)
        self.declare_parameter("max_det", 1000)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("show", True)
        self.declare_parameter("save", True)
        self.declare_parameter("window_width", 1280)
        self.declare_parameter("window_height", 720)

    def _get_param(self, name: str):
        return self.get_parameter(name).value

    def resolve_model_path(self) -> str:
        model_dir = str(self._get_param("model_dir")).strip()
        model_path = str(self._get_param("model_path")).strip()
        model_name = str(self._get_param("model_name")).strip()

        # model_path 优先；若 model_path 只是文件名，也自动拼接 model_dir。
        if model_path:
            return _resolve_file(model_dir, model_path, DEFAULT_MODEL_NAME, default_ext=".pt")
        return _resolve_file(model_dir, model_name, DEFAULT_MODEL_NAME, default_ext=".pt")

    def resolve_image_path(self) -> str:
        raw_image_dir = str(self._get_param("raw_image_dir")).strip()
        image_path = str(self._get_param("image_path")).strip()
        image_name = str(self._get_param("image_name")).strip()

        # image_path 优先；若 image_path 只是文件名，也自动拼接 raw_image_dir。
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
        conf = float(self._get_param("conf"))
        iou = float(self._get_param("iou"))
        max_det = int(self._get_param("max_det"))
        device = str(self._get_param("device")).strip()
        show = _as_bool(self._get_param("show"))
        save = _as_bool(self._get_param("save"))
        window_width = int(self._get_param("window_width"))
        window_height = int(self._get_param("window_height"))

        image_file = Path(image_path).expanduser()
        model_file = Path(model_path).expanduser()
        out_dir = Path(output_dir).expanduser()
        out_dir.mkdir(parents=True, exist_ok=True)

        if not model_file.exists():
            raise FileNotFoundError(f"Model file does not exist: {model_file}")
        if not image_file.exists():
            raise FileNotFoundError(f"Image file does not exist: {image_file}")

        model_name = _model_stem(str(model_file))
        conf_text = _safe_float_text(conf)
        output_name = f"{image_file.stem}_{model_name}_seg_imgsz{imgsz}_conf{conf_text}{image_file.suffix}"
        output_path = out_dir / output_name

        self.get_logger().info(f"Loading model: {model_file}")
        model = YOLO(str(model_file))

        self.get_logger().info(
            f"Running image segmentation: image={image_file}, imgsz={imgsz}, "
            f"conf={conf}, iou={iou}, max_det={max_det}, device={device}"
        )

        results = model.predict(
            source=str(image_file),
            imgsz=imgsz,
            conf=conf,
            iou=iou,
            max_det=max_det,
            device=device,
            verbose=False,
        )

        if not results:
            self.get_logger().error("No result returned by Ultralytics.")
            return

        result = results[0]
        annotated = result.plot()
        num_boxes = 0 if result.boxes is None else len(result.boxes)
        num_masks = 0 if result.masks is None else len(result.masks)
        self.get_logger().info(f"Detected boxes={num_boxes}, masks={num_masks}")

        if save:
            ok = cv2.imwrite(str(output_path), annotated)
            if not ok:
                raise RuntimeError(f"Failed to save output image: {output_path}")
            self.get_logger().info(f"Saved output image: {output_path}")

        if show:
            display = annotated
            if window_width > 0 and window_height > 0:
                display = cv2.resize(annotated, (window_width, window_height))
            cv2.imshow("YOLO Seg Image", display)
            self.get_logger().info("Press any key in the OpenCV window to close it.")
            cv2.waitKey(0)
            cv2.destroyAllWindows()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloSegImageNode()
    try:
        node.run_once()
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(str(exc))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
