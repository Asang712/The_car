#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO oriented bounding box detection for local video files.
Environment: Ubuntu 22 + ROS2 Humble + CPU + Ultralytics .pt model.

Usage design:
- Switch model by model_name, e.g. yolo11n-obb or yolo11n-obb.pt.
- Switch test video by video_name, e.g. street.mp4.
- model_path/video_path are still supported. If a filename is passed, it is resolved under the default directory.
"""

from __future__ import annotations

from pathlib import Path
from typing import Iterable, List, Optional

import cv2
import rclpy
from rclpy.node import Node
from ultralytics import YOLO


DEFAULT_BASE = "/home/yahboom"
DEFAULT_MODEL_DIR = f"{DEFAULT_BASE}/MODELS/yolo"
DEFAULT_MODEL_NAME = "yolo11n-obb.pt"
DEFAULT_VIDEO_DIR = f"{DEFAULT_BASE}/MODELS/yolo/videos"
DEFAULT_VIDEO_NAME = "street.mp4"
DEFAULT_OUTPUT_DIR = f"{DEFAULT_BASE}/MODELS/yolo/videos/output"
VIDEO_EXTS = (".mp4", ".avi", ".mov", ".mkv", ".m4v")


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
    value = str(name_or_path).strip()
    if not value:
        value = default_name
    if default_ext:
        value = _ensure_suffix(value, default_ext)

    p = Path(value).expanduser()
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
        raise FileNotFoundError(f"No video file found in: {d}")

    return str(files[0])


def _parse_classes(classes_text: str) -> Optional[List[int]]:
    text = str(classes_text).strip()
    if not text:
        return None

    values: List[int] = []
    for item in text.replace(";", ",").split(","):
        item = item.strip()
        if not item:
            continue
        values.append(int(item))

    return values if values else None


def _plot_result(result, line_width: int):
    try:
        return result.plot(line_width=line_width)
    except TypeError:
        return result.plot()


def _format_names(names) -> str:
    if isinstance(names, dict):
        return ", ".join([f"{k}:{v}" for k, v in list(names.items())[:30]])
    return str(names)


class YoloObbVideoNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_obb_video_node")

        self.declare_parameter("model_dir", DEFAULT_MODEL_DIR)
        self.declare_parameter("model_name", DEFAULT_MODEL_NAME)
        self.declare_parameter("model_path", "")
        self.declare_parameter("raw_video_dir", DEFAULT_VIDEO_DIR)
        self.declare_parameter("video_name", DEFAULT_VIDEO_NAME)
        self.declare_parameter("video_path", "")
        self.declare_parameter("output_dir", DEFAULT_OUTPUT_DIR)
        self.declare_parameter("imgsz", 1024)
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("iou", 0.7)
        self.declare_parameter("max_det", 1000)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("classes", "")
        self.declare_parameter("vid_stride", 1)
        self.declare_parameter("show", True)
        self.declare_parameter("save", True)
        self.declare_parameter("output_fps", 0.0)
        self.declare_parameter("window_width", 1280)
        self.declare_parameter("window_height", 720)
        self.declare_parameter("line_width", 2)
        self.declare_parameter("print_classes", True)

    def _get_param(self, name: str):
        return self.get_parameter(name).value

    def resolve_model_path(self) -> str:
        model_dir = str(self._get_param("model_dir")).strip()
        model_path = str(self._get_param("model_path")).strip()
        model_name = str(self._get_param("model_name")).strip()

        if model_path:
            return _resolve_file(model_dir, model_path, DEFAULT_MODEL_NAME, default_ext=".pt")
        return _resolve_file(model_dir, model_name, DEFAULT_MODEL_NAME, default_ext=".pt")

    def resolve_video_path(self) -> str:
        raw_video_dir = str(self._get_param("raw_video_dir")).strip()
        video_path = str(self._get_param("video_path")).strip()
        video_name = str(self._get_param("video_name")).strip()

        if video_path:
            return _resolve_file(raw_video_dir, video_path, DEFAULT_VIDEO_NAME)
        if video_name:
            return _resolve_file(raw_video_dir, video_name, DEFAULT_VIDEO_NAME)
        return _find_first_file(raw_video_dir, VIDEO_EXTS)

    def run_once(self) -> None:
        model_path = self.resolve_model_path()
        video_path = self.resolve_video_path()
        output_dir = str(self._get_param("output_dir")).strip()
        imgsz = int(self._get_param("imgsz"))
        conf = float(self._get_param("conf"))
        iou = float(self._get_param("iou"))
        max_det = int(self._get_param("max_det"))
        device = str(self._get_param("device")).strip()
        classes = _parse_classes(str(self._get_param("classes")))
        vid_stride = max(1, int(self._get_param("vid_stride")))
        show = _as_bool(self._get_param("show"))
        save = _as_bool(self._get_param("save"))
        output_fps = float(self._get_param("output_fps"))
        window_width = int(self._get_param("window_width"))
        window_height = int(self._get_param("window_height"))
        line_width = int(self._get_param("line_width"))
        print_classes = _as_bool(self._get_param("print_classes"))

        video_file = Path(video_path).expanduser()
        model_file = Path(model_path).expanduser()
        out_dir = Path(output_dir).expanduser()
        out_dir.mkdir(parents=True, exist_ok=True)

        if not model_file.exists():
            raise FileNotFoundError(f"Model file does not exist: {model_file}")
        if not video_file.exists():
            raise FileNotFoundError(f"Video file does not exist: {video_file}")

        model_name = _model_stem(str(model_file))
        output_path = out_dir / f"{video_file.stem}_obb_{model_name}_{imgsz}.mp4"

        self.get_logger().info(f"Loading OBB model: {model_file}")
        model = YOLO(str(model_file))
        if print_classes:
            self.get_logger().info(f"Model classes: {_format_names(model.names)}")

        cap = cv2.VideoCapture(str(video_file))
        if not cap.isOpened():
            raise RuntimeError(f"Failed to open video: {video_file}")

        frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        src_fps = float(cap.get(cv2.CAP_PROP_FPS))
        fps = output_fps if output_fps > 0 else (src_fps if src_fps > 0 else 20.0)

        writer = None
        if save:
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            writer = cv2.VideoWriter(str(output_path), fourcc, fps, (frame_width, frame_height))
            if not writer.isOpened():
                cap.release()
                raise RuntimeError(f"Failed to open video writer: {output_path}")

        self.get_logger().info(
            f"Running OBB video inference: video={video_file}, output={output_path}, "
            f"imgsz={imgsz}, conf={conf}, iou={iou}, max_det={max_det}, device={device}, "
            f"classes={classes}, vid_stride={vid_stride}, save={save}, show={show}"
        )

        frame_index = 0
        last_annotated = None
        total_obb = 0

        try:
            while cap.isOpened():
                success, frame = cap.read()
                if not success:
                    break

                run_infer = (frame_index % vid_stride == 0) or last_annotated is None
                if run_infer:
                    results = model.predict(
                        source=frame,
                        imgsz=imgsz,
                        conf=conf,
                        iou=iou,
                        max_det=max_det,
                        device=device,
                        classes=classes,
                        verbose=False,
                    )
                    if results:
                        result = results[0]
                        last_annotated = _plot_result(result, line_width=line_width)
                        total_obb = 0 if result.obb is None else len(result.obb)
                    else:
                        last_annotated = frame
                        total_obb = 0

                annotated = last_annotated if last_annotated is not None else frame

                if writer is not None:
                    writer.write(annotated)

                if show:
                    display = annotated
                    if window_width > 0 and window_height > 0:
                        display = cv2.resize(annotated, (window_width, window_height))
                    cv2.imshow("YOLO OBB Video", display)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        self.get_logger().info("q pressed, stopping video inference.")
                        break

                if frame_index % 30 == 0:
                    self.get_logger().info(f"frame={frame_index}, obb={total_obb}")

                frame_index += 1
        finally:
            cap.release()
            if writer is not None:
                writer.release()
            if show:
                cv2.destroyAllWindows()

        if save:
            self.get_logger().info(f"Saved output video: {output_path}")
        self.get_logger().info("Finished OBB video inference.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloObbVideoNode()
    try:
        node.run_once()
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(str(exc))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
