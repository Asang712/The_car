#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO image classification for local video files.
Environment: Ubuntu 22 + ROS2 Humble + CPU + Ultralytics .pt model.

Classification is a whole-frame task, so this node does not use tracking.
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
DEFAULT_VIDEO_DIR = f"{DEFAULT_BASE}/MODELS/yolo/videos"
DEFAULT_VIDEO_NAME = "cup.mp4"
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


def _resolve_file(base_dir: str, name_or_path: str, default_name: str, default_ext: str | None = None) -> str:
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


def _topk_lines(result, topk: int) -> List[Tuple[str, float]]:
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


def _draw_classification_overlay(frame, lines: List[Tuple[str, float]], title: str | None = None):
    annotated = frame.copy()
    display_lines = []
    if title:
        display_lines.append(title)
    if lines:
        display_lines.extend([f"{name}: {conf:.2f}" for name, conf in lines])
    else:
        display_lines.append("No classification result")

    x, y0 = 12, 30
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.75
    thickness = 2
    line_gap = 30

    max_width = 0
    for text in display_lines:
        (tw, _), _ = cv2.getTextSize(text, font, font_scale, thickness)
        max_width = max(max_width, tw)
    box_h = line_gap * len(display_lines) + 12
    cv2.rectangle(annotated, (6, 6), (max_width + 28, box_h), (0, 0, 0), -1)

    for i, text in enumerate(display_lines):
        y = y0 + i * line_gap
        cv2.putText(annotated, text, (x, y), font, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)
    return annotated


class YoloClsVideoNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_cls_video_node")

        self.declare_parameter("model_dir", DEFAULT_MODEL_DIR)
        self.declare_parameter("model_name", DEFAULT_MODEL_NAME)
        self.declare_parameter("model_path", "")
        self.declare_parameter("raw_video_dir", DEFAULT_VIDEO_DIR)
        self.declare_parameter("video_name", DEFAULT_VIDEO_NAME)
        self.declare_parameter("video_path", "")
        self.declare_parameter("output_dir", DEFAULT_OUTPUT_DIR)
        self.declare_parameter("imgsz", 224)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("topk", 5)
        self.declare_parameter("vid_stride", 1)
        self.declare_parameter("show", True)
        self.declare_parameter("save", True)
        self.declare_parameter("output_fps", 0.0)
        self.declare_parameter("window_width", 1280)
        self.declare_parameter("window_height", 720)

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
        device = str(self._get_param("device")).strip()
        topk = int(self._get_param("topk"))
        vid_stride = max(1, int(self._get_param("vid_stride")))
        show = _as_bool(self._get_param("show"))
        save = _as_bool(self._get_param("save"))
        output_fps_param = float(self._get_param("output_fps"))
        window_width = int(self._get_param("window_width"))
        window_height = int(self._get_param("window_height"))

        video_file = Path(video_path).expanduser()
        model_file = Path(model_path).expanduser()
        out_dir = Path(output_dir).expanduser()
        out_dir.mkdir(parents=True, exist_ok=True)

        if not model_file.exists():
            raise FileNotFoundError(f"Model file does not exist: {model_file}")
        if not video_file.exists():
            raise FileNotFoundError(f"Video file does not exist: {video_file}")

        model_name = _model_stem(str(model_file))
        output_name = f"{video_file.stem}_cls_{model_name}_{imgsz}_top{topk}.mp4"
        output_path = out_dir / output_name

        self.get_logger().info(f"Loading model: {model_file}")
        model = YOLO(str(model_file))

        cap = cv2.VideoCapture(str(video_file))
        if not cap.isOpened():
            raise RuntimeError(f"Failed to open video: {video_file}")

        frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = float(cap.get(cv2.CAP_PROP_FPS))
        if fps <= 1e-6:
            fps = 15.0
        output_fps = output_fps_param if output_fps_param > 0 else fps

        writer = None
        if save:
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            writer = cv2.VideoWriter(str(output_path), fourcc, output_fps, (frame_width, frame_height))
            if not writer.isOpened():
                cap.release()
                raise RuntimeError(f"Failed to open video writer: {output_path}")

        self.get_logger().info(
            f"Running video classification: video={video_file}, output={output_path}, "
            f"imgsz={imgsz}, topk={topk}, device={device}, vid_stride={vid_stride}, "
            f"save={save}, show={show}"
        )

        frame_index = 0
        last_lines: List[Tuple[str, float]] = []
        try:
            while cap.isOpened():
                ok, frame = cap.read()
                if not ok:
                    break

                frame_index += 1
                if frame_index == 1 or frame_index % vid_stride == 0:
                    results = model.predict(source=frame, imgsz=imgsz, device=device, verbose=False)
                    if results:
                        last_lines = _topk_lines(results[0], topk)

                title = f"{model_name}  top{topk}  frame:{frame_index}"
                annotated = _draw_classification_overlay(frame, last_lines, title)

                if writer is not None:
                    writer.write(annotated)

                if show:
                    display = annotated
                    if window_width > 0 and window_height > 0:
                        display = cv2.resize(annotated, (window_width, window_height))
                    cv2.imshow("YOLO Classification Video", display)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        self.get_logger().info("q pressed, stopping video classification.")
                        break
        finally:
            cap.release()
            if writer is not None:
                writer.release()
            if show:
                cv2.destroyAllWindows()

        if save:
            self.get_logger().info(f"Saved output video: {output_path}")
        self.get_logger().info("Finished video classification.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloClsVideoNode()
    try:
        node.run_once()
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(str(exc))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
