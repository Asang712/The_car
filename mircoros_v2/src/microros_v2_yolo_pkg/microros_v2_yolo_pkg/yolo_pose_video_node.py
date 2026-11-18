#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO pose estimation + optional tracking for local video files.
Environment: Ubuntu 22 + ROS2 Humble + CPU + Ultralytics .pt model.

设计目标：
- 通过 model_name 切换模型，例如 yolo11n-pose.pt / yolo11s-pose / yolo11m-pose。
- 通过 video_name 切换测试视频，例如 people.mp4。
- 默认使用 model.predict(...)；设置 use_track 后可输出 track id。
"""

from __future__ import annotations

from pathlib import Path
from typing import Iterable, Optional, List

import rclpy
from rclpy.node import Node
from ultralytics import YOLO


DEFAULT_BASE = "/home/yahboom"
DEFAULT_MODEL_DIR = f"{DEFAULT_BASE}/MODELS/yolo"
DEFAULT_MODEL_NAME = "yolo11n-pose.pt"
DEFAULT_VIDEO_DIR = f"{DEFAULT_BASE}/MODELS/yolo/videos"
DEFAULT_VIDEO_NAME = "people.mp4"
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
    value = str(path_or_name).strip()
    if not value:
        return value
    p = Path(value)
    if p.suffix:
        return value
    return f"{value}{suffix}"


def _resolve_file(
    base_dir: str,
    name_or_path: str,
    default_name: str,
    default_ext: Optional[str] = None,
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


def _parse_classes(value: str) -> Optional[List[int]]:
    text = str(value).strip()
    if not text:
        return None
    classes: List[int] = []
    for item in text.split(","):
        item = item.strip()
        if item:
            classes.append(int(item))
    return classes if classes else None


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


class YoloPoseVideoNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_pose_video_node")

        self.declare_parameter("model_dir", DEFAULT_MODEL_DIR)
        self.declare_parameter("model_name", DEFAULT_MODEL_NAME)
        self.declare_parameter("model_path", "")
        self.declare_parameter("raw_video_dir", DEFAULT_VIDEO_DIR)
        self.declare_parameter("video_name", DEFAULT_VIDEO_NAME)
        self.declare_parameter("video_path", "")
        self.declare_parameter("output_dir", DEFAULT_OUTPUT_DIR)
        self.declare_parameter("imgsz", 960)
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("iou", 0.7)
        self.declare_parameter("max_det", 300)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("classes", "")
        self.declare_parameter("vid_stride", 1)
        self.declare_parameter("tracker", "bytetrack.yaml")
        self.declare_parameter("show", True)
        self.declare_parameter("save", True)
        self.declare_parameter("exist_ok", True)
        self.declare_parameter("line_width", 2)
        self.declare_parameter("use_track", False)

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
        vid_stride = int(self._get_param("vid_stride"))
        tracker = str(self._get_param("tracker")).strip()
        show = _as_bool(self._get_param("show"))
        save = _as_bool(self._get_param("save"))
        exist_ok = _as_bool(self._get_param("exist_ok"))
        line_width = int(self._get_param("line_width"))
        use_track = _as_bool(self._get_param("use_track"))

        video_file = Path(video_path).expanduser()
        model_file = Path(model_path).expanduser()
        out_dir = Path(output_dir).expanduser()
        out_dir.mkdir(parents=True, exist_ok=True)

        if not model_file.exists():
            raise FileNotFoundError(f"Model file does not exist: {model_file}")
        if not video_file.exists():
            raise FileNotFoundError(f"Video file does not exist: {video_file}")

        model_name = _model_stem(str(model_file))
        task_name = "pose_track" if use_track else "pose_predict"
        run_name = f"{video_file.stem}_{task_name}_{model_name}_{imgsz}"

        self.get_logger().info(f"Loading model: {model_file}")
        model = YOLO(str(model_file))

        self.get_logger().info(
            f"Running video pose estimation: video={video_file}, output={out_dir / run_name}, "
            f"imgsz={imgsz}, conf={conf}, iou={iou}, max_det={max_det}, device={device}, "
            f"vid_stride={vid_stride}, tracker={tracker}, use_track={use_track}, classes={classes}"
        )

        common_kwargs = dict(
            source=str(video_file),
            project=str(out_dir),
            name=run_name,
            exist_ok=exist_ok,
            imgsz=imgsz,
            conf=conf,
            iou=iou,
            max_det=max_det,
            device=device,
            classes=classes,
            vid_stride=vid_stride,
            save=save,
            show=show,
            line_width=line_width,
            verbose=False,
        )

        if use_track:
            model.track(tracker=tracker, **common_kwargs)
        else:
            model.predict(**common_kwargs)

        self.get_logger().info(f"Finished. Ultralytics output directory: {out_dir / run_name}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloPoseVideoNode()
    try:
        node.run_once()
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(str(exc))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
