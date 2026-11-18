#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO pose estimation + optional tracking for ROS2 Image topic.
Environment: Ubuntu 22 + ROS2 Humble + CPU + Ultralytics .pt model.

设计目标：
- 订阅 /camera/image_raw，发布 /yolo_pose/annotated_image。
- 通过 model_name 切换模型，例如 yolo11n-pose.pt / yolo11s-pose。
- CPU 推理时使用丢帧保护，避免图像回调堆积导致延迟越来越大。
"""

from __future__ import annotations

import os
import time
from datetime import datetime
from pathlib import Path
from typing import Optional, List

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from .image_message_utils import bgr_to_image_msg, image_msg_to_bgr
from std_msgs.msg import String
from ultralytics import YOLO


DEFAULT_BASE = "/home/yahboom"
DEFAULT_MODEL_DIR = f"{DEFAULT_BASE}/MODELS/yolo"
DEFAULT_MODEL_NAME = "yolo11n-pose.pt"
DEFAULT_OUTPUT_DIR = f"{DEFAULT_BASE}/MODELS/yolo/videos/output"


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


def _make_qos(reliability: str, depth: int) -> QoSProfile:
    reliability = reliability.strip().lower()
    if reliability in ("reliable", "r"):
        rel = ReliabilityPolicy.RELIABLE
    else:
        rel = ReliabilityPolicy.BEST_EFFORT
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=max(1, int(depth)),
        reliability=rel,
    )


class YoloPoseRealtimeNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_pose_realtime_node")

        self.declare_parameter("model_dir", DEFAULT_MODEL_DIR)
        self.declare_parameter("model_name", DEFAULT_MODEL_NAME)
        self.declare_parameter("model_path", "")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("annotated_topic", "/yolo_pose/annotated_image")
        self.declare_parameter("status_topic", "/yolo_pose/status")
        self.declare_parameter("output_dir", DEFAULT_OUTPUT_DIR)
        self.declare_parameter("imgsz", 960)
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("iou", 0.7)
        self.declare_parameter("max_det", 300)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("classes", "")
        self.declare_parameter("tracker", "bytetrack.yaml")
        self.declare_parameter("show", True)
        self.declare_parameter("save_video", True)
        self.declare_parameter("output_fps", 20.0)
        self.declare_parameter("window_width", 1280)
        self.declare_parameter("window_height", 720)
        self.declare_parameter("qos_reliability", "best_effort")
        self.declare_parameter("qos_depth", 1)
        self.declare_parameter("process_every_n_frames", 1)
        self.declare_parameter("line_width", 2)
        self.declare_parameter("publish_status", True)
        self.declare_parameter("use_track", False)

        self.model_dir = str(self.get_parameter("model_dir").value).strip()
        self.model_name_param = str(self.get_parameter("model_name").value).strip()
        self.model_path_param = str(self.get_parameter("model_path").value).strip()
        self.model_path = self.resolve_model_path()

        self.image_topic = str(self.get_parameter("image_topic").value).strip()
        self.annotated_topic = str(self.get_parameter("annotated_topic").value).strip()
        self.status_topic = str(self.get_parameter("status_topic").value).strip()
        self.output_dir = str(self.get_parameter("output_dir").value).strip()
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.conf = float(self.get_parameter("conf").value)
        self.iou = float(self.get_parameter("iou").value)
        self.max_det = int(self.get_parameter("max_det").value)
        self.device = str(self.get_parameter("device").value).strip()
        self.classes = _parse_classes(str(self.get_parameter("classes").value))
        self.tracker = str(self.get_parameter("tracker").value).strip()
        self.show = _as_bool(self.get_parameter("show").value)
        self.save_video = _as_bool(self.get_parameter("save_video").value)
        self.output_fps = float(self.get_parameter("output_fps").value)
        self.window_width = int(self.get_parameter("window_width").value)
        self.window_height = int(self.get_parameter("window_height").value)
        self.qos_reliability = str(self.get_parameter("qos_reliability").value).strip()
        self.qos_depth = int(self.get_parameter("qos_depth").value)
        self.process_every_n_frames = max(1, int(self.get_parameter("process_every_n_frames").value))
        self.line_width = int(self.get_parameter("line_width").value)
        self.publish_status = _as_bool(self.get_parameter("publish_status").value)
        self.use_track = _as_bool(self.get_parameter("use_track").value)

        model_file = Path(self.model_path).expanduser()
        if not model_file.exists():
            raise FileNotFoundError(f"Model file does not exist: {model_file}")
        Path(self.output_dir).expanduser().mkdir(parents=True, exist_ok=True)

        self.get_logger().info(f"Loading model: {model_file}")
        self.model = YOLO(str(model_file))
        self.model_name = _model_stem(str(model_file))
        self.processing = False
        self.frame_index = 0
        self.last_time = time.monotonic()
        self.last_fps = 0.0
        self.video_writer = None
        self.video_path = None

        self.annotated_pub = self.create_publisher(Image, self.annotated_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10) if self.publish_status else None

        qos = _make_qos(self.qos_reliability, self.qos_depth)
        self.sub = self.create_subscription(Image, self.image_topic, self.image_callback, qos)

        self.get_logger().info(
            f"Realtime pose estimation ready. input={self.image_topic}, output={self.annotated_topic}, "
            f"model={self.model_name}, imgsz={self.imgsz}, conf={self.conf}, iou={self.iou}, "
            f"max_det={self.max_det}, device={self.device}, tracker={self.tracker}, "
            f"use_track={self.use_track}, save_video={self.save_video}, classes={self.classes}"
        )

    def resolve_model_path(self) -> str:
        if self.model_path_param:
            return _resolve_file(self.model_dir, self.model_path_param, DEFAULT_MODEL_NAME, default_ext=".pt")
        return _resolve_file(self.model_dir, self.model_name_param, DEFAULT_MODEL_NAME, default_ext=".pt")

    def _init_video_writer(self, frame) -> None:
        if self.video_writer is not None or not self.save_video:
            return
        h, w = frame.shape[:2]
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        mode_name = "track" if self.use_track else "detect"
        filename = f"realtime_pose_{mode_name}_{self.model_name}_{self.imgsz}_{timestamp}.mp4"
        self.video_path = str(Path(self.output_dir).expanduser() / filename)
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self.video_writer = cv2.VideoWriter(self.video_path, fourcc, self.output_fps, (w, h))
        if not self.video_writer.isOpened():
            self.get_logger().error(f"Failed to open video writer: {self.video_path}")
            self.video_writer = None
        else:
            self.get_logger().info(f"Saving realtime output video: {self.video_path}")

    def _update_fps(self) -> float:
        now = time.monotonic()
        dt = now - self.last_time
        self.last_time = now
        if dt > 1e-6:
            self.last_fps = 1.0 / dt
        return self.last_fps

    def image_callback(self, msg: Image) -> None:
        self.frame_index += 1
        if self.frame_index % self.process_every_n_frames != 0:
            return

        # CPU 推理时避免回调堆积。上一帧还在处理，新帧直接丢弃，保证低延迟。
        if self.processing:
            return

        self.processing = True
        try:
            frame = image_msg_to_bgr(msg)

            common_kwargs = dict(
                source=frame,
                imgsz=self.imgsz,
                conf=self.conf,
                iou=self.iou,
                max_det=self.max_det,
                device=self.device,
                classes=self.classes,
                line_width=self.line_width,
                verbose=False,
            )

            if self.use_track:
                results = self.model.track(
                    persist=True,
                    tracker=self.tracker,
                    **common_kwargs,
                )
            else:
                results = self.model.predict(**common_kwargs)

            if not results:
                return

            result = results[0]
            annotated = result.plot(line_width=self.line_width)
            fps = self._update_fps()
            num_boxes = 0 if result.boxes is None else len(result.boxes)
            num_keypoints = 0 if result.keypoints is None else len(result.keypoints)

            cv2.putText(
                annotated,
                f"{self.model_name}  FPS:{fps:.1f}  persons:{num_boxes} keypoints:{num_keypoints}",
                (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

            out_msg = bgr_to_image_msg(annotated, msg.header)
            self.annotated_pub.publish(out_msg)

            if self.status_pub is not None:
                status = String()
                status.data = (
                    f"model={self.model_name}, fps={fps:.2f}, persons={num_boxes}, "
                    f"keypoint_sets={num_keypoints}, imgsz={self.imgsz}, conf={self.conf}, "
                    f"iou={self.iou}, device={self.device}, use_track={self.use_track}"
                )
                self.status_pub.publish(status)

            self._init_video_writer(annotated)
            if self.video_writer is not None:
                self.video_writer.write(annotated)

            if self.show:
                display = annotated
                if self.window_width > 0 and self.window_height > 0:
                    display = cv2.resize(annotated, (self.window_width, self.window_height))
                cv2.imshow("YOLO Pose Realtime", display)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q"), 27):
                    self.get_logger().info("q pressed, shutting down.")
                    os._exit(0)  # pylint: disable=protected-access

        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Realtime callback error: {exc}")
        finally:
            self.processing = False

    def destroy_node(self) -> bool:
        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info(f"Realtime output video saved: {self.video_path}")
        if self.show:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = YoloPoseRealtimeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(exc)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
