#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO image classification for ROS2 Image topic.
Environment: Ubuntu 22 + ROS2 Humble + CPU + Ultralytics .pt model.

特点：
1. 订阅 ROS2 图像话题，实时分类整帧图像。
2. 发布带分类结果的小字号可视化图像。
3. 可选保存实时分类视频。
4. 使用自定义小字体叠加，不再使用遮挡严重的大字号 result.plot()。
5. 支持只指定 model_name，例如 yolo11n-cls 或 yolo11n-cls.pt。
"""

from __future__ import annotations

import os
import time
from datetime import datetime
from pathlib import Path
from typing import List, Tuple

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
DEFAULT_MODEL_NAME = "yolo11n-cls.pt"
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


class YoloClsRealtimeNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_cls_realtime_node")

        self.declare_parameter("model_dir", DEFAULT_MODEL_DIR)
        self.declare_parameter("model_name", DEFAULT_MODEL_NAME)
        self.declare_parameter("model_path", "")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("annotated_topic", "/yolo_cls/annotated_image")
        self.declare_parameter("status_topic", "/yolo_cls/status")
        self.declare_parameter("output_dir", DEFAULT_OUTPUT_DIR)

        self.declare_parameter("imgsz", 224)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("topk", 5)
        self.declare_parameter("display_topk", 3)

        self.declare_parameter("show", True)
        self.declare_parameter("save_video", True)
        self.declare_parameter("output_fps", 20.0)

        self.declare_parameter("window_width", 1280)
        self.declare_parameter("window_height", 720)

        self.declare_parameter("qos_reliability", "best_effort")
        self.declare_parameter("qos_depth", 1)
        self.declare_parameter("process_every_n_frames", 1)
        self.declare_parameter("publish_status", True)

        # 小字号显示相关参数
        self.declare_parameter("text_scale", 0.45)
        self.declare_parameter("text_thickness", 1)
        self.declare_parameter("text_x", 10)
        self.declare_parameter("text_y", 22)
        self.declare_parameter("text_line_gap", 18)
        self.declare_parameter("show_fps_text", True)
        self.declare_parameter("show_model_text", True)
        self.declare_parameter("draw_text_shadow", True)

        self.model_dir = str(self.get_parameter("model_dir").value).strip()
        self.model_name_param = str(self.get_parameter("model_name").value).strip()
        self.model_path_param = str(self.get_parameter("model_path").value).strip()
        self.model_path = self.resolve_model_path()

        self.image_topic = str(self.get_parameter("image_topic").value).strip()
        self.annotated_topic = str(self.get_parameter("annotated_topic").value).strip()
        self.status_topic = str(self.get_parameter("status_topic").value).strip()
        self.output_dir = str(self.get_parameter("output_dir").value).strip()

        self.imgsz = int(self.get_parameter("imgsz").value)
        self.device = str(self.get_parameter("device").value).strip()
        self.topk = max(1, int(self.get_parameter("topk").value))
        self.display_topk = max(1, int(self.get_parameter("display_topk").value))

        self.show = _as_bool(self.get_parameter("show").value)
        self.save_video = _as_bool(self.get_parameter("save_video").value)
        self.output_fps = float(self.get_parameter("output_fps").value)

        self.window_width = int(self.get_parameter("window_width").value)
        self.window_height = int(self.get_parameter("window_height").value)

        self.qos_reliability = str(self.get_parameter("qos_reliability").value).strip()
        self.qos_depth = int(self.get_parameter("qos_depth").value)
        self.process_every_n_frames = max(
            1,
            int(self.get_parameter("process_every_n_frames").value),
        )
        self.publish_status = _as_bool(self.get_parameter("publish_status").value)

        self.text_scale = float(self.get_parameter("text_scale").value)
        self.text_thickness = int(self.get_parameter("text_thickness").value)
        self.text_x = int(self.get_parameter("text_x").value)
        self.text_y = int(self.get_parameter("text_y").value)
        self.text_line_gap = int(self.get_parameter("text_line_gap").value)
        self.show_fps_text = _as_bool(self.get_parameter("show_fps_text").value)
        self.show_model_text = _as_bool(self.get_parameter("show_model_text").value)
        self.draw_text_shadow = _as_bool(self.get_parameter("draw_text_shadow").value)

        model_file = Path(self.model_path).expanduser()
        if not model_file.exists():
            raise FileNotFoundError(f"Model file does not exist: {model_file}")

        Path(self.output_dir).expanduser().mkdir(parents=True, exist_ok=True)

        self.get_logger().info(f"Loading classification model: {model_file}")
        self.model = YOLO(str(model_file))
        self.model_name = _model_stem(str(model_file))
        self.processing = False
        self.frame_index = 0
        self.last_time = time.monotonic()
        self.last_fps = 0.0

        self.last_labels: List[Tuple[str, float]] = []
        self.video_writer = None
        self.video_path = None

        self.annotated_pub = self.create_publisher(Image, self.annotated_topic, 10)
        self.status_pub = (
            self.create_publisher(String, self.status_topic, 10)
            if self.publish_status
            else None
        )

        qos = _make_qos(self.qos_reliability, self.qos_depth)
        self.sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos,
        )

        self.get_logger().info(
            f"Realtime classification ready. input={self.image_topic}, "
            f"output={self.annotated_topic}, model={self.model_name}, "
            f"imgsz={self.imgsz}, topk={self.topk}, display_topk={self.display_topk}, "
            f"device={self.device}, text_scale={self.text_scale}"
        )

    def resolve_model_path(self) -> str:
        if self.model_path_param:
            return _resolve_file(
                self.model_dir,
                self.model_path_param,
                DEFAULT_MODEL_NAME,
                default_ext=".pt",
            )

        return _resolve_file(
            self.model_dir,
            self.model_name_param,
            DEFAULT_MODEL_NAME,
            default_ext=".pt",
        )

    def _update_fps(self) -> float:
        now = time.monotonic()
        dt = now - self.last_time
        self.last_time = now

        if dt > 1e-6:
            self.last_fps = 1.0 / dt

        return self.last_fps

    def _extract_topk(self, result) -> List[Tuple[str, float]]:
        if result.probs is None:
            return []

        names = result.names
        probs = result.probs

        try:
            top_indices = probs.top5[: self.topk]
        except Exception:
            top_indices = []

        labels: List[Tuple[str, float]] = []

        for idx in top_indices:
            try:
                score = float(probs.data[idx])
                label = str(names[int(idx)])
                labels.append((label, score))
            except Exception:
                continue

        return labels

    def _draw_small_text(self, frame, labels: List[Tuple[str, float]], fps: float):
        annotated = frame.copy()

        lines = []

        if self.show_model_text:
            lines.append(f"{self.model_name}")

        if self.show_fps_text:
            lines.append(f"FPS:{fps:.1f}")

        for label, score in labels[: self.display_topk]:
            lines.append(f"{label}: {score:.2f}")

        x = self.text_x
        y = self.text_y

        for i, text in enumerate(lines):
            yy = y + i * self.text_line_gap

            if self.draw_text_shadow:
                cv2.putText(
                    annotated,
                    text,
                    (x + 1, yy + 1),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    self.text_scale,
                    (0, 0, 0),
                    max(1, self.text_thickness + 1),
                    cv2.LINE_AA,
                )

            cv2.putText(
                annotated,
                text,
                (x, yy),
                cv2.FONT_HERSHEY_SIMPLEX,
                self.text_scale,
                (0, 255, 0),
                self.text_thickness,
                cv2.LINE_AA,
            )

        return annotated

    def _init_video_writer(self, frame) -> None:
        if self.video_writer is not None or not self.save_video:
            return

        h, w = frame.shape[:2]
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"realtime_cls_{self.model_name}_{self.imgsz}_{timestamp}.mp4"
        self.video_path = str(Path(self.output_dir).expanduser() / filename)

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self.video_writer = cv2.VideoWriter(
            self.video_path,
            fourcc,
            self.output_fps,
            (w, h),
        )

        if not self.video_writer.isOpened():
            self.get_logger().error(f"Failed to open video writer: {self.video_path}")
            self.video_writer = None
        else:
            self.get_logger().info(f"Saving realtime output video: {self.video_path}")

    def image_callback(self, msg: Image) -> None:
        self.frame_index += 1

        if self.frame_index % self.process_every_n_frames != 0:
            return

        # CPU 推理下防止回调堆积，保持低延迟。
        if self.processing:
            return

        self.processing = True

        try:
            frame = image_msg_to_bgr(msg)

            results = self.model.predict(
                source=frame,
                imgsz=self.imgsz,
                device=self.device,
                verbose=False,
            )

            fps = self._update_fps()

            if results:
                self.last_labels = self._extract_topk(results[0])

            annotated = self._draw_small_text(frame, self.last_labels, fps)

            out_msg = bgr_to_image_msg(annotated, msg.header)
            self.annotated_pub.publish(out_msg)

            if self.status_pub is not None:
                status = String()
                label_text = ", ".join(
                    [f"{label}:{score:.3f}" for label, score in self.last_labels[: self.display_topk]]
                )
                status.data = (
                    f"model={self.model_name}, fps={fps:.2f}, "
                    f"imgsz={self.imgsz}, device={self.device}, top={label_text}"
                )
                self.status_pub.publish(status)

            self._init_video_writer(annotated)
            if self.video_writer is not None:
                self.video_writer.write(annotated)

            if self.show:
                display = annotated
                if self.window_width > 0 and self.window_height > 0:
                    display = cv2.resize(annotated, (self.window_width, self.window_height))

                cv2.imshow("YOLO Classification Realtime", display)
                key = cv2.waitKey(1) & 0xFF

                if key in (ord("q"), ord("Q"), 27):
                    self.get_logger().info("q pressed, shutting down.")
                    os._exit(0)  # pylint: disable=protected-access

        except Exception as exc:
            self.get_logger().error(f"Realtime classification callback error: {exc}")
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
        node = YoloClsRealtimeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
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
