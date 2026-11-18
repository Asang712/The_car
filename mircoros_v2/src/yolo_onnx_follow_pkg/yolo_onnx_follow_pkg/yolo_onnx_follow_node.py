#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Standalone YOLOv11 ONNX visual-follow node.

The node subscribes to ``/camera/image_raw``, runs YOLO ONNX inference locally,
shows detections in an OpenCV window, and starts visual following after the user
clicks a target box. ``CAR_TYPE=ptz`` uses PTZ + chassis yaw, while
``CAR_TYPE=no_ptz`` uses chassis yaw only for a fixed camera. It is independent
from the LLM/Dify action layer.
"""

import math
import os
import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image

from yolo_onnx_follow_pkg.ptz_controller import PtzController, clamp


COCO_CLASS_NAMES = [
    "person",
    "bicycle",
    "car",
    "motorcycle",
    "airplane",
    "bus",
    "train",
    "truck",
    "boat",
    "traffic light",
    "fire hydrant",
    "stop sign",
    "parking meter",
    "bench",
    "bird",
    "cat",
    "dog",
    "horse",
    "sheep",
    "cow",
    "elephant",
    "bear",
    "zebra",
    "giraffe",
    "backpack",
    "umbrella",
    "handbag",
    "tie",
    "suitcase",
    "frisbee",
    "skis",
    "snowboard",
    "sports ball",
    "kite",
    "baseball bat",
    "baseball glove",
    "skateboard",
    "surfboard",
    "tennis racket",
    "bottle",
    "wine glass",
    "cup",
    "fork",
    "knife",
    "spoon",
    "bowl",
    "banana",
    "apple",
    "sandwich",
    "orange",
    "broccoli",
    "carrot",
    "hot dog",
    "pizza",
    "donut",
    "cake",
    "chair",
    "couch",
    "potted plant",
    "bed",
    "dining table",
    "toilet",
    "tv",
    "laptop",
    "mouse",
    "remote",
    "keyboard",
    "cell phone",
    "microwave",
    "oven",
    "toaster",
    "sink",
    "refrigerator",
    "book",
    "clock",
    "vase",
    "scissors",
    "teddy bear",
    "hair drier",
    "toothbrush",
]


@dataclass
class Detection:
    bbox: tuple[float, float, float, float]
    score: float
    class_id: int
    class_name: str
    track_id: int = -1

    def __post_init__(self) -> None:
        self.bbox = tuple(float(value) for value in self.bbox)
        self.score = float(self.score)
        self.class_id = int(self.class_id)
        self.class_name = str(self.class_name)
        self.track_id = int(self.track_id)

    @property
    def cx(self) -> float:
        return (self.bbox[0] + self.bbox[2]) * 0.5

    @property
    def cy(self) -> float:
        return (self.bbox[1] + self.bbox[3]) * 0.5

    @property
    def width(self) -> float:
        return max(0.0, self.bbox[2] - self.bbox[0])

    @property
    def height(self) -> float:
        return max(0.0, self.bbox[3] - self.bbox[1])


class SimpleIouTracker:
    """Small local tracker that assigns stable ids by IoU and center matching."""

    def __init__(
        self,
        iou_threshold: float = 0.3,
        max_lost_frames: int = 10,
        center_threshold_ratio: float = 0.12,
    ):
        self.iou_threshold = float(iou_threshold)
        self.max_lost_frames = int(max_lost_frames)
        self.center_threshold_ratio = float(center_threshold_ratio)
        self.next_id: int = 1
        self.tracks: dict[int, dict] = {}

    def update(
        self,
        detections: list[Detection],
        image_width: int | None = None,
        image_height: int | None = None,
    ) -> list[Detection]:
        unmatched_track_ids = set(self.tracks.keys())
        frame_diagonal = 0.0
        if image_width is not None and image_height is not None:
            frame_diagonal = math.hypot(
                max(1.0, float(image_width)), max(1.0, float(image_height))
            )
        max_center_dist = frame_diagonal * max(0.0, self.center_threshold_ratio)
        for detection in sorted(detections, key=lambda item: item.score, reverse=True):
            best_track_id = None
            best_iou = 0.0
            nearest_track_id = None
            nearest_center_dist = float("inf")
            for track_id in list(unmatched_track_ids):
                track = self.tracks[track_id]
                if track["class_id"] != detection.class_id:
                    continue
                iou = self._iou(track["bbox"], detection.bbox)
                if iou > best_iou:
                    best_iou = iou
                    best_track_id = track_id
                center_dist = self._center_distance(track["bbox"], detection.bbox)
                if center_dist < nearest_center_dist:
                    nearest_center_dist = center_dist
                    nearest_track_id = track_id

            if best_track_id is not None and best_iou >= self.iou_threshold:
                detection.track_id = int(best_track_id)
                unmatched_track_ids.discard(best_track_id)
            elif (
                nearest_track_id is not None
                and max_center_dist > 0.0
                and nearest_center_dist <= max_center_dist
            ):
                detection.track_id = int(nearest_track_id)
                unmatched_track_ids.discard(nearest_track_id)
            else:
                detection.track_id = int(self.next_id)
                self.next_id += 1

            self.tracks[int(detection.track_id)] = {
                "bbox": detection.bbox,
                "class_id": int(detection.class_id),
                "lost": 0,
            }

        for track_id in unmatched_track_ids:
            self.tracks[track_id]["lost"] += 1
            if self.tracks[track_id]["lost"] > self.max_lost_frames:
                del self.tracks[track_id]
        return detections

    @staticmethod
    def _iou(a, b) -> float:
        ax1, ay1, ax2, ay2 = a
        bx1, by1, bx2, by2 = b
        inter_x1 = max(ax1, bx1)
        inter_y1 = max(ay1, by1)
        inter_x2 = min(ax2, bx2)
        inter_y2 = min(ay2, by2)
        inter_w = max(0.0, inter_x2 - inter_x1)
        inter_h = max(0.0, inter_y2 - inter_y1)
        inter_area = inter_w * inter_h
        area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
        area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
        union = area_a + area_b - inter_area
        return inter_area / union if union > 0.0 else 0.0

    @staticmethod
    def _center_distance(a, b) -> float:
        ax1, ay1, ax2, ay2 = a
        bx1, by1, bx2, by2 = b
        return math.hypot(
            (ax1 + ax2 - bx1 - bx2) * 0.5,
            (ay1 + ay2 - by1 - by2) * 0.5,
        )


class YoloOnnxFollowNode(Node):
    """Standalone ONNX YOLO detector + local target selection + visual follow."""

    def __init__(self):
        super().__init__("yolo_onnx_follow_node")
        self.config = self._load_config()
        self.demo_config = self.config["YOLO_ONNX_FOLLOW"]
        self.car_type = self._read_car_type()
        self.bridge = CvBridge()

        self.net = None
        self.class_names = self._load_class_names()
        self.tracker = SimpleIouTracker(
            iou_threshold=float(self.demo_config.get("track_iou_threshold", 0.2)),
            max_lost_frames=int(self.demo_config.get("track_max_lost_frames", 30)),
            center_threshold_ratio=float(
                self.demo_config.get("track_center_threshold_ratio", 0.12)
            ),
        )

        self.frame_lock = threading.Lock()
        self.inference_lock = threading.Lock()
        self.latest_frame = None
        self.latest_detections: list[Detection] = []
        self.selected_track_id = self._initial_track_id()
        self.selected_class_name = self._initial_class_name()
        self.selected_class_id = self._initial_class_id()
        self.selected_last_bbox = None
        self.selected_last_seen_time = 0.0
        self.selected_lost_frames = 0
        self.selected_lost_reported = False
        self.last_reacquire_log_time = 0.0
        self.latest_target_state = self._empty_target_state()

        self.start_time = time.monotonic()
        self.image_frame_count = 0
        self.last_image_time = 0.0
        self.last_frame_log_time = 0.0
        self.last_image_watchdog_log_time = 0.0
        self.last_inference_duration = 0.0
        self.last_inference_warning_time = 0.0
        self.display_period_sec = 1.0 / max(
            1.0, float(self.demo_config.get("display_rate_hz", 20.0))
        )
        self.last_display_time = 0.0
        self.prev_err_x = 0.0
        self.prev_err_y = 0.0
        self.prev_control_time = 0.0
        self.last_body_az = 0.0
        self.body_correcting = False
        self.prev_body_error_eff = 0.0
        self.last_motion_log_time = 0.0
        self.last_inference_time = 0.0
        self.shutdown_requested = False
        self._cleaned_up = False
        self.ptz = None

        self._init_ros()
        self._load_model()
        if self.car_type == "ptz":
            self.ptz = PtzController(self, self.config)
        self._init_ui()

        follow_period = 1.0 / max(1.0, float(self.demo_config["follow_rate_hz"]))
        self.follow_timer = self.create_timer(follow_period, self._follow_tick)
        self.image_watchdog_timer = self.create_timer(2.0, self._image_watchdog_tick)
        self.get_logger().info(
            f"[YOLO_ONNX] ready: CAR_TYPE={self.car_type}; "
            "click a box to follow, r=reselect, q=quit"
        )

    # ---- config --------------------------------------------------------

    def _load_config(self) -> dict:
        self.declare_parameter("config_file", "")
        config_file = str(self.get_parameter("config_file").value or "")
        if config_file and os.path.exists(config_file):
            with open(config_file, "r", encoding="utf-8") as config_stream:
                config = yaml.safe_load(config_stream) or {}
        else:
            config = self._default_config()

        demo_config = config.setdefault("YOLO_ONNX_FOLLOW", {})
        self._declare_and_override_param(
            demo_config, "model_path", str(demo_config.get("model_path", ""))
        )
        self._declare_and_override_param(
            demo_config,
            "image_topic",
            str(demo_config.get("image_topic", "/camera/image_raw")),
        )
        self._declare_and_override_param(
            demo_config, "target_class", str(demo_config.get("target_class", ""))
        )
        self._declare_and_override_param(
            demo_config,
            "target_class_id",
            int(demo_config.get("target_class_id", -1)),
        )
        self._declare_and_override_param(
            demo_config, "target_id", int(demo_config.get("target_id", -1))
        )
        return config

    def _declare_and_override_param(self, section: dict, name: str, default):
        self.declare_parameter(name, default)
        value = self.get_parameter(name).value
        if value not in (None, ""):
            section[name] = value

    def _read_car_type(self) -> str:
        car_type = str(
            os.environ.get("CAR_TYPE", "ptz") or "ptz"
        ).strip().lower()
        if car_type in ("ptz", "no_ptz"):
            return car_type
        self.get_logger().warn(
            f"[CONFIG] unsupported CAR_TYPE={car_type!r}; falling back to 'ptz'"
        )
        return "ptz"

    def _diagnostic_log_enabled(self) -> bool:
        return bool(self.demo_config.get("diagnostic_log_enable", False))

    def _log_diagnostic(self, text: str) -> None:
        if self._diagnostic_log_enabled():
            self.get_logger().info(str(text or ""))

    def _log_diagnostic_warn(self, text: str) -> None:
        if self._diagnostic_log_enabled():
            self.get_logger().warn(str(text or ""))

    def _log_target_event(
        self,
        event: str,
        track_id: int = -1,
        class_id: int = -1,
        class_name: str = "",
        extra: str = "",
    ) -> None:
        class_id = int(class_id)
        if not class_name and class_id >= 0:
            class_name = self._class_name(class_id)
        text = (
            f"[TARGET] event={event}, id={int(track_id)}, "
            f"class={class_name or 'unknown'}, class_id={class_id}"
        )
        if extra:
            text = f"{text}, {extra}"
        self.get_logger().info(text)

    @staticmethod
    def _default_config() -> dict:
        return {
            "SYSTEM": {"cmd_vel_topic": "/cmd_vel"},
            "PTZ": {
                "angle_topic": "/cmd_ptz_angle",
                "servo1_min_angle": -90.0,
                "servo1_max_angle": 90.0,
                "servo2_min_angle": -90.0,
                "servo2_max_angle": 20.0,
                "servo1_initial_angle": 0.0,
                "servo2_initial_angle": -45.0,
                "publish_count": 1,
                "publish_interval_sec": 0.05,
                "settle_time_sec": 0.3,
                "initialize_on_startup": True,
                "startup_publish_count": 10,
                "startup_publish_interval_sec": 0.1,
                "recenter_after_stop": True,
            },
            "YOLO_ONNX_FOLLOW": {
                "model_path": "",
                "class_names_path": "",
                "image_topic": "/camera/image_raw",
                "input_width": 640,
                "input_height": 640,
                "conf_threshold": 0.2,
                "nms_threshold": 0.45,
                "max_detections": 3,
                "has_objectness": "auto",
                "use_cuda": False,
                "show_window": True,
                "diagnostic_log_enable": False,
                "display_rate_hz": 20.0,
                "window_width": 960,
                "window_height": 720,
                "window_name": "YOLOv11 ONNX Follow",
                "target_class": "",
                "target_class_id": -1,
                "target_id": -1,
                "track_iou_threshold": 0.2,
                "track_center_threshold_ratio": 0.12,
                "track_max_lost_frames": 30,
                "reacquire_enable": True,
                "reacquire_iou_threshold": 0.05,
                "reacquire_max_center_ratio": 0.45,
                "reacquire_max_lost_frames": 30,
                "reacquire_score_weight": 0.15,
                "target_center_smoothing_alpha": 0.65,
                "max_inference_rate_hz": 15.0,
                "follow_rate_hz": 10.0,
                "target_stale_timeout_sec": 0.6,
                "pixel_tolerance": 15.0,
                "kp_x": 0.6,
                "kd_x": 0.15,
                "kp_y": 0.6,
                "kd_y": 0.15,
                "max_step_xy": 5.0,
                "sign_x": 1.0,
                "sign_y": -1.0,
                "enable_body_turn_follow": True,
                "body_kp": 0.03,
                "body_kd": 0.0,
                "body_tolerance_deg": 3.0,
                "servo1_engage_deg": 5.0,
                "max_angular_z": 0.6,
                "body_dir": -1.0,
                "body_ff_gain": 1.0,
                "fixed_camera_body_kp": 0.4,
                "fixed_camera_body_kd": 0.0,
                "fixed_camera_max_angular_z": 0.3,
            },
        }

    def _load_class_names(self) -> list[str]:
        path = str(self.demo_config.get("class_names_path", "") or "")
        if path and os.path.exists(path):
            with open(path, "r", encoding="utf-8") as names_file:
                names = [line.strip() for line in names_file if line.strip()]
            if names:
                return names
        return list(COCO_CLASS_NAMES)

    def _initial_track_id(self) -> int:
        try:
            return int(self.demo_config.get("target_id", -1))
        except (TypeError, ValueError):
            return -1

    def _initial_class_name(self) -> str:
        return str(self.demo_config.get("target_class", "") or "").strip().lower()

    def _initial_class_id(self) -> int:
        try:
            return int(self.demo_config.get("target_class_id", -1))
        except (TypeError, ValueError):
            return -1

    # ---- startup -------------------------------------------------------

    def _init_ros(self) -> None:
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        cmd_vel_topic = self.config.get("SYSTEM", {}).get("cmd_vel_topic", "/cmd_vel")
        image_topic = self.demo_config.get("image_topic", "/camera/image_raw")
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, qos)
        self.image_sub = self.create_subscription(
            Image, image_topic, self._image_callback, qos
        )
        self._log_diagnostic(
            f"[CONFIG] image_topic={image_topic}, cmd_vel_topic={cmd_vel_topic}"
        )

    def _load_model(self) -> None:
        model_path = str(self.demo_config.get("model_path", "") or "")
        if not model_path or not os.path.exists(model_path):
            raise FileNotFoundError(
                f"YOLO ONNX model_path is invalid: {model_path or '<empty>'}"
            )
        self.net = cv2.dnn.readNetFromONNX(model_path)
        if bool(self.demo_config.get("use_cuda", False)):
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
        self._log_diagnostic(f"[YOLO_ONNX] model loaded: {model_path}")

    def _init_ui(self) -> None:
        if not bool(self.demo_config.get("show_window", True)):
            return
        window_name = str(self.demo_config.get("window_name", "YOLOv11 ONNX Follow"))
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        window_width = max(320, int(self.demo_config.get("window_width", 960)))
        window_height = max(240, int(self.demo_config.get("window_height", 720)))
        cv2.resizeWindow(window_name, window_width, window_height)
        cv2.setMouseCallback(window_name, self._on_mouse)
        self._show_waiting_frame("Starting, waiting for camera frames...")

    # ---- image and inference ------------------------------------------

    def _image_callback(self, msg: Image) -> None:
        now = time.monotonic()
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as error:
            self._log_diagnostic_warn(f"[YOLO_ONNX] image convert failed: {error}")
            return

        self._store_latest_frame(frame, msg, now)

        max_rate = max(0.1, float(self.demo_config.get("max_inference_rate_hz", 15.0)))
        if now - self.last_inference_time < 1.0 / max_rate:
            return
        if not self.inference_lock.acquire(blocking=False):
            return
        self.last_inference_time = now

        try:
            infer_start = time.monotonic()
            detections = self._run_yolo(frame)
            self.last_inference_duration = time.monotonic() - infer_start
            self._maybe_log_slow_inference(self.last_inference_duration)
            tracked = self.tracker.update(
                detections, image_width=frame.shape[1], image_height=frame.shape[0]
            )
            with self.frame_lock:
                self.latest_detections = tracked
            self._update_selected_target(tracked, frame.shape[1], frame.shape[0])
        finally:
            self.inference_lock.release()

    def _store_latest_frame(self, frame, msg: Image, now: float) -> None:
        with self.frame_lock:
            self.latest_frame = frame.copy()
            self.image_frame_count += 1
            frame_count = self.image_frame_count
            self.last_image_time = now

        if frame_count == 1 or now - self.last_frame_log_time >= 5.0:
            self.last_frame_log_time = now
            if self._diagnostic_log_enabled():
                height, width = frame.shape[:2]
                mean_value = float(np.mean(frame)) if frame.size else 0.0
                self._log_diagnostic(
                    "[IMAGE] frame "
                    f"#{frame_count}: {width}x{height}, "
                    f"encoding={msg.encoding}, mean={mean_value:.1f}"
                )

    def _maybe_log_slow_inference(self, duration: float) -> None:
        if not self._diagnostic_log_enabled():
            return
        if duration < 1.0:
            return
        now = time.monotonic()
        if now - self.last_inference_warning_time < 5.0:
            return
        self.last_inference_warning_time = now
        self._log_diagnostic_warn(
            f"[YOLO_ONNX] inference is slow: {duration:.2f}s/frame"
        )

    def _run_yolo(self, frame) -> list[Detection]:
        input_w = int(self.demo_config.get("input_width", 640))
        input_h = int(self.demo_config.get("input_height", 640))
        letterboxed, scale, pad_x, pad_y = self._letterbox(frame, input_w, input_h)
        blob = cv2.dnn.blobFromImage(
            letterboxed, 1.0 / 255.0, (input_w, input_h), swapRB=True, crop=False
        )
        self.net.setInput(blob)
        outputs = self.net.forward()
        return self._decode_outputs(outputs, frame.shape, scale, pad_x, pad_y)

    @staticmethod
    def _letterbox(frame, input_w: int, input_h: int):
        height, width = frame.shape[:2]
        scale = min(input_w / width, input_h / height)
        resized_w = int(round(width * scale))
        resized_h = int(round(height * scale))
        resized = cv2.resize(frame, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)
        canvas = np.full((input_h, input_w, 3), 114, dtype=np.uint8)
        pad_x = (input_w - resized_w) // 2
        pad_y = (input_h - resized_h) // 2
        canvas[pad_y : pad_y + resized_h, pad_x : pad_x + resized_w] = resized
        return canvas, scale, pad_x, pad_y

    def _decode_outputs(
        self, outputs, frame_shape, scale: float, pad_x: int, pad_y: int
    ) -> list[Detection]:
        conf_threshold = float(self.demo_config.get("conf_threshold", 0.2))
        raw = outputs[0] if isinstance(outputs, (list, tuple)) else outputs
        raw = np.squeeze(raw)
        if raw.ndim != 2:
            return []
        if raw.shape[0] < raw.shape[1] and raw.shape[0] in (6, 7, 84, 85):
            raw = raw.T

        boxes = []
        scores = []
        class_ids = []
        for row in raw:
            parsed = self._parse_yolo_row(row)
            if parsed is None:
                continue
            box, score, class_id = parsed
            if score < conf_threshold:
                continue
            x1, y1, x2, y2 = self._map_box_to_frame(box, frame_shape, scale, pad_x, pad_y)
            if x2 <= x1 or y2 <= y1:
                continue
            boxes.append([x1, y1, x2 - x1, y2 - y1])
            scores.append(float(score))
            class_ids.append(int(class_id))

        if not boxes:
            return []

        indices = cv2.dnn.NMSBoxes(
            boxes,
            scores,
            conf_threshold,
            float(self.demo_config.get("nms_threshold", 0.45)),
        )
        detections = []
        for index in np.array(indices).reshape(-1) if len(indices) else []:
            x, y, w, h = boxes[int(index)]
            class_id = int(class_ids[int(index)])
            class_name = self._class_name(class_id)
            detections.append(
                Detection(
                    bbox=(float(x), float(y), float(x + w), float(y + h)),
                    score=scores[int(index)],
                    class_id=class_id,
                    class_name=class_name,
                )
            )
        max_detections = int(self.demo_config.get("max_detections", 3))
        if max_detections > 0:
            detections = sorted(detections, key=lambda item: item.score, reverse=True)
            detections = detections[:max_detections]
        return detections

    def _parse_yolo_row(
        self, row
    ) -> tuple[tuple[float, float, float, float], float, int] | None:
        row = np.asarray(row, dtype=np.float32).reshape(-1)
        if row.size < 6:
            return None

        if row.size in (6, 7):
            x1, y1, x2, y2 = row[:4]
            score = float(row[4])
            class_id = int(row[5])
            return (float(x1), float(y1), float(x2), float(y2)), score, class_id

        class_count = max(1, len(self.class_names))
        has_objectness = str(self.demo_config.get("has_objectness", "auto")).lower()
        if has_objectness == "true":
            score_values = row[5:]
            objectness = float(row[4])
            class_id = int(np.argmax(score_values))
            score = objectness * float(score_values[class_id])
        elif has_objectness == "false":
            score_values = row[4:]
            class_id = int(np.argmax(score_values))
            score = float(score_values[class_id])
        elif row.size == class_count + 5:
            score_values = row[5:]
            objectness = float(row[4])
            class_id = int(np.argmax(score_values))
            score = objectness * float(score_values[class_id])
        else:
            score_values = row[4:]
            class_id = int(np.argmax(score_values))
            score = float(score_values[class_id])

        cx, cy, width, height = row[:4]
        return (
            (
                float(cx - width * 0.5),
                float(cy - height * 0.5),
                float(cx + width * 0.5),
                float(cy + height * 0.5),
            ),
            score,
            class_id,
        )

    def _map_box_to_frame(self, box, frame_shape, scale: float, pad_x: int, pad_y: int):
        height, width = frame_shape[:2]
        x1, y1, x2, y2 = box
        if max(abs(x1), abs(y1), abs(x2), abs(y2)) <= 1.5:
            input_w = float(self.demo_config.get("input_width", 640))
            input_h = float(self.demo_config.get("input_height", 640))
            x1, x2 = x1 * input_w, x2 * input_w
            y1, y2 = y1 * input_h, y2 * input_h
        x1 = (x1 - pad_x) / scale
        x2 = (x2 - pad_x) / scale
        y1 = (y1 - pad_y) / scale
        y2 = (y2 - pad_y) / scale
        return (
            clamp(x1, 0.0, float(width - 1)),
            clamp(y1, 0.0, float(height - 1)),
            clamp(x2, 0.0, float(width - 1)),
            clamp(y2, 0.0, float(height - 1)),
        )

    def _class_name(self, class_id: int) -> str:
        class_id = int(class_id)
        if 0 <= class_id < len(self.class_names):
            return self.class_names[class_id]
        return str(class_id)

    # ---- target selection ---------------------------------------------

    @staticmethod
    def _empty_target_state() -> dict:
        return {
            "valid": False,
            "stamp": 0.0,
            "cx": 0.0,
            "cy": 0.0,
            "image_width": 640,
            "image_height": 480,
            "track_id": -1,
            "class_name": "",
        }

    def _update_selected_target(
        self, detections: list[Detection], image_width: int, image_height: int
    ) -> None:
        selected = self._select_detection(detections, image_width, image_height)
        now = time.monotonic()
        if selected is None:
            has_selection = (
                self.selected_track_id > 0
                or self.selected_class_id >= 0
                or bool(self.selected_class_name)
            )
            if has_selection:
                self.selected_lost_frames += 1
                self._maybe_log_selected_lost()
            max_lost = max(
                1, int(self.demo_config.get("reacquire_max_lost_frames", 30))
            )
            if (
                self.selected_lost_frames > max_lost
                or not self.latest_target_state.get("valid", False)
            ):
                self.latest_target_state["valid"] = False
                self.latest_target_state["stamp"] = now
            return

        target_cx, target_cy = self._smooth_target_center(selected, now)
        self._remember_selected_detection(selected)
        self.latest_target_state.update(
            {
                "valid": True,
                "stamp": now,
                "cx": target_cx,
                "cy": target_cy,
                "image_width": image_width,
                "image_height": image_height,
                "track_id": int(selected.track_id),
                "class_name": selected.class_name,
            }
        )

    def _smooth_target_center(
        self, detection: Detection, now: float
    ) -> tuple[float, float]:
        raw_cx = float(detection.cx)
        raw_cy = float(detection.cy)
        previous = self.latest_target_state
        if not previous.get("valid", False):
            return raw_cx, raw_cy

        alpha = clamp(
            float(self.demo_config.get("target_center_smoothing_alpha", 0.65)),
            0.05,
            1.0,
        )
        if alpha >= 0.999:
            return raw_cx, raw_cy

        stale_timeout = max(
            0.1, float(self.demo_config.get("target_stale_timeout_sec", 0.6))
        )
        if now - float(previous.get("stamp", 0.0)) > stale_timeout:
            return raw_cx, raw_cy

        previous_track_id = int(previous.get("track_id", -1))
        same_track = previous_track_id == int(detection.track_id)
        same_class = str(previous.get("class_name", "")) == detection.class_name
        reacquired_same_class = (
            previous_track_id > 0
            and previous_track_id != int(detection.track_id)
            and same_class
            and self.selected_lost_frames > 0
        )
        if not same_track and not reacquired_same_class:
            return raw_cx, raw_cy

        prev_cx = float(previous.get("cx", raw_cx))
        prev_cy = float(previous.get("cy", raw_cy))
        return (
            prev_cx + (raw_cx - prev_cx) * alpha,
            prev_cy + (raw_cy - prev_cy) * alpha,
        )

    def _select_detection(
        self, detections: list[Detection], image_width: int, image_height: int
    ) -> Detection | None:
        if self.selected_track_id > 0:
            for detection in detections:
                if detection.track_id == self.selected_track_id:
                    return detection
            return self._try_reacquire_selected_detection(
                detections, image_width, image_height
            )

        if self.selected_class_id >= 0:
            matching = [
                detection
                for detection in detections
                if int(detection.class_id) == self.selected_class_id
            ]
            if not matching:
                return None
            return min(
                matching,
                key=lambda item: self._center_sort_key(
                    item, image_width, image_height
                ),
            )

        if self.selected_class_name:
            matching = [
                detection
                for detection in detections
                if self._matches_selected_class(detection)
            ]
            if not matching:
                return None
            return min(
                matching,
                key=lambda item: self._center_sort_key(
                    item, image_width, image_height
                ),
            )
        return None

    def _remember_selected_detection(self, detection: Detection) -> None:
        if (
            self.selected_track_id <= 0
            and self.selected_class_id < 0
            and not self.selected_class_name
        ):
            return

        self.selected_track_id = int(detection.track_id)
        self.selected_class_id = int(detection.class_id)
        self.selected_last_bbox = detection.bbox
        self.selected_last_seen_time = time.monotonic()
        self.selected_lost_frames = 0
        self.selected_lost_reported = False

    def _try_reacquire_selected_detection(
        self, detections: list[Detection], image_width: int, image_height: int
    ) -> Detection | None:
        if not bool(self.demo_config.get("reacquire_enable", True)):
            return None
        if self.selected_class_id < 0 or self.selected_last_bbox is None:
            return None
        max_lost = max(
            1, int(self.demo_config.get("reacquire_max_lost_frames", 30))
        )
        if self.selected_lost_frames > max_lost:
            return None

        candidates = [
            detection
            for detection in detections
            if int(detection.class_id) == self.selected_class_id
        ]
        if not candidates:
            return None

        diagonal = math.hypot(max(1.0, image_width), max(1.0, image_height))
        max_center_ratio = max(
            0.01, float(self.demo_config.get("reacquire_max_center_ratio", 0.45))
        )
        max_center_dist = diagonal * max_center_ratio
        iou_threshold = max(
            0.0, float(self.demo_config.get("reacquire_iou_threshold", 0.05))
        )

        scored = [
            self._reacquire_score(detection, max_center_dist)
            for detection in candidates
        ]
        scored = [item for item in scored if item is not None]
        if not scored:
            return None

        _score, selected = max(scored, key=lambda item: item[0])
        iou = SimpleIouTracker._iou(self.selected_last_bbox, selected.bbox)
        center_dist = self._bbox_center_distance(
            self.selected_last_bbox, selected.bbox
        )
        if iou < iou_threshold and center_dist > max_center_dist:
            return None

        old_id = self.selected_track_id
        self.selected_track_id = int(selected.track_id)
        self.selected_lost_frames = max(1, self.selected_lost_frames)
        now = time.monotonic()
        if now - self.last_reacquire_log_time >= 1.0:
            self.last_reacquire_log_time = now
            self._log_target_event(
                "reacquired",
                track_id=selected.track_id,
                class_id=selected.class_id,
                class_name=selected.class_name,
                extra=f"previous_id={old_id}",
            )
        return selected

    def _reacquire_score(
        self, detection: Detection, max_center_dist: float
    ) -> tuple[float, Detection] | None:
        iou = SimpleIouTracker._iou(self.selected_last_bbox, detection.bbox)
        center_dist = self._bbox_center_distance(self.selected_last_bbox, detection.bbox)
        if center_dist > max_center_dist and iou <= 0.0:
            return None

        score_weight = max(
            0.0, float(self.demo_config.get("reacquire_score_weight", 0.15))
        )
        distance_score = 1.0 - clamp(center_dist / max_center_dist, 0.0, 1.0)
        score = iou * 2.0 + distance_score + score_weight * detection.score
        return score, detection

    @staticmethod
    def _bbox_center_distance(a, b) -> float:
        ax1, ay1, ax2, ay2 = a
        bx1, by1, bx2, by2 = b
        acx = (ax1 + ax2) * 0.5
        acy = (ay1 + ay2) * 0.5
        bcx = (bx1 + bx2) * 0.5
        bcy = (by1 + by2) * 0.5
        return math.hypot(acx - bcx, acy - bcy)

    def _matches_selected_class(self, detection: Detection) -> bool:
        selected = self.selected_class_name
        if not selected:
            return False
        return (
            detection.class_name.lower() == selected
        )

    @staticmethod
    def _center_sort_key(detection: Detection, image_width: int, image_height: int) -> tuple:
        dx = detection.cx - image_width * 0.5
        dy = detection.cy - image_height * 0.5
        return dx * dx + dy * dy, -detection.score, -detection.width * detection.height

    def _on_mouse(self, event, x, y, flags, param) -> None:
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        with self.frame_lock:
            detections = list(self.latest_detections)
        for detection in reversed(detections):
            x1, y1, x2, y2 = detection.bbox
            if x1 <= x <= x2 and y1 <= y <= y2:
                self.selected_track_id = int(detection.track_id)
                self.selected_class_name = ""
                self.selected_class_id = int(detection.class_id)
                self.selected_last_bbox = detection.bbox
                self.selected_last_seen_time = time.monotonic()
                self.selected_lost_frames = 0
                self.selected_lost_reported = False
                self._log_target_event(
                    "selected",
                    track_id=detection.track_id,
                    class_id=detection.class_id,
                    class_name=detection.class_name,
                )
                return

    def _clear_selection(self) -> None:
        old_track_id = self.selected_track_id
        old_class_id = self.selected_class_id
        old_class_name = self.selected_class_name
        self.selected_track_id = -1
        self.selected_class_name = ""
        self.selected_class_id = -1
        self.selected_last_bbox = None
        self.selected_last_seen_time = 0.0
        self.selected_lost_frames = 0
        self.selected_lost_reported = False
        self.latest_target_state = self._empty_target_state()
        self._publish_stop_twist()
        self._reset_follow_controller()
        self._log_target_event(
            "cleared",
            track_id=old_track_id,
            class_id=old_class_id,
            class_name=old_class_name,
        )

    def _maybe_log_selected_lost(self) -> None:
        if self.selected_lost_reported:
            return
        max_lost = max(
            1, int(self.demo_config.get("reacquire_max_lost_frames", 30))
        )
        if self.selected_lost_frames <= max_lost:
            return
        self.selected_lost_reported = True
        self._log_target_event(
            "lost",
            track_id=self.selected_track_id,
            class_id=self.selected_class_id,
            class_name=self.selected_class_name,
        )

    # ---- display -------------------------------------------------------

    def pump_display_once(self) -> None:
        if not bool(self.demo_config.get("show_window", True)):
            return
        now = time.monotonic()
        if now - self.last_display_time < self.display_period_sec:
            return
        self.last_display_time = now
        self._display_tick()

    def _display_tick(self) -> None:
        if not bool(self.demo_config.get("show_window", True)):
            return

        with self.frame_lock:
            frame = None if self.latest_frame is None else self.latest_frame.copy()
            detections = list(self.latest_detections)
            frame_count = self.image_frame_count
            last_image_time = self.last_image_time

        if frame is None:
            self._show_waiting_frame(
                "Waiting for /camera/image_raw frames..."
            )
            return

        age = time.monotonic() - last_image_time if last_image_time > 0.0 else 0.0
        self._show_frame(frame, detections, frame_count=frame_count, frame_age=age)

    def _show_waiting_frame(self, text: str) -> None:
        if not bool(self.demo_config.get("show_window", True)):
            return
        view = np.zeros((480, 640, 3), dtype=np.uint8)
        lines = [
            text,
            f"topic: {self.demo_config.get('image_topic', '/camera/image_raw')}",
            "check: ros2 topic hz /camera/image_raw",
            "press q/Esc to quit",
        ]
        for row, line in enumerate(lines):
            cv2.putText(
                view,
                line,
                (24, 72 + row * 36),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
        window_name = str(self.demo_config.get("window_name", "YOLOv11 ONNX Follow"))
        cv2.imshow(window_name, view)
        self._handle_window_key()

    def _show_frame(
        self,
        frame,
        detections: list[Detection],
        frame_count: int = 0,
        frame_age: float = 0.0,
    ) -> None:
        if not bool(self.demo_config.get("show_window", True)):
            return
        view = frame.copy()
        for detection in detections:
            selected = detection.track_id == self.selected_track_id
            color = (0, 255, 255) if selected else (0, 180, 0)
            x1, y1, x2, y2 = [int(value) for value in detection.bbox]
            cv2.rectangle(view, (x1, y1), (x2, y2), color, 2)
            label = (
                f"{detection.class_name} id={detection.track_id} "
                f"{detection.score:.2f}"
            )
            cv2.putText(
                view,
                label,
                (x1, max(20, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
                cv2.LINE_AA,
            )

        status = "click target | r reselect | q quit"
        if self.selected_track_id > 0:
            status = f"following id={self.selected_track_id} | r reselect | q quit"
        elif self.selected_class_name:
            status = f"following class={self.selected_class_name} | r reselect | q quit"
        infer_ms = self.last_inference_duration * 1000.0
        debug_status = (
            f"frame={frame_count} det={len(detections)} "
            f"age={frame_age:.2f}s infer={infer_ms:.0f}ms"
        )
        cv2.putText(
            view,
            status,
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            view,
            debug_status,
            (10, 52),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        window_name = str(self.demo_config.get("window_name", "YOLOv11 ONNX Follow"))
        cv2.imshow(window_name, view)
        self._handle_window_key()

    def _handle_window_key(self) -> None:
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("r"), ord("R")):
            self._clear_selection()
        elif key in (ord("q"), ord("Q"), 27):
            self.shutdown_requested = True

    def _image_watchdog_tick(self) -> None:
        if not self._diagnostic_log_enabled():
            return
        now = time.monotonic()
        if self.last_image_time <= 0.0:
            if now - self.last_image_watchdog_log_time >= 5.0:
                self.last_image_watchdog_log_time = now
                image_topic = self.demo_config.get("image_topic", "/camera/image_raw")
                self._log_diagnostic_warn(
                    "[IMAGE] no frames received yet from "
                    f"{image_topic}; check topic name/type and camera publisher"
                )
            return

        stale_sec = now - self.last_image_time
        if stale_sec >= 3.0 and now - self.last_image_watchdog_log_time >= 5.0:
            self.last_image_watchdog_log_time = now
            self._log_diagnostic_warn(
                f"[IMAGE] last frame is stale: {stale_sec:.1f}s ago"
            )

    # ---- follow control ------------------------------------------------

    def _follow_tick(self) -> None:
        if self.shutdown_requested:
            self.cleanup()
            if rclpy.ok():
                rclpy.shutdown()
            return

        cfg = self._follow_config()
        state = dict(self.latest_target_state)
        now = time.monotonic()
        fresh = (now - float(state["stamp"])) <= cfg["stale_timeout_sec"]
        if not state["valid"] or not fresh:
            self._publish_stop_twist()
            self._reset_follow_controller()
            return

        if self.prev_control_time > 0.0:
            dt = now - self.prev_control_time
        else:
            dt = 1.0 / cfg["rate_hz"]
        dt = max(dt, 1e-3)
        if self.ptz is not None:
            self._apply_ptz_follow(state, cfg, dt)
        twist = self._compute_chassis_twist(state, cfg, dt)
        self.cmd_vel_pub.publish(twist)
        self._log_motion_state(twist)
        self.prev_control_time = now

    def _follow_config(self) -> dict:
        sign_x = clamp(float(self.demo_config.get("sign_x", 1.0)), -1.0, 1.0)
        return {
            "rate_hz": max(1.0, float(self.demo_config.get("follow_rate_hz", 10.0))),
            "stale_timeout_sec": max(
                0.1, float(self.demo_config.get("target_stale_timeout_sec", 0.6))
            ),
            "pixel_tolerance": max(
                0.0, float(self.demo_config.get("pixel_tolerance", 15.0))
            ),
            "ptz_kp": max(0.0, float(self.demo_config.get("kp_x", 0.6))),
            "ptz_kd": max(0.0, float(self.demo_config.get("kd_x", 0.15))),
            "ptz_kp_y": max(0.0, float(self.demo_config.get("kp_y", 0.6))),
            "ptz_kd_y": max(0.0, float(self.demo_config.get("kd_y", 0.15))),
            "max_ptz_step": max(
                0.0, float(self.demo_config.get("max_step_xy", 5.0))
            ),
            "ptz_sign_x": sign_x,
            "ptz_sign_y": clamp(
                float(self.demo_config.get("sign_y", -1.0)), -1.0, 1.0
            ),
            "enable_body_turn_follow": bool(
                self.demo_config.get("enable_body_turn_follow", True)
            ),
            "body_kp": max(0.0, float(self.demo_config.get("body_kp", 0.03))),
            "body_kd": max(0.0, float(self.demo_config.get("body_kd", 0.0))),
            "body_tolerance_deg": max(
                0.0, float(self.demo_config.get("body_tolerance_deg", 3.0))
            ),
            "servo1_engage_deg": max(
                1.0, float(self.demo_config.get("servo1_engage_deg", 5.0))
            ),
            "max_angular_z": max(
                0.0, float(self.demo_config.get("max_angular_z", 0.6))
            ),
            "body_dir": clamp(
                float(self.demo_config.get("body_dir", -sign_x)), -1.0, 1.0
            ),
            "body_ff_gain": clamp(
                float(self.demo_config.get("body_ff_gain", 1.0)), 0.0, 3.0
            ),
            "fixed_camera_body_kp": max(
                0.0, float(self.demo_config.get("fixed_camera_body_kp", 0.4))
            ),
            "fixed_camera_body_kd": max(
                0.0, float(self.demo_config.get("fixed_camera_body_kd", 0.0))
            ),
            "fixed_camera_max_angular_z": max(
                0.0,
                float(self.demo_config.get("fixed_camera_max_angular_z", 0.3)),
            ),
        }

    def _apply_ptz_follow(self, state: dict, cfg: dict, dt: float) -> None:
        if abs(self.last_body_az) > 1e-6:
            body_yaw_deg = math.degrees(self.last_body_az * dt)
            vor_deg = cfg["body_ff_gain"] * cfg["ptz_sign_x"] * body_yaw_deg
            self.ptz.servo1_angle = clamp(
                self.ptz.servo1_angle + vor_deg,
                self.ptz.servo1_min_angle,
                self.ptz.servo1_max_angle,
            )

        half_w = max(1.0, float(state["image_width"]) * 0.5)
        half_h = max(1.0, float(state["image_height"]) * 0.5)

        err_x = float(state["cx"]) - half_w
        if abs(err_x) <= cfg["pixel_tolerance"]:
            self.prev_err_x = 0.0
            step_x = 0.0
        else:
            norm_x = clamp(err_x / half_w, -1.0, 1.0)
            deriv_x = norm_x - self.prev_err_x
            out_x = clamp(
                cfg["ptz_kp"] * norm_x + cfg["ptz_kd"] * deriv_x, -1.0, 1.0
            )
            step_x = out_x * cfg["max_ptz_step"]
            self.prev_err_x = norm_x

        err_y = float(state["cy"]) - half_h
        if abs(err_y) <= cfg["pixel_tolerance"]:
            self.prev_err_y = 0.0
            step_y = 0.0
        else:
            norm_y = clamp(err_y / half_h, -1.0, 1.0)
            deriv_y = norm_y - self.prev_err_y
            out_y = clamp(
                cfg["ptz_kp_y"] * norm_y + cfg["ptz_kd_y"] * deriv_y, -1.0, 1.0
            )
            step_y = out_y * cfg["max_ptz_step"]
            self.prev_err_y = norm_y

        self.ptz.publish_servo_angles(
            servo1_angle=clamp(
                self.ptz.servo1_angle + cfg["ptz_sign_x"] * step_x,
                self.ptz.servo1_min_angle,
                self.ptz.servo1_max_angle,
            ),
            servo2_angle=clamp(
                self.ptz.servo2_angle + cfg["ptz_sign_y"] * step_y,
                self.ptz.servo2_min_angle,
                self.ptz.servo2_max_angle,
            ),
            repeat=False,
        )

    def _compute_chassis_twist(
        self, state: dict, cfg: dict, dt: float
    ) -> Twist:
        twist = Twist()
        if cfg["enable_body_turn_follow"]:
            if self.ptz is None:
                twist.angular.z = self._compute_fixed_camera_body_turn(
                    state, cfg, dt
                )
            else:
                twist.angular.z = self._compute_body_turn(cfg, dt)
            self.last_body_az = float(twist.angular.z)
        else:
            self.last_body_az = 0.0
        return twist

    def _compute_fixed_camera_body_turn(
        self, state: dict, cfg: dict, dt: float
    ) -> float:
        half_w = max(1.0, float(state["image_width"]) * 0.5)
        error_px = float(state["cx"]) - half_w
        if abs(error_px) <= cfg["pixel_tolerance"]:
            self.prev_body_error_eff = 0.0
            return 0.0

        normalized_error = clamp(error_px / half_w, -1.0, 1.0)
        derivative = (normalized_error - self.prev_body_error_eff) / max(dt, 1e-3)
        angular_z = cfg["body_dir"] * (
            cfg["fixed_camera_body_kp"] * normalized_error
            + cfg["fixed_camera_body_kd"] * derivative
        )
        self.prev_body_error_eff = normalized_error
        max_angular_z = cfg["fixed_camera_max_angular_z"]
        return clamp(angular_z, -max_angular_z, max_angular_z)

    def _compute_body_turn(self, cfg: dict, dt: float) -> float:
        servo_err = self._get_pan_error_deg()
        if abs(servo_err) >= cfg["servo1_engage_deg"]:
            self.body_correcting = True
        elif abs(servo_err) <= cfg["body_tolerance_deg"]:
            self.body_correcting = False

        if not self.body_correcting:
            self.prev_body_error_eff = 0.0
            return 0.0

        deriv = (servo_err - self.prev_body_error_eff) / max(dt, 1e-3)
        angular_z = cfg["body_dir"] * (
            cfg["body_kp"] * servo_err + cfg["body_kd"] * deriv
        )
        angular_z = clamp(angular_z, -cfg["max_angular_z"], cfg["max_angular_z"])
        self.prev_body_error_eff = servo_err
        return angular_z

    def _get_pan_error_deg(self) -> float:
        initial_angle = float(getattr(self.ptz, "servo1_initial_angle", 0.0))
        servo_angle = float(getattr(self.ptz, "servo1_angle", initial_angle))
        return servo_angle - initial_angle

    def _reset_follow_controller(self) -> None:
        self.prev_err_x = 0.0
        self.prev_err_y = 0.0
        self.prev_control_time = 0.0
        self.last_body_az = 0.0
        self.body_correcting = False
        self.prev_body_error_eff = 0.0

    def _publish_stop_twist(self) -> None:
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def _log_motion_state(self, twist: Twist) -> None:
        if not self._diagnostic_log_enabled():
            return
        now = time.monotonic()
        if now - self.last_motion_log_time < 0.5:
            return
        self.last_motion_log_time = now
        angular_z = float(getattr(getattr(twist, "angular", None), "z", 0.0))
        if self.ptz is None:
            detail = "fixed_camera"
        else:
            detail = f"servo1={self.ptz.servo1_angle:.2f}deg"
        self._log_diagnostic(
            f"[YOLO_ONNX][CMD] {detail}, x=0.000, z={angular_z:.3f}"
        )

    # ---- node API required by PtzController ----------------------------

    def publish_progress(self, text: str) -> None:
        self._log_diagnostic(str(text or ""))

    def reject_unsupported_action(self, action: str, reason: str) -> bool:
        self.get_logger().error(f"[CAPABILITY] {action}: {reason}")
        return False

    def should_interrupt(self) -> bool:
        return False

    def cleanup(self) -> None:
        if self._cleaned_up:
            return
        self._cleaned_up = True
        if rclpy.ok():
            self._publish_stop_twist()
            if self.ptz is not None:
                self.ptz.recenter_after_stop()
        if bool(self.demo_config.get("show_window", True)):
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = YoloOnnxFollowNode()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        while rclpy.ok() and not node.shutdown_requested:
            executor.spin_once(timeout_sec=0.01)
            node.pump_display_once()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.cleanup()
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

