#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
april_tag_follow_ptz_node.py

中文说明:
    ROS2 Humble AprilTag 跟随节点（云台 + 车身）。
    AprilTag 检测保留原有实现，云台和车身控制与
    tracking_bbox_follow_node 使用同一套 KCF 风格控制器。

    AprilTag 专属部分只保留:
    - dt_apriltags.Detector 检测
    - camera.yaml 相机内参加载
    - 主 Tag 选择
    - pose_z 结果显示

运行前准备:
    source ~/workspaces/MicroROS-V2-ROS2-SRC/start_yolo_ros2_env.sh
"""

from __future__ import annotations

import os
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import cv2 as cv
import numpy as np
import rclpy
import yaml
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from .image_conversion import image_to_bgr
from .kcf_style_follow_control import (
    CONTROL_DYNAMIC_PARAMETER_NAMES,
    CONTROL_RESTART_REQUIRED_PARAMETER_NAMES,
    KcfStyleFollowController,
)

try:
    from dt_apriltags import Detector as AprilTagDetector
except ImportError:
    AprilTagDetector = None


DEFAULT_NODE_NAME = "april_tag_follow_ptz_node"

STATE_IDLE = "idle"
STATE_TRACKING = "tracking"

KEY_CODE_SPACE = 32
KEY_CODE_ESCAPE = 27
MINIMUM_TIMER_RATE_HZ = 1.0

DEFAULT_CAMERA_YAML = (
    "/home/yahboom/workspaces/MicroROS-V2-ROS2-SRC/microros_car_v2_ws/"
    "src/microros_v2_opencv_visual_pkg/config/camera.yaml"
)

DYNAMIC_PARAMETER_NAMES = CONTROL_DYNAMIC_PARAMETER_NAMES | frozenset({
    "show_debug_window",
    "image_width", "image_height", "window_width", "window_height",
    "overlay_font_scale", "overlay_line_height",
    "overlay_margin_x", "overlay_margin_y", "overlay_show_background",
    "enable_horizontal_flip", "enable_vertical_flip", "rotate_image_code",
    "lost_frame_threshold",
    "tag_family", "tag_size", "quad_decimate", "tag_select_strategy",
    "camera_yaml_path", "use_camera_yaml",
})

RESTART_REQUIRED_PARAMETER_NAMES = CONTROL_RESTART_REQUIRED_PARAMETER_NAMES | frozenset(
    {"image_topic", "window_name"}
)


def to_bool(v: Any, default: bool) -> bool:
    if isinstance(v, bool):
        return v
    if isinstance(v, str):
        n = v.strip().lower()
        if n in ("true", "1", "yes", "y", "on"):
            return True
        if n in ("false", "0", "no", "n", "off"):
            return False
    if isinstance(v, (int, float)):
        return bool(v)
    return default


def load_camera_calibration(yaml_path: str) -> Optional[Tuple[float, float, float, float]]:
    path = os.path.expanduser(yaml_path)
    if not path or not os.path.exists(path):
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        cm = data.get("camera_matrix", {}).get("data", [])
        if len(cm) < 9:
            return None
        return float(cm[0]), float(cm[4]), float(cm[2]), float(cm[5])
    except Exception:
        return None


class AprilTagFollowPtzNode(Node):

    def __init__(self) -> None:
        super().__init__(DEFAULT_NODE_NAME)
        self._tgt_lock = threading.Lock()
        self._latest_frame: Optional[np.ndarray] = None

        self._state = STATE_TRACKING
        self._shutdown = False

        self._has_target = False
        self._lost_frames = 0
        self._last_target_time = 0.0
        self._tag_id = -1
        self._target_x: Optional[float] = None
        self._target_y: Optional[float] = None
        self._tag_pose_x: Optional[float] = None
        self._tag_pose_y: Optional[float] = None
        self._tag_pose_z: Optional[float] = None
        self._tag_corners: List[Tuple[int, int]] = []

        self._fps = 0.0
        self._fps_n = 0
        self._fps_t = time.perf_counter()
        self._win_ok = False
        self._detector = None
        self._camera_params: Optional[Tuple[float, float, float, float]] = None

        self._declare_params()
        self._load_params()
        self._init_detector()
        self._controller = KcfStyleFollowController(
            self, self._get_control_target, self._control_is_active
        )

        video_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self._sub = self.create_subscription(
            Image, self._img_topic, self._on_image, video_qos
        )
        self._key_timer = None
        if self._show_win:
            self._init_win()
            self._key_timer = self.create_timer(0.05, self._poll_keys)

        self._param_cb = self.add_on_set_parameters_callback(self._on_param_update)
        self.get_logger().info(
            f"AprilTag follow started with KCF-style PTZ/body control at "
            f"{self._controller.follow_rate_hz:.1f}Hz. "
            f"family={self._tag_family} tag_size={self._tag_size:.3f}m"
        )

    def _declare_params(self) -> None:
        KcfStyleFollowController.declare_parameters(self)
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("window_name", "april_tag_follow_ptz")
        self.declare_parameter("show_debug_window", True)
        self.declare_parameter("window_width", 640)
        self.declare_parameter("window_height", 480)
        self.declare_parameter("overlay_font_scale", 0.38)
        self.declare_parameter("overlay_line_height", 16)
        self.declare_parameter("overlay_margin_x", 8)
        self.declare_parameter("overlay_margin_y", 16)
        self.declare_parameter("overlay_show_background", True)
        self.declare_parameter("enable_horizontal_flip", False)
        self.declare_parameter("enable_vertical_flip", False)
        self.declare_parameter("rotate_image_code", "none")
        self.declare_parameter("lost_frame_threshold", 5)
        self.declare_parameter("tag_family", "tag36h11")
        self.declare_parameter("tag_size", 0.048)
        self.declare_parameter("quad_decimate", 2.0)
        self.declare_parameter("tag_select_strategy", "largest")
        self.declare_parameter("camera_yaml_path", DEFAULT_CAMERA_YAML)
        self.declare_parameter("use_camera_yaml", True)

    def _load_params(self) -> None:
        self._img_topic = str(self.get_parameter("image_topic").value)
        self._win_name = str(self.get_parameter("window_name").value)
        self._show_win = to_bool(self.get_parameter("show_debug_window").value, True)
        self._img_w = int(self.get_parameter("image_width").value)
        self._img_h = int(self.get_parameter("image_height").value)
        self._win_w = int(self.get_parameter("window_width").value)
        self._win_h = int(self.get_parameter("window_height").value)
        self._font = float(self.get_parameter("overlay_font_scale").value)
        self._lh = int(self.get_parameter("overlay_line_height").value)
        self._mx = int(self.get_parameter("overlay_margin_x").value)
        self._my = int(self.get_parameter("overlay_margin_y").value)
        self._ov_bg = to_bool(self.get_parameter("overlay_show_background").value, True)
        self._flip_h = to_bool(self.get_parameter("enable_horizontal_flip").value, False)
        self._flip_v = to_bool(self.get_parameter("enable_vertical_flip").value, False)
        self._rot = str(self.get_parameter("rotate_image_code").value).lower()
        self._lost_frame_threshold = max(1, int(self.get_parameter("lost_frame_threshold").value))
        self._tag_family = str(self.get_parameter("tag_family").value)
        self._tag_size = float(self.get_parameter("tag_size").value)
        self._quad_decimate = float(self.get_parameter("quad_decimate").value)
        self._tag_strategy = str(self.get_parameter("tag_select_strategy").value).lower()
        self._camera_yaml = str(self.get_parameter("camera_yaml_path").value)
        self._use_yaml = to_bool(self.get_parameter("use_camera_yaml").value, True)
        self._rqt = to_bool(self.get_parameter("enable_rqt_dynamic_parameters").value, True)

    def _init_detector(self) -> None:
        if AprilTagDetector is None:
            self.get_logger().error("dt_apriltags not installed. pip install dt-apriltags")
            return
        self._detector = AprilTagDetector(
            families=self._tag_family,
            nthreads=4,
            quad_decimate=float(self._quad_decimate),
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0,
        )
        self._camera_params = None
        if self._use_yaml:
            cp = load_camera_calibration(self._camera_yaml)
            if cp:
                self._camera_params = cp
                self.get_logger().info(f"Loaded camera calibration from {self._camera_yaml}")
            else:
                self.get_logger().warn("Camera yaml not found. Pose estimation disabled.")

    def _on_image(self, msg: Image) -> None:
        try:
            frame = image_to_bgr(msg)
            if frame is None:
                return
            if self._flip_h:
                frame = cv.flip(frame, 1)
            if self._flip_v:
                frame = cv.flip(frame, 0)
            if self._rot == "90_clockwise":
                frame = cv.rotate(frame, cv.ROTATE_90_CLOCKWISE)
            elif self._rot == "90_counterclockwise":
                frame = cv.rotate(frame, cv.ROTATE_90_COUNTERCLOCKWISE)
            elif self._rot == "180":
                frame = cv.rotate(frame, cv.ROTATE_180)
            if self._img_w > 0 and self._img_h > 0:
                frame = cv.resize(frame, (self._img_w, self._img_h))
            self._latest_frame = frame
        except Exception:
            return

        disp = frame.copy()
        self._detect(disp)

        if self._show_win:
            self._draw_overlay(disp)
            cv.imshow(self._win_name, disp)

        self._fps_n += 1
        now = time.perf_counter()
        if now - self._fps_t >= 1.0:
            self._fps = self._fps_n / (now - self._fps_t)
            self._fps_n = 0
            self._fps_t = now

    def _detect(self, frame: np.ndarray) -> None:
        if self._detector is None:
            return

        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        frame_w, frame_h = frame.shape[1], frame.shape[0]

        cam_params = None
        if self._camera_params is not None:
            fx, fy, cx, cy = self._camera_params
            sx = frame_w / 320.0
            sy = frame_h / 240.0
            cam_params = (fx * sx, fy * sy, cx * sx, cy * sy)

        try:
            tags = self._detector.detect(
                gray,
                estimate_tag_pose=cam_params is not None,
                camera_params=cam_params,
                tag_size=float(self._tag_size),
            )
        except Exception:
            tags = []

        if not tags:
            self._mark_lost()
            return

        if self._tag_strategy == "smallest_id":
            main = min(tags, key=lambda t: t.tag_id)
        else:
            main = max(tags, key=lambda t: cv.contourArea(np.array(t.corners, dtype=np.float32).reshape(-1, 2)))

        for tag in tags:
            corners = np.array(tag.corners, dtype=np.int32).reshape((-1, 1, 2))
            is_main = tag.tag_id == main.tag_id
            color = (255, 0, 255) if is_main else (0, 255, 0)
            cv.polylines(frame, [corners], True, color, 2)
            cx_t, cy_t = int(tag.center[0]), int(tag.center[1])
            cv.circle(frame, (cx_t, cy_t), 3, (0, 0, 255), -1)
            cv.putText(frame, f"id:{tag.tag_id}", (cx_t + 5, cy_t - 5),
                       cv.FONT_HERSHEY_SIMPLEX, 0.3, color, 1, cv.LINE_AA)

        pose_x, pose_y, pose_z = None, None, None
        if hasattr(main, "pose_t") and main.pose_t is not None:
            pt = np.asarray(main.pose_t).ravel()
            if pt.size >= 3:
                pose_x, pose_y, pose_z = float(pt[0]), float(pt[1]), float(pt[2])

        with self._tgt_lock:
            self._lost_frames = 0
            self._has_target = True
            self._last_target_time = time.monotonic()
            self._tag_id = int(main.tag_id)
            self._target_x = float(main.center[0])
            self._target_y = float(main.center[1])
            self._tag_pose_x = pose_x
            self._tag_pose_y = pose_y
            self._tag_pose_z = pose_z
            self._tag_corners = [(int(c[0]), int(c[1])) for c in main.corners]

    def _mark_lost(self) -> None:
        with self._tgt_lock:
            self._lost_frames += 1
            if self._lost_frames >= self._lost_frame_threshold:
                self._has_target = False

    def _control_is_active(self) -> bool:
        return self._state == STATE_TRACKING

    def _get_control_target(self) -> Dict[str, Any]:
        with self._tgt_lock:
            tx, ty = self._target_x, self._target_y
            corners = list(self._tag_corners)
            valid = self._has_target and tx is not None and ty is not None
            stamp = self._last_target_time
        if corners:
            xs = [corner[0] for corner in corners]
            ys = [corner[1] for corner in corners]
            width = max(xs) - min(xs)
            height = max(ys) - min(ys)
        else:
            width = 0
            height = 0
        return {
            "valid": valid and width > 0 and height > 0,
            "stamp": stamp,
            "cx": float(tx or 0.0),
            "cy": float(ty or 0.0),
            "width": float(width),
            "height": float(height),
            "image_width": self._img_w,
            "image_height": self._img_h,
        }

    def _pub_zero_cmd(self) -> None:
        self._controller.publish_zero_twist()

    def _init_win(self) -> None:
        cv.namedWindow(self._win_name, cv.WINDOW_NORMAL)
        cv.resizeWindow(self._win_name, self._win_w, self._win_h)

    def _poll_keys(self) -> None:
        try:
            k = cv.waitKey(1) & 0xFF
        except Exception:
            return
        if k in (255, -1):
            return
        if k in (ord("q"), ord("Q"), KEY_CODE_ESCAPE):
            self._shutdown = True
            self._pub_zero_cmd()
            try:
                self.get_logger().info("Force exiting.")
            except Exception:
                pass
            os._exit(0)
        elif k == KEY_CODE_SPACE:
            if self._state == STATE_TRACKING:
                self._state = STATE_IDLE
                self._reset_pd()
                self._pub_zero_cmd()
                self.get_logger().info("IDLE")
            else:
                self._state = STATE_TRACKING
                self._reset_pd()
                self.get_logger().info("TRACKING (PTZ+body)")
        elif k in (ord("i"), ord("I")):
            self._state = STATE_IDLE
            self._reset_pd()
            self._pub_zero_cmd()
            self.get_logger().info("IDLE")

    def _pub_ptz(self) -> None:
        self._controller.publish_ptz_angle()

    def _reset_pd(self) -> None:
        self._controller.reset()
        self._lost_frames = 0

    def _draw_overlay(self, frame: np.ndarray) -> None:
        iw, ih = frame.shape[1], frame.shape[0]
        icx, icy = iw // 2, ih // 2
        cv.drawMarker(frame, (icx, icy), (0, 255, 255), cv.MARKER_CROSS, 20, 2)
        tolerance = int(self._controller.pixel_tolerance)
        cv.rectangle(frame, (icx - tolerance, icy - tolerance),
                     (icx + tolerance, icy + tolerance), (0, 180, 255), 2)
        with self._tgt_lock:
            ok = self._has_target
            tx, ty = self._target_x, self._target_y
            pz = self._tag_pose_z
            tag_id = self._tag_id
            lf = self._lost_frames
        tgt_txt = (
            f"Tag:{tag_id} Target:{int(tx)},{int(ty)}"
            if ok and tx is not None and ty is not None
            else f"Tag:None(lost{lf}/{self._lost_frame_threshold})"
        )
        dist_txt = f"Pose z:{pz:.3f}m" if pz is not None else "Pose z:--"
        lines = [
            f"State:{self._state} FPS:{int(self._fps)} {tgt_txt}",
            f"PTZ s1:{self._controller.horizontal_angle:.1f} s2:{self._controller.pitch_angle:.1f} body_kp:{self._controller.body_kp:.2f}",
            f"{dist_txt}",
            f"PD x:{self._controller.ptz_kp_x:.2f}/{self._controller.ptz_kd_x:.2f} step:{self._controller.max_ptz_step:.1f} FF:{self._controller.body_ff_gain:.1f}",
            "SPACE:toggle | i:idle | q:quit",
        ]
        if self._ov_bg:
            bh = self._lh * len(lines) + 8
            cv.rectangle(frame, (0, 0), (iw, min(ih, bh)), (0, 0, 0), -1)
        for i, t in enumerate(lines):
            cv.putText(frame, t, (self._mx, self._my + i * self._lh),
                       cv.FONT_HERSHEY_SIMPLEX, self._font, (0, 255, 255), 1, cv.LINE_AA)

    def _on_param_update(self, params: List[Parameter]) -> SetParametersResult:
        if not params:
            return SetParametersResult(successful=True)
        names = {p.name for p in params}
        bad = names - DYNAMIC_PARAMETER_NAMES - RESTART_REQUIRED_PARAMETER_NAMES
        if bad:
            return SetParametersResult(successful=False, reason=f"Unsupported: {sorted(bad)}")
        if names & RESTART_REQUIRED_PARAMETER_NAMES:
            return SetParametersResult(successful=False, reason="Restart required")
        if not self._rqt and names != {"enable_rqt_dynamic_parameters"}:
            return SetParametersResult(successful=False, reason="Disabled")

        old_debug, old_size = self._show_win, (self._win_w, self._win_h)
        v = self._merge(params)
        err = self._check(v)
        if err:
            return SetParametersResult(successful=False, reason=err)
        self._apply(v)

        self._sync_win(old_debug, old_size)
        return SetParametersResult(successful=True)

    def _merge(self, params: List[Parameter]) -> Dict[str, Any]:
        d = dict(
            show_debug_window=self._show_win,
            image_width=self._img_w, image_height=self._img_h,
            window_width=self._win_w, window_height=self._win_h,
            overlay_font_scale=self._font, overlay_line_height=self._lh,
            overlay_margin_x=self._mx, overlay_margin_y=self._my, overlay_show_background=self._ov_bg,
            enable_horizontal_flip=self._flip_h, enable_vertical_flip=self._flip_v,
            rotate_image_code=self._rot,
            lost_frame_threshold=self._lost_frame_threshold,
            tag_family=self._tag_family, tag_size=self._tag_size,
            quad_decimate=self._quad_decimate, tag_select_strategy=self._tag_strategy,
            camera_yaml_path=self._camera_yaml, use_camera_yaml=self._use_yaml,
            enable_rqt_dynamic_parameters=self._rqt,
        )
        for p in params:
            if p.type_ != Parameter.Type.NOT_SET:
                d[p.name] = p.value
        return d

    def _check(self, v: Dict[str, Any]) -> str:
        control_error = self._controller.validate_parameter_overrides(v)
        if control_error:
            return control_error
        try:
            if int(v["lost_frame_threshold"]) < 1:
                return "lost_frame_threshold >=1"
            if float(v["tag_size"]) <= 0:
                return "tag_size >0"
            if float(v["quad_decimate"]) < 1.0:
                return "quad_decimate >=1"
        except Exception as e:
            return str(e)
        return ""

    def _apply(self, v: Dict[str, Any]) -> None:
        self._show_win = to_bool(v["show_debug_window"], True)
        self._img_w = int(v["image_width"])
        self._img_h = int(v["image_height"])
        self._win_w = int(v["window_width"])
        self._win_h = int(v["window_height"])
        self._font = float(v["overlay_font_scale"])
        self._lh = int(v["overlay_line_height"])
        self._mx = int(v["overlay_margin_x"])
        self._my = int(v["overlay_margin_y"])
        self._ov_bg = to_bool(v["overlay_show_background"], True)
        self._flip_h = to_bool(v["enable_horizontal_flip"], False)
        self._flip_v = to_bool(v["enable_vertical_flip"], False)
        self._rot = str(v["rotate_image_code"]).lower()
        self._lost_frame_threshold = max(1, int(v["lost_frame_threshold"]))
        self._tag_family = str(v["tag_family"])
        self._tag_size = float(v["tag_size"])
        self._quad_decimate = float(v["quad_decimate"])
        self._tag_strategy = str(v["tag_select_strategy"]).lower()
        self._camera_yaml = str(v["camera_yaml_path"])
        self._use_yaml = to_bool(v["use_camera_yaml"], True)
        self._rqt = to_bool(v["enable_rqt_dynamic_parameters"], True)
        self._controller.apply_parameter_overrides(v)
        self._init_detector()
        self._reset_pd()

    def _sync_win(self, prev_debug: bool, prev_size: Tuple[int, int]) -> None:
        try:
            chg = prev_size != (self._win_w, self._win_h)
            if self._show_win and not prev_debug:
                self._init_win()
            elif not self._show_win and prev_debug:
                cv.destroyWindow(self._win_name)
            elif self._show_win and chg:
                cv.resizeWindow(self._win_name, self._win_w, self._win_h)
        except Exception:
            pass

    def destroy_node(self) -> bool:
        self._controller.shutdown()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagFollowPtzNode()
    try:
        while rclpy.ok() and not node._shutdown:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
