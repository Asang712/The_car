#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
qr_follow_node.py
中文说明:
    ROS2 Humble QR 码检测 + 车身跟随节点（No-PTZ 版）。
    摄像头固定朝向正前方。车身 PD 基于像素误差直接控制角速度 + QR 尺寸距离跟随。
    与 qr_follow_ptz_node 对应，去掉 PTZ 层，车身 100% 承担水平跟踪。
English:
    ROS2 Humble QR code detection + body follow node (No-PTZ version).
    Camera fixed forward. Body PD uses pixel error directly for angular velocity + QR size distance follow.
"""
from __future__ import annotations
import math
import os
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple
import cv2 as cv
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from .image_conversion import image_to_bgr
try:
    import pyzbar.pyzbar as pyzbar_module
except Exception:
    pyzbar_module = None
DEFAULT_NODE_NAME = "qr_follow_node"
STATE_IDLE = "idle"
STATE_TRACKING = "tracking"
KEY_CODE_SPACE = 32
KEY_CODE_ESCAPE = 27
MINIMUM_TIMER_RATE_HZ = 1.0
DYNAMIC_PARAMETER_NAMES = frozenset({
    "show_debug_window",
    "processing_width", "processing_height",
    "display_width", "display_height",
    "overlay_font_scale", "overlay_line_height", "overlay_margin_x", "overlay_margin_y",
    "overlay_show_background",
    "enable_horizontal_flip", "enable_vertical_flip", "rotate_image_code",
    "qr_detector_backend", "qr_select_strategy",
    "lost_timeout", "follow_stable_frames",
    "deadband_x_pixel",
    "angular_kp", "angular_kd", "max_angular_z",
    "lost_frame_threshold", "reverse_body_direction",
    "enable_distance_follow", "target_qr_size_px", "distance_tolerance_qr_size_px",
    "distance_kp", "distance_kd", "max_linear_x", "reverse_linear_direction",
    "image_processing_rate_hz", "body_control_rate",
    "enable_rqt_dynamic_parameters",
    "show_deadband_lines", "show_center_line",
})
RESTART_REQUIRED_PARAMETER_NAMES = frozenset({
    "image_topic", "cmd_vel_topic", "window_name",
})
def clamp(value: float, lo: float, hi: float) -> float:
    return max(float(lo), min(float(hi), float(value)))
class QrFollowNode(Node):
    def __init__(self) -> None:
        super().__init__(DEFAULT_NODE_NAME)
        self._image_lock = threading.Lock()
        self._track_lock = threading.Lock()
        self._latest_frame: Optional[np.ndarray] = None
        self._latest_seq = 0
        self._processed_seq = 0
        self._state = STATE_IDLE
        self._shutdown = False
        self._has_target = False
        self._lost_frames = 0
        self._target_x: Optional[float] = None
        self._target_y: Optional[float] = None
        self._target_size_px: Optional[float] = None
        self._stable_count = 0
        self._prev_s1 = 0.0
        self._prev_body_t = 0.0
        self._last_body_az = 0.0
        self._prev_size_err = 0.0
        self._prev_linear_t = 0.0
        self._measured_fps = 0.0
        self._fps_count = 0
        self._fps_last_t = time.perf_counter()
        self._win_ready = False
        self._last_qr_update_t = 0.0
        self._qr_code_detector = cv.QRCodeDetector()
        self._declare_params()
        self._load_params()
        best_effort_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._img_sub = self.create_subscription(
            Image, self._img_topic, self._on_image, best_effort_qos)
        self._cmd_pub = self.create_publisher(Twist, self._cmd_topic, best_effort_qos)
        self._img_timer = self.create_timer(
            1.0 / max(float(self._img_rate), MINIMUM_TIMER_RATE_HZ), self._process_image)
        self._body_timer = self.create_timer(
            1.0 / max(float(self._body_rate), MINIMUM_TIMER_RATE_HZ), self._body_tick)
        self._key_timer = None
        if self._show_win:
            self._ensure_win()
            self._key_timer = self.create_timer(0.05, self._poll_keys)
        self._param_cb = self.add_on_set_parameters_callback(self._on_param_update)
        self.get_logger().info(
            "QrFollow body started. SPACE=toggle, i=idle, q=quit. img=%.0fHz body=%.0fHz"
            % (self._img_rate, self._body_rate))
    def _declare_params(self) -> None:
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("window_name", "qr_follow")
        self.declare_parameter("show_debug_window", True)
        self.declare_parameter("processing_width", 320)
        self.declare_parameter("processing_height", 240)
        self.declare_parameter("display_width", 960)
        self.declare_parameter("display_height", 720)
        self.declare_parameter("overlay_font_scale", 0.32)
        self.declare_parameter("overlay_line_height", 13)
        self.declare_parameter("overlay_margin_x", 6)
        self.declare_parameter("overlay_margin_y", 16)
        self.declare_parameter("overlay_show_background", False)
        self.declare_parameter("enable_horizontal_flip", False)
        self.declare_parameter("enable_vertical_flip", False)
        self.declare_parameter("rotate_image_code", "none")
        self.declare_parameter("qr_detector_backend", "pyzbar")
        self.declare_parameter("qr_select_strategy", "largest")
        self.declare_parameter("lost_timeout", 0.5)
        self.declare_parameter("follow_stable_frames", 3)
        self.declare_parameter("deadband_x_pixel", 50.0)
        self.declare_parameter("angular_kp", 1.6)
        self.declare_parameter("angular_kd", 0.75)
        self.declare_parameter("max_angular_z", 0.6)
        self.declare_parameter("lost_frame_threshold", 5)
        self.declare_parameter("reverse_body_direction", True)
        self.declare_parameter("enable_distance_follow", True)
        self.declare_parameter("target_qr_size_px", 120.0)
        self.declare_parameter("distance_tolerance_qr_size_px", 20.0)
        self.declare_parameter("distance_kp", 0.05)
        self.declare_parameter("distance_kd", 0.03)
        self.declare_parameter("max_linear_x", 0.18)
        self.declare_parameter("reverse_linear_direction", False)
        self.declare_parameter("image_processing_rate_hz", 15.0)
        self.declare_parameter("body_control_rate", 10.0)
        self.declare_parameter("show_deadband_lines", False)
        self.declare_parameter("show_center_line", False)
        self.declare_parameter("enable_rqt_dynamic_parameters", True)
    def _load_params(self) -> None:
        self._img_topic = str(self.get_parameter("image_topic").value)
        self._cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._win_name = str(self.get_parameter("window_name").value)
        self._show_win = bool(self.get_parameter("show_debug_window").value)
        self._proc_w = int(self.get_parameter("processing_width").value)
        self._proc_h = int(self.get_parameter("processing_height").value)
        self._disp_w = int(self.get_parameter("display_width").value)
        self._disp_h = int(self.get_parameter("display_height").value)
        self._font_scale = float(self.get_parameter("overlay_font_scale").value)
        self._line_h = int(self.get_parameter("overlay_line_height").value)
        self._ovl_x = int(self.get_parameter("overlay_margin_x").value)
        self._ovl_y = int(self.get_parameter("overlay_margin_y").value)
        self._bg = bool(self.get_parameter("overlay_show_background").value)
        self._flip_h = bool(self.get_parameter("enable_horizontal_flip").value)
        self._flip_v = bool(self.get_parameter("enable_vertical_flip").value)
        self._rot = str(self.get_parameter("rotate_image_code").value)
        self._qr_backend = str(self.get_parameter("qr_detector_backend").value).lower()
        self._qr_strategy = str(self.get_parameter("qr_select_strategy").value).lower()
        self._lost_timeout = max(float(self.get_parameter("lost_timeout").value), 0.05)
        self._follow_stable = max(1, int(self.get_parameter("follow_stable_frames").value))
        self._db_x = float(self.get_parameter("deadband_x_pixel").value)
        self._body_kp = max(float(self.get_parameter("angular_kp").value), 0.0)
        self._body_kd = max(float(self.get_parameter("angular_kd").value), 0.0)
        self._max_angular_z = max(float(self.get_parameter("max_angular_z").value), 0.0)
        self._lost_thr = max(1, int(self.get_parameter("lost_frame_threshold").value))
        self._rev_body = bool(self.get_parameter("reverse_body_direction").value)
        self._enable_distance = bool(self.get_parameter("enable_distance_follow").value)
        self._target_qr_size_px = float(self.get_parameter("target_qr_size_px").value)
        self._distance_tol = max(float(self.get_parameter("distance_tolerance_qr_size_px").value), 0.0)
        self._distance_kp = max(float(self.get_parameter("distance_kp").value), 0.0)
        self._distance_kd = max(float(self.get_parameter("distance_kd").value), 0.0)
        self._max_linear_x = max(float(self.get_parameter("max_linear_x").value), 0.0)
        self._rev_linear = bool(self.get_parameter("reverse_linear_direction").value)
        self._img_rate = max(float(self.get_parameter("image_processing_rate_hz").value), MINIMUM_TIMER_RATE_HZ)
        self._body_rate = max(float(self.get_parameter("body_control_rate").value), MINIMUM_TIMER_RATE_HZ)
        self._dyn = bool(self.get_parameter("enable_rqt_dynamic_parameters").value)
        self._show_deadband = bool(self.get_parameter("show_deadband_lines").value)
        self._show_center = bool(self.get_parameter("show_center_line").value)
    def _on_image(self, msg: Image) -> None:
        try:
            frame = image_to_bgr(msg)
        except Exception as exception:
            self.get_logger().warn(f"Image conversion failed: {exception}")
            return
        with self._image_lock:
            self._latest_frame = frame
            self._latest_seq += 1
    def _process_image(self) -> None:
        if self._shutdown:
            return
        with self._image_lock:
            if self._latest_frame is None:
                return
            seq = self._latest_seq
        if seq == self._processed_seq:
            return
        self._processed_seq = seq
        with self._image_lock:
            frame = self._latest_frame.copy()
        frame = cv.resize(frame, (int(self._proc_w), int(self._proc_h)))
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
        self._detect(frame)
        self._draw_guides(frame)
        self._draw_overlay(frame)
        self._update_fps()
        if self._show_win:
            self._ensure_win()
            cv.imshow(self._win_name, cv.resize(frame, (self._disp_w, self._disp_h)))
    def _poll_keys(self) -> None:
        k = cv.waitKey(1) & 0xFF
        if k == 255:
            return
        if k in (ord("q"), ord("Q"), KEY_CODE_ESCAPE):
            self._shutdown = True
            os._exit(0)
        if k in (ord("i"), ord("I")):
            self._state = STATE_IDLE
            self._reset_pd()
            self._pub_zero_cmd()
            self.get_logger().info("IDLE")
            return
        if k == KEY_CODE_SPACE:
            if self._state == STATE_TRACKING:
                self._state = STATE_IDLE
                self._reset_pd()
                self._pub_zero_cmd()
                self.get_logger().info("IDLE")
            else:
                self._state = STATE_TRACKING
                self._reset_pd()
                self.get_logger().info("TRACKING")
    def _detect(self, frame: np.ndarray) -> None:
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        qr_cx, qr_cy, qr_w = None, None, None
        if self._qr_backend in {"pyzbar", "auto"} and pyzbar_module:
            barcodes = pyzbar_module.decode(gray)
            if barcodes:
                bc = barcodes[0]
                rect = bc.rect
                qr_cx = float(rect.left + rect.width * 0.5)
                qr_cy = float(rect.top + rect.height * 0.5)
                qr_w = float(max(rect.width, rect.height))
                cv.rectangle(frame, (rect.left, rect.top),
                           (rect.left + rect.width, rect.top + rect.height), (0, 255, 0), 2)
                cv.putText(frame, bc.data.decode("utf-8", errors="ignore")[:20],
                          (rect.left, max(20, rect.top - 10)),
                          cv.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
        if qr_cx is None:
            detection_ok, decoded_info, points, _ = self._qr_code_detector.detectAndDecodeMulti(frame)
            if detection_ok and points is not None and decoded_info:
                pts = points[0].reshape(-1, 2).astype(np.int32)
                qr_cx = float(np.mean(pts[:, 0]))
                qr_cy = float(np.mean(pts[:, 1]))
                x, y, w, h = cv.boundingRect(pts)
                qr_w = float(max(w, h))
                cv.polylines(frame, [pts], True, (0, 255, 0), 2)
        now = time.perf_counter()
        if qr_cx is not None:
            cv.circle(frame, (int(qr_cx), int(qr_cy)), 5, (0, 0, 255), -1)
            self._last_qr_update_t = now
            prev_has = self._has_target
            with self._track_lock:
                self._has_target = True
                self._lost_frames = 0
                self._target_x = qr_cx
                self._target_y = qr_cy
                self._target_size_px = qr_w
                if prev_has:
                    self._stable_count += 1
                else:
                    self._stable_count = 1
            return
        # no QR found
        if now - self._last_qr_update_t > self._lost_timeout:
            with self._track_lock:
                self._has_target = False
                self._lost_frames += 1
                if self._lost_frames >= self._lost_thr:
                    self._target_x = None
                    self._target_y = None
                    self._target_size_px = None
                    self._stable_count = 0
        else:
            with self._track_lock:
                self._lost_frames += 1
    # ── Body control ─────────────────────────────────────────
    def _reset_pd(self) -> None:
        self._prev_s1 = 0.0
        self._prev_body_t = 0.0
        self._last_body_az = 0.0
        self._prev_size_err = 0.0
        self._prev_linear_t = 0.0
    def _body_tick(self) -> None:
        if self._state != STATE_TRACKING:
            self._pub_zero_cmd()
            return
        with self._track_lock:
            has_target = self._has_target
            tx = self._target_x
            qr_size = self._target_size_px
            stable = self._stable_count
        if not has_target or tx is None or stable < self._follow_stable:
            self._pub_zero_cmd()
            return
        icx = float(self._proc_w) * 0.5
        pixel_err = float(tx - icx)
        if abs(pixel_err) < self._db_x:
            pixel_err = 0.0
        s1 = pixel_err / icx
        n = time.perf_counter()
        dt_body = n - self._prev_body_t if self._prev_body_t > 0.0 else 1.0 / max(float(self._body_rate), MINIMUM_TIMER_RATE_HZ)
        dt_body = max(dt_body, 1e-3)
        ds1 = (s1 - self._prev_s1) / dt_body
        az = self._body_kp * s1 + self._body_kd * ds1
        az = clamp(az, -self._max_angular_z, self._max_angular_z)
        if self._rev_body:
            az = -az
        cmd = Twist()
        cmd.angular.z = float(az)
        linear_x = 0.0
        if self._enable_distance and qr_size is not None:
            size_err = float(self._target_qr_size_px - qr_size)
            if abs(size_err) < self._distance_tol:
                size_err = 0.0
            dt_linear = n - self._prev_linear_t if self._prev_linear_t > 0.0 else dt_body
            dt_linear = max(dt_linear, 1e-3)
            d_err = (size_err - self._prev_size_err) / dt_linear
            linear_x = self._distance_kp * size_err + self._distance_kd * d_err
            linear_x = clamp(linear_x, -self._max_linear_x, self._max_linear_x)
            if self._rev_linear:
                linear_x = -linear_x
            self._prev_size_err = size_err
            self._prev_linear_t = n
        else:
            self._prev_size_err = 0.0
            self._prev_linear_t = 0.0
        cmd.linear.x = float(linear_x)
        self._cmd_pub.publish(cmd)
        self._prev_s1 = s1
        self._prev_body_t = n
        self._last_body_az = az
    def _pub_zero_cmd(self) -> None:
        self._cmd_pub.publish(Twist())
    # ── Display ─────────────────────────────────────────────
    def _ensure_win(self) -> None:
        if self._win_ready:
            return
        cv.namedWindow(self._win_name, cv.WINDOW_NORMAL)
        cv.resizeWindow(self._win_name, self._disp_w, self._disp_h)
        self._win_ready = True
    def _draw_guides(self, frame: np.ndarray) -> None:
        cx, cy = int(self._proc_w * 0.5), int(self._proc_h * 0.5)
        cv.line(frame, (cx, 0), (cx, self._proc_h), (255, 255, 0), 1)
        cv.line(frame, (0, cy), (self._proc_w, cy), (255, 255, 0), 1)
        if self._db_x > 0:
            # 死区边界两条橙色竖线，间距 = 2 × deadband_x_pixel
            if self._show_deadband:
                cv.line(frame, (cx - int(self._db_x), 0), (cx - int(self._db_x), frame.shape[0]), (0, 180, 255), 2)
                cv.line(frame, (cx + int(self._db_x), 0), (cx + int(self._db_x), frame.shape[0]), (0, 180, 255), 2)
        # 画面水平正中间红色竖线
        if self._show_center:
            cv.line(frame, (cx, 0), (cx, frame.shape[0]), (0, 0, 255), 2)
    def _draw_overlay(self, frame: np.ndarray) -> None:
        with self._track_lock:
            state = self._state
            has_target = self._has_target
            qr_size = self._target_size_px
            stable = self._stable_count
            lost = self._lost_frames
        sz = qr_size if qr_size else 0.0
        lines = [
            f"State:{state} FPS:{int(self._measured_fps)} QR:{'YES' if has_target else 'NO'} stable:{stable} lost:{lost}",
            f"Body kp:{self._body_kp:.2f} kd:{self._body_kd:.2f} max_az:{self._max_angular_z:.2f} dead:{self._db_x:.0f} dist:{'on' if self._enable_distance else 'off'}",
            f"QR size:{sz:.0f}/{self._target_qr_size_px:.0f}px err:{self._target_qr_size_px - sz:.0f}",
            "SPACE:toggle | i:idle | q:quit",
        ]
        if self._bg:
            cv.rectangle(frame, (0, 0), (min(self._proc_w, 640), self._line_h * len(lines) + 8), (0, 0, 0), -1)
        for i, text in enumerate(lines):
            cv.putText(frame, text, (self._ovl_x, self._ovl_y + i * self._line_h),
                       cv.FONT_HERSHEY_SIMPLEX, self._font_scale, (0, 255, 255), 1, cv.LINE_AA)
    def _update_fps(self) -> None:
        self._fps_count += 1
        n = time.perf_counter()
        elapsed = n - self._fps_last_t
        if elapsed >= 1.0:
            self._measured_fps = self._fps_count / elapsed
            self._fps_count = 0
            self._fps_last_t = n
    def _on_param_update(self, params: List[Parameter]) -> SetParametersResult:
        if not self._dyn:
            return SetParametersResult(successful=False, reason="Dynamic parameters disabled.")
        try:
            should_recreate_img = False
            should_recreate_body = False
            for p in params:
                if p.name in RESTART_REQUIRED_PARAMETER_NAMES:
                    return SetParametersResult(successful=False, reason=f"'{p.name}' requires restart.")
                if p.name not in DYNAMIC_PARAMETER_NAMES:
                    continue
                if p.name == "image_processing_rate_hz":
                    should_recreate_img = True
                if p.name == "body_control_rate":
                    should_recreate_body = True
                if p.name == "show_debug_window":
                    if not bool(p.value) and self._win_ready:
                        cv.destroyWindow(self._win_name)
                        self._win_ready = False
            self._load_params()
            if should_recreate_img:
                self.destroy_timer(self._img_timer)
                self._img_timer = self.create_timer(1.0 / max(float(self._img_rate), MINIMUM_TIMER_RATE_HZ), self._process_image)
            if should_recreate_body:
                self.destroy_timer(self._body_timer)
                self._body_timer = self.create_timer(1.0 / max(float(self._body_rate), MINIMUM_TIMER_RATE_HZ), self._body_tick)
            return SetParametersResult(successful=True)
        except Exception as e:
            return SetParametersResult(successful=False, reason=str(e))
    def destroy_node(self) -> bool:
        self._shutdown = True
        self._pub_zero_cmd()
        if self._show_win:
            cv.destroyAllWindows()
        return super().destroy_node()
def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = QrFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
if __name__ == "__main__":
    main()
