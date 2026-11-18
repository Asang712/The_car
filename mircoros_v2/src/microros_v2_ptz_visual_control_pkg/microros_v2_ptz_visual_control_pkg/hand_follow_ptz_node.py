#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
hand_follow_ptz_node.py

中文说明:
    ROS2 Humble 手掌检测 + PTZ 云台 + 车身跟随节点。
    PTZ 控制框架：归一化像素误差 → PD → 步进限幅 → 平滑 → /cmd_ptz_angle。
    车身控制框架：PTZ 水平角度 → 车身 share PD → /cmd_vel.angular.z +
                   手部包围框高度 → 距离 PD → /cmd_vel.linear.x。
    与 gesture_follow_ptz_node 架构完全一致，仅检测层简化为纯手掌识别。

English:
    ROS2 Humble hand detection + PTZ + body follow node.
    Same architecture as gesture_follow_ptz_node, with palm-only detection.
"""

from __future__ import annotations

import os

os.environ.setdefault("MEDIAPIPE_DISABLE_TF", "1")
os.environ.setdefault("MEDIAPIPE_DISABLE_GPU", "1")

import threading
import time
from typing import Any, Dict, List, Optional

import cv2 as cv
import numpy as np
import rclpy
from geometry_msgs.msg import Twist, Vector3
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from .image_conversion import image_to_bgr

from .hand_detector import HandDetector


DEFAULT_NODE_NAME = "hand_follow_ptz_node"
LANDMARK_INDEX_FINGER_MCP = 5

STATE_IDLE = "idle"
STATE_TRACKING = "tracking"

KEY_CODE_SPACE = 32
KEY_CODE_ESCAPE = 27
MINIMUM_TIMER_RATE_HZ = 1.0

DYNAMIC_PARAMETER_NAMES = frozenset({
    "show_debug_window",
    "processing_width", "processing_height",
    "display_width", "display_height",
    "overlay_font_scale", "overlay_line_height",
    "overlay_show_background",
    "enable_horizontal_flip", "enable_vertical_flip", "rotate_image_code",
    "horizontal_min_deg", "horizontal_max_deg",
    "pitch_min_deg", "pitch_max_deg",
    "reverse_horizontal_direction", "reverse_pitch_direction",
    "deadband_x", "deadband_y",
    "pd_kp", "pd_kd", "ptz_step_smooth_alpha", "max_ptz_step",
    "ptz_smooth_alpha", "ptz_control_rate", "image_processing_rate_hz",
    "body_control_rate", "angular_kp", "angular_kd", "angular_tolerance_deg", "max_angular_z",
    "body_share", "body_ff_gain", "lost_frame_threshold", "reverse_body_direction",
    "enable_distance_follow", "target_hand_size_px", "distance_tolerance_hand_size_px",
    "distance_kp", "distance_kd", "max_linear_x", "reverse_linear_direction",
    "enable_rqt_dynamic_parameters",
})

RESTART_REQUIRED_PARAMETER_NAMES = frozenset({
    "image_topic", "ptz_command_topic", "cmd_vel_topic", "window_name",
})


def clamp(value: float, lo: float, hi: float) -> float:
    return max(float(lo), min(float(hi), float(value)))


class HandFollowPtzNode(Node):

    def __init__(self) -> None:
        super().__init__(DEFAULT_NODE_NAME)
        self._image_lock = threading.Lock()
        self._track_lock = threading.Lock()

        self._latest_frame: Optional[np.ndarray] = None
        self._latest_seq = 0
        self._processed_seq = 0

        self._state = STATE_IDLE
        self._shutdown = False

        self._has_hand = False
        self._has_target = False
        self._lost_frames = 0
        self._finger_count = 0
        self._target_x: Optional[float] = None
        self._target_y: Optional[float] = None
        self._target_size_px: Optional[float] = None

        self._h_deg = 0.0
        self._p_deg = 0.0

        self._prev_err_x = 0.0
        self._prev_err_y = 0.0
        self._prev_ctrl_t = 0.0
        self._last_sx = 0.0
        self._last_sy = 0.0

        self._prev_s1 = 0.0
        self._prev_s1_eff = 0.0
        self._prev_body_t = 0.0
        self._last_body_az = 0.0

        self._prev_size_err = 0.0
        self._prev_linear_t = 0.0

        self._measured_fps = 0.0
        self._fps_count = 0
        self._fps_last_t = time.perf_counter()
        self._win_ready = False

        self._declare_params()
        self._load_params()

        self._h_deg = 0.0
        self._p_deg = -45.0

        self._hand_det = HandDetector()

        video_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._img_sub = self.create_subscription(
            Image, self._img_topic, self._on_image, video_qos
        )
        self._ptz_pub = self.create_publisher(Vector3, self._ptz_topic, 10)
        self._cmd_pub = self.create_publisher(Twist, self._cmd_topic, 10)

        self._img_timer = self.create_timer(
            1.0 / max(float(self._img_rate), MINIMUM_TIMER_RATE_HZ), self._process_image
        )
        self._ptz_timer = self.create_timer(
            1.0 / max(float(self._ptz_rate), MINIMUM_TIMER_RATE_HZ), self._ptz_tick
        )
        self._body_timer = None
        self._rebuild_body_timer(False)

        self._key_timer = None
        if self._show_win:
            self._ensure_win()
            self._key_timer = self.create_timer(0.05, self._poll_keys)

        self._param_cb = self.add_on_set_parameters_callback(self._on_param_update)

        # 发布初始 PTZ 角度
        self._pub_ptz()

        self.get_logger().info(
            "HandFollow PTZ+body started. SPACE=toggle, q=quit. "
            "ptz=%.0fHz img=%.0fHz body=%.0fHz"
            % (self._ptz_rate, self._img_rate, self._body_rate)
        )

    def _declare_params(self) -> None:
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("ptz_command_topic", "/cmd_ptz_angle")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("window_name", "hand_follow_ptz")
        self.declare_parameter("show_debug_window", True)
        self.declare_parameter("processing_width", 320)
        self.declare_parameter("processing_height", 240)
        self.declare_parameter("display_width", 960)
        self.declare_parameter("display_height", 720)
        self.declare_parameter("overlay_font_scale", 0.32)
        self.declare_parameter("overlay_line_height", 13)
        self.declare_parameter("overlay_show_background", False)
        self.declare_parameter("enable_horizontal_flip", False)
        self.declare_parameter("enable_vertical_flip", False)
        self.declare_parameter("rotate_image_code", "none")
        self.declare_parameter("horizontal_min_deg", -90.0)
        self.declare_parameter("horizontal_max_deg", 90.0)
        self.declare_parameter("pitch_min_deg", -90.0)
        self.declare_parameter("pitch_max_deg", 20.0)
        self.declare_parameter("reverse_horizontal_direction", False)
        self.declare_parameter("reverse_pitch_direction", True)
        self.declare_parameter("deadband_x", 20.0)
        self.declare_parameter("deadband_y", 20.0)
        self.declare_parameter("pd_kp", 0.7)
        self.declare_parameter("pd_kd", 0.25)
        self.declare_parameter("ptz_step_smooth_alpha", 0.35)
        self.declare_parameter("max_ptz_step", 1.2)
        self.declare_parameter("ptz_smooth_alpha", 0.85)
        self.declare_parameter("ptz_control_rate", 20.0)
        self.declare_parameter("image_processing_rate_hz", 15.0)
        self.declare_parameter("body_control_rate", 10.0)
        self.declare_parameter("angular_kp", 0.05)
        self.declare_parameter("angular_kd", 0.05)
        self.declare_parameter("body_share", 0.5)
        self.declare_parameter("body_ff_gain", 1.3)
        self.declare_parameter("lost_frame_threshold", 5)
        self.declare_parameter("angular_tolerance_deg", 3.0)
        self.declare_parameter("max_angular_z", 0.12)
        self.declare_parameter("reverse_body_direction", True)
        self.declare_parameter("enable_distance_follow", False)
        self.declare_parameter("target_hand_size_px", 120.0)
        self.declare_parameter("distance_tolerance_hand_size_px", 12.0)
        self.declare_parameter("distance_kp", 0.008)
        self.declare_parameter("distance_kd", 0.003)
        self.declare_parameter("max_linear_x", 0.12)
        self.declare_parameter("reverse_linear_direction", False)
        self.declare_parameter("enable_rqt_dynamic_parameters", True)

    def _load_params(self) -> None:
        self._img_topic = str(self.get_parameter("image_topic").value)
        self._ptz_topic = str(self.get_parameter("ptz_command_topic").value)
        self._cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._win_name = str(self.get_parameter("window_name").value)
        self._show_win = bool(self.get_parameter("show_debug_window").value)
        self._proc_w = int(self.get_parameter("processing_width").value)
        self._proc_h = int(self.get_parameter("processing_height").value)
        self._disp_w = int(self.get_parameter("display_width").value)
        self._disp_h = int(self.get_parameter("display_height").value)
        self._font_scale = float(self.get_parameter("overlay_font_scale").value)
        self._line_h = int(self.get_parameter("overlay_line_height").value)
        self._bg = bool(self.get_parameter("overlay_show_background").value)
        self._flip_h = bool(self.get_parameter("enable_horizontal_flip").value)
        self._flip_v = bool(self.get_parameter("enable_vertical_flip").value)
        self._rot = str(self.get_parameter("rotate_image_code").value)
        self._h_min = float(self.get_parameter("horizontal_min_deg").value)
        self._h_max = float(self.get_parameter("horizontal_max_deg").value)
        self._p_min = float(self.get_parameter("pitch_min_deg").value)
        self._p_max = float(self.get_parameter("pitch_max_deg").value)
        self._rev_h = bool(self.get_parameter("reverse_horizontal_direction").value)
        self._rev_p = bool(self.get_parameter("reverse_pitch_direction").value)
        self._db_x = float(self.get_parameter("deadband_x").value)
        self._db_y = float(self.get_parameter("deadband_y").value)
        self._kp = max(float(self.get_parameter("pd_kp").value), 0.0)
        self._kd = max(float(self.get_parameter("pd_kd").value), 0.0)
        self._step_alpha = clamp(float(self.get_parameter("ptz_step_smooth_alpha").value), 0.0, 1.0)
        self._max_step = max(float(self.get_parameter("max_ptz_step").value), 0.0)
        self._ptz_alpha = clamp(float(self.get_parameter("ptz_smooth_alpha").value), 0.01, 1.0)
        self._ptz_rate = max(float(self.get_parameter("ptz_control_rate").value), MINIMUM_TIMER_RATE_HZ)
        self._img_rate = max(float(self.get_parameter("image_processing_rate_hz").value), MINIMUM_TIMER_RATE_HZ)
        self._body_rate = max(float(self.get_parameter("body_control_rate").value), MINIMUM_TIMER_RATE_HZ)
        self._body_kp = max(float(self.get_parameter("angular_kp").value), 0.0)
        self._body_kd = max(float(self.get_parameter("angular_kd").value), 0.0)
        self._body_share = float(self.get_parameter("body_share").value)
        self._body_ff_gain = float(self.get_parameter("body_ff_gain").value)
        self._lost_thr = max(1, int(self.get_parameter("lost_frame_threshold").value))
        self._ang_tol_deg = max(float(self.get_parameter("angular_tolerance_deg").value), 0.0)
        self._max_angular_z = max(float(self.get_parameter("max_angular_z").value), 0.0)
        self._rev_body = bool(self.get_parameter("reverse_body_direction").value)
        self._enable_distance = bool(self.get_parameter("enable_distance_follow").value)
        self._target_hand_size_px = float(self.get_parameter("target_hand_size_px").value)
        self._distance_tol_px = max(float(self.get_parameter("distance_tolerance_hand_size_px").value), 0.0)
        self._distance_kp = max(float(self.get_parameter("distance_kp").value), 0.0)
        self._distance_kd = max(float(self.get_parameter("distance_kd").value), 0.0)
        self._max_linear_x = max(float(self.get_parameter("max_linear_x").value), 0.0)
        self._rev_linear = bool(self.get_parameter("reverse_linear_direction").value)
        self._dyn = bool(self.get_parameter("enable_rqt_dynamic_parameters").value)

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
        try:
            self._hand_det.findHands(frame, draw=False)
        except TypeError:
            self._hand_det.findHands(frame)

        if len(self._hand_det.lmList) == 0:
            with self._track_lock:
                self._has_hand = False
                self._finger_count = 0
                self._lost_frames += 1
                if self._lost_frames >= self._lost_thr:
                    self._has_target = False
                    self._target_x = None
                    self._target_y = None
                    self._target_size_px = None
            return

        try:
            mx, my = self._hand_det.findPoint(LANDMARK_INDEX_FINGER_MCP)
        except Exception:
            mx, my = 0, 0

        fingers = self._hand_det.fingersUp()
        if fingers is None:
            fingers = [0, 0, 0, 0, 0]
        fc = int(sum(1 if int(v) != 0 else 0 for v in fingers))

        hand_size_px = None
        if self._hand_det.cyList:
            hand_size_px = float(max(self._hand_det.cyList) - min(self._hand_det.cyList))

        # palm bounding box
        x_vals = [pt[1] for pt in self._hand_det.lmList]
        y_vals = [pt[2] for pt in self._hand_det.lmList]
        if x_vals and y_vals:
            cv.rectangle(frame, (int(min(x_vals)), int(min(y_vals))),
                         (int(max(x_vals)), int(max(y_vals))), (0, 255, 0), 2)

        # tracking point
        cv.circle(frame, (int(mx), int(my)), 8, (255, 0, 255), -1)
        cv.circle(frame, (int(mx), int(my)), 10, (255, 255, 0), 2)

        with self._track_lock:
            self._has_hand = True
            self._lost_frames = 0
            self._finger_count = fc
            self._target_x = float(mx)
            self._target_y = float(my)
            self._target_size_px = hand_size_px
            self._has_target = fc > 0

    # ── PTZ control (same as hand_tracker) ─────────────────────

    def _reset_ptz_pd(self) -> None:
        self._prev_err_x = 0.0
        self._prev_err_y = 0.0
        self._prev_ctrl_t = 0.0
        self._last_sx = 0.0
        self._last_sy = 0.0

    def _reset_body_pd(self) -> None:
        self._prev_s1 = 0.0
        self._prev_s1_eff = 0.0
        self._prev_body_t = 0.0
        self._last_body_az = 0.0
        self._prev_size_err = 0.0
        self._prev_linear_t = 0.0

    def _reset_pd(self) -> None:
        self._reset_ptz_pd()
        self._reset_body_pd()

    def _ptz_tick(self) -> None:
        if self._state != STATE_TRACKING:
            return
        with self._track_lock:
            tx = self._target_x
            ty = self._target_y
            has_hand = self._has_hand
            fc = self._finger_count

        if not has_hand or fc == 0 or tx is None or ty is None:
            return

        icx = float(self._proc_w) * 0.5
        icy = float(self._proc_h) * 0.5
        pex = float(tx - icx)
        pey = float(ty - icy)

        cx = abs(pex) < self._db_x
        cy = abs(pey) < self._db_y
        if cx:
            pex = 0.0
        if cy:
            pey = 0.0

        n = time.perf_counter()
        dt = n - self._prev_ctrl_t if self._prev_ctrl_t > 0.0 else 1.0 / max(float(self._ptz_rate), MINIMUM_TIMER_RATE_HZ)
        dt = max(dt, 1e-3)

        nex = pex / icx
        ney = pey / icy
        dx = (nex - self._prev_err_x) / dt
        dy = (ney - self._prev_err_y) / dt

        rx = clamp((self._kp * nex + self._kd * dx) * self._max_step, -self._max_step, self._max_step)
        ry = clamp((self._kp * ney + self._kd * dy) * self._max_step, -self._max_step, self._max_step)

        a = self._step_alpha
        sx = (1.0 - a) * self._last_sx + a * rx
        sy = (1.0 - a) * self._last_sy + a * ry
        if cx:
            sx = 0.0; nex = 0.0
        if cy:
            sy = 0.0; ney = 0.0

        self._prev_err_x = nex
        self._prev_err_y = ney
        self._prev_ctrl_t = n
        self._last_sx = sx
        self._last_sy = sy

        if self._rev_h:
            sx = -sx
        if self._rev_p:
            sy = -sy

        dh = clamp(self._h_deg + sx, self._h_min, self._h_max)
        dp = clamp(self._p_deg + sy, self._p_min, self._p_max)

        sa = self._ptz_alpha
        self._h_deg = clamp((1.0 - sa) * self._h_deg + sa * dh, self._h_min, self._h_max)
        self._p_deg = clamp((1.0 - sa) * self._p_deg + sa * dp, self._p_min, self._p_max)

        self._pub_ptz()

    def _pub_ptz(self) -> None:
        msg = Vector3()
        msg.x = float(self._h_deg)
        msg.y = float(self._p_deg)
        msg.z = 0.0
        self._ptz_pub.publish(msg)

    # ── Body control (from gesture_follow) ─────────────────────

    def _rebuild_body_timer(self, log: bool) -> None:
        period = 1.0 / max(float(self._body_rate), MINIMUM_TIMER_RATE_HZ)
        if self._body_timer is not None:
            self.destroy_timer(self._body_timer)
        self._body_timer = self.create_timer(period, self._body_tick)
        if log:
            self.get_logger().info(f"Body timer: {self._body_rate:.1f}Hz")

    def _body_tick(self) -> None:
        if self._state != STATE_TRACKING:
            self._reset_body_pd()
            self._pub_zero_cmd()
            return

        with self._track_lock:
            has_target = self._has_target
            size_px = self._target_size_px

        if not has_target:
            self._reset_body_pd()
            self._pub_zero_cmd()
            return

        s1 = float(self._h_deg)
        if abs(s1) < self._ang_tol_deg:
            s1 = 0.0
        s1_eff = s1 * self._body_share

        n = time.perf_counter()
        dt_body = n - self._prev_body_t if self._prev_body_t > 0.0 else 1.0 / max(float(self._body_rate), MINIMUM_TIMER_RATE_HZ)
        dt_body = max(dt_body, 1e-3)
        ds1_eff = (s1_eff - self._prev_s1_eff) / dt_body

        az = self._body_kp * s1_eff + self._body_kd * ds1_eff
        az = clamp(az, -self._max_angular_z, self._max_angular_z)
        if self._rev_body:
            az = -az

        cmd = Twist()
        cmd.angular.z = float(az)

        linear_x = 0.0
        if self._enable_distance and size_px is not None:
            size_err = float(self._target_hand_size_px - size_px)
            if abs(size_err) < self._distance_tol_px:
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
        self._prev_s1_eff = s1_eff
        self._prev_body_t = n
        self._last_body_az = az

    def _pub_zero_cmd(self) -> None:
        self._cmd_pub.publish(Twist())

    # ── Display ─────────────────────────────────────────────────

    def _ensure_win(self) -> None:
        if self._win_ready:
            return
        cv.namedWindow(self._win_name, cv.WINDOW_NORMAL)
        cv.resizeWindow(self._win_name, self._disp_w, self._disp_h)
        self._win_ready = True

    def _draw_guides(self, frame: np.ndarray) -> None:
        cx = int(frame.shape[1] * 0.5)
        cy = int(frame.shape[0] * 0.5)
        cv.line(frame, (cx, 0), (cx, frame.shape[0]), (255, 255, 0), 1)
        cv.line(frame, (0, cy), (frame.shape[1], cy), (255, 255, 0), 1)
        if self._db_x > 0 and self._db_y > 0:
            cv.rectangle(frame, (cx - int(self._db_x), cy - int(self._db_y)),
                         (cx + int(self._db_x), cy + int(self._db_y)), (80, 80, 80), 1)

    def _draw_overlay(self, frame: np.ndarray) -> None:
        with self._track_lock:
            state = self._state
            has_hand = self._has_hand
            size_px = self._target_size_px

        lines = [
            "State: %s | FPS: %.1f | Hand: %s" % (state, self._measured_fps, "YES" if has_hand else "NO"),
            "PTZ: H=%.1f P=%.1f  step=%.1f" % (self._h_deg, self._p_deg, self._max_step),
            "Body: az=%.2f lx=%.2f  size=%.0f" % (self._last_body_az, 0.0, size_px if size_px else 0.0),
            "Keys: SPACE toggle, i idle, q quit",
        ]
        if self._bg:
            cv.rectangle(frame, (0, 0), (min(frame.shape[1], 640), self._line_h * len(lines) + 8), (0, 0, 0), -1)
        for i, text in enumerate(lines):
            cv.putText(frame, text, (6, 15 + i * self._line_h),
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
            should_recreate_ptz = False
            should_recreate_img = False
            should_recreate_body = False
            for p in params:
                if p.name in RESTART_REQUIRED_PARAMETER_NAMES:
                    return SetParametersResult(successful=False, reason=f"'{p.name}' requires restart.")
                if p.name not in DYNAMIC_PARAMETER_NAMES:
                    continue
                if p.name == "ptz_control_rate":
                    should_recreate_ptz = True
                if p.name == "image_processing_rate_hz":
                    should_recreate_img = True
                if p.name == "body_control_rate":
                    should_recreate_body = True
                if p.name == "show_debug_window":
                    if not bool(p.value) and self._win_ready:
                        cv.destroyWindow(self._win_name)
                        self._win_ready = False
            self._apply_param_values(params)
            if should_recreate_ptz:
                self.destroy_timer(self._ptz_timer)
                self._ptz_timer = self.create_timer(1.0 / max(float(self._ptz_rate), MINIMUM_TIMER_RATE_HZ), self._ptz_tick)
            if should_recreate_img:
                self.destroy_timer(self._img_timer)
                self._img_timer = self.create_timer(1.0 / max(float(self._img_rate), MINIMUM_TIMER_RATE_HZ), self._process_image)
            if should_recreate_body:
                self._rebuild_body_timer(True)
            return SetParametersResult(successful=True)
        except Exception as e:
            return SetParametersResult(successful=False, reason=str(e))

    def _apply_param_values(self, params: List[Parameter]) -> None:
        for p in params:
            n, v = p.name, p.value
            if n in ("show_debug_window", "enable_horizontal_flip", "enable_vertical_flip",
                     "reverse_horizontal_direction", "reverse_pitch_direction",
                     "reverse_body_direction", "reverse_linear_direction",
                     "enable_distance_follow", "enable_rqt_dynamic_parameters"):
                setattr(self, "_" + n.replace("enable_", "").replace("reverse_", "rev_").replace("show_debug_window", "show_win"), bool(v))
            elif n in ("processing_width", "processing_height", "display_width", "display_height",
                       "overlay_line_height", "lost_frame_threshold"):
                setattr(self, "_" + n.replace("processing_", "proc_").replace("display_", "disp_")
                        .replace("overlay_line_height", "line_h").replace("lost_frame_threshold", "lost_thr"), int(v))
            elif n in ("horizontal_min_deg", "horizontal_max_deg", "pitch_min_deg", "pitch_max_deg",
                       "deadband_x", "deadband_y", "pd_kp", "pd_kd",
                       "ptz_step_smooth_alpha", "max_ptz_step", "ptz_smooth_alpha",
                       "ptz_control_rate", "image_processing_rate_hz", "overlay_font_scale",
                       "body_control_rate", "angular_kp", "angular_kd", "body_share", "body_ff_gain",
                       "angular_tolerance_deg", "max_angular_z", "target_hand_size_px",
                       "distance_tolerance_hand_size_px", "distance_kp", "distance_kd", "max_linear_x"):
                setattr(self, "_" + n.replace("horizontal_", "h_").replace("pitch_", "p_").
                        replace("ptz_control_rate", "ptz_rate").replace("image_processing_rate_hz", "img_rate").
                        replace("deadband_", "db_").replace("overlay_font_scale", "font_scale").
                        replace("pd_", "").replace("ptz_step_smooth_alpha", "step_alpha").
                        replace("max_ptz_step", "max_step").replace("ptz_smooth_alpha", "ptz_alpha").
                        replace("body_control_rate", "body_rate").replace("angular_", "body_").
                        replace("angular_tolerance_deg", "ang_tol_deg").replace("max_angular_z", "max_angular_z").
                        replace("target_hand_size_px", "target_hand_size_px").
                        replace("distance_tolerance_hand_size_px", "distance_tol_px").
                        replace("distance_", "distance_").replace("max_linear_x", "max_linear_x"), float(v))
            elif n == "rotate_image_code":
                self._rot = str(v)
            elif n in ("window_name",):
                setattr(self, "_win_name", str(v))
            elif n == "image_topic":
                self._img_topic = str(v)
            elif n == "ptz_command_topic":
                self._ptz_topic = str(v)
            elif n == "cmd_vel_topic":
                self._cmd_topic = str(v)
            elif n == "overlay_show_background":
                self._bg = bool(v)

    def destroy_node(self) -> bool:
        self._shutdown = True
        self._pub_zero_cmd()
        if self._show_win:
            cv.destroyAllWindows()
        try:
            self._hand_det.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = HandFollowPtzNode()
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
