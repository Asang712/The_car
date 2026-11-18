#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gesture_follow_ptz_node.py

中文说明:
    ROS2 Humble 手势跟随节点（云台 + 车身）。
    保留 gesture_tracker_ptz_node 的 MediaPipe 手部识别与图像处理节奏，
    在此基础上补齐 follow 系列的车身旋转与距离跟随控制。

    识别内核:
    - 使用 HandDetector
    - 以食指根部关节（LM5）作为居中目标点
    - 仅在检测到手且非拳头（finger_count > 0）时视作有效跟随目标

    距离跟随:
    - 使用手部包围框高度作为视觉尺寸代理量 target_hand_size_px
"""

from __future__ import annotations

import os
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


DEFAULT_NODE_NAME = "gesture_follow_ptz_node"
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
    "overlay_show_background", "overlay_show_legend",
    "enable_horizontal_flip", "enable_vertical_flip", "rotate_image_code",
    "horizontal_min_deg", "horizontal_max_deg",
    "pitch_min_deg", "pitch_max_deg",
    "initial_horizontal_deg", "initial_pitch_deg",
    "reverse_horizontal_direction", "reverse_pitch_direction",
    "deadband_x", "deadband_y",
    "pd_kp", "pd_kd", "ptz_step_smooth_alpha", "max_ptz_step",
    "ptz_smooth_alpha", "ptz_control_rate", "image_processing_rate_hz",
    "body_control_rate", "angular_kp", "angular_kd", "angular_tolerance_deg", "max_angular_z",
    "body_share", "body_ff_gain", "lost_frame_threshold", "reverse_body_direction",
    "enable_distance_follow", "target_hand_size_px", "distance_tolerance_hand_size_px",
    "distance_kp", "distance_kd", "max_linear_x", "reverse_linear_direction",
    "publish_initial_ptz_command", "initial_publish_count", "initial_publish_interval_sec",
    "hand_detection_confidence",
    "print_ptz_angle", "print_interval_sec",
    "enable_rqt_dynamic_parameters",
})

RESTART_REQUIRED_PARAMETER_NAMES = frozenset({
    "image_topic", "ptz_command_topic", "cmd_vel_topic", "window_name",
})


def clamp(value: float, lo: float, hi: float) -> float:
    return max(float(lo), min(float(hi), float(value)))


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


class GestureFollowPtzNode(Node):

    def __init__(self) -> None:
        super().__init__(DEFAULT_NODE_NAME)
        self._image_lock = threading.Lock()
        self._gesture_lock = threading.Lock()

        self._latest_frame: Optional[np.ndarray] = None
        self._latest_seq = 0
        self._processed_seq = 0

        self._state = STATE_IDLE
        self._shutdown = False

        self._has_hand = False
        self._has_target = False
        self._lost_frames = 0
        self._finger_count = 0
        self._gesture_name = "None"
        self._finger_code = "[0,0,0,0,0]"
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
        self._last_print_t = 0.0

        self._declare_params()
        self._load_params()

        self._h_deg = float(self._init_h_deg)
        self._p_deg = float(self._init_p_deg)

        self._hand_det = HandDetector(detectorCon=float(self._hand_conf))

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

        self._init_ptz_timer = None
        self._init_ptz_remain = 0
        if self._pub_init_ptz:
            self._start_init_ptz()

        self._param_cb = self.add_on_set_parameters_callback(self._on_param_update)

        self.get_logger().info(
            f"Gesture follow PTZ+body started. SPACE=toggle, i=idle, q=quit. "
            f"ptz={self._ptz_rate:.0f}Hz img={self._img_rate:.0f}Hz body={self._body_rate:.0f}Hz"
        )

    def _declare_params(self) -> None:
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("ptz_command_topic", "/cmd_ptz_angle")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("window_name", "gesture_follow_ptz")
        self.declare_parameter("show_debug_window", True)
        self.declare_parameter("processing_width", 320)
        self.declare_parameter("processing_height", 240)
        self.declare_parameter("display_width", 960)
        self.declare_parameter("display_height", 720)
        self.declare_parameter("overlay_font_scale", 0.32)
        self.declare_parameter("overlay_line_height", 13)
        self.declare_parameter("overlay_show_background", False)
        self.declare_parameter("overlay_show_legend", True)
        self.declare_parameter("enable_horizontal_flip", False)
        self.declare_parameter("enable_vertical_flip", False)
        self.declare_parameter("rotate_image_code", "none")
        self.declare_parameter("horizontal_min_deg", -90.0)
        self.declare_parameter("horizontal_max_deg", 90.0)
        self.declare_parameter("pitch_min_deg", -90.0)
        self.declare_parameter("pitch_max_deg", 20.0)
        self.declare_parameter("initial_horizontal_deg", 0.0)
        self.declare_parameter("initial_pitch_deg", -45.0)
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
        self.declare_parameter("publish_initial_ptz_command", True)
        self.declare_parameter("initial_publish_count", 20)
        self.declare_parameter("initial_publish_interval_sec", 0.2)
        self.declare_parameter("hand_detection_confidence", 0.8)
        self.declare_parameter("print_ptz_angle", False)
        self.declare_parameter("print_interval_sec", 0.5)
        self.declare_parameter("enable_rqt_dynamic_parameters", True)

    def _load_params(self) -> None:
        self._img_topic = str(self.get_parameter("image_topic").value)
        self._ptz_topic = str(self.get_parameter("ptz_command_topic").value)
        self._cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._win_name = str(self.get_parameter("window_name").value)
        self._show_win = to_bool(self.get_parameter("show_debug_window").value, True)
        self._proc_w = int(self.get_parameter("processing_width").value)
        self._proc_h = int(self.get_parameter("processing_height").value)
        self._disp_w = int(self.get_parameter("display_width").value)
        self._disp_h = int(self.get_parameter("display_height").value)
        self._font_scale = float(self.get_parameter("overlay_font_scale").value)
        self._line_h = int(self.get_parameter("overlay_line_height").value)
        self._show_bg = to_bool(self.get_parameter("overlay_show_background").value, False)
        self._show_legend = to_bool(self.get_parameter("overlay_show_legend").value, True)
        self._flip_h = to_bool(self.get_parameter("enable_horizontal_flip").value, False)
        self._flip_v = to_bool(self.get_parameter("enable_vertical_flip").value, False)
        self._rot = str(self.get_parameter("rotate_image_code").value).lower()
        self._h_min = float(self.get_parameter("horizontal_min_deg").value)
        self._h_max = float(self.get_parameter("horizontal_max_deg").value)
        self._p_min = float(self.get_parameter("pitch_min_deg").value)
        self._p_max = float(self.get_parameter("pitch_max_deg").value)
        self._init_h_deg = float(self.get_parameter("initial_horizontal_deg").value)
        self._init_p_deg = float(self.get_parameter("initial_pitch_deg").value)
        self._rev_h = to_bool(self.get_parameter("reverse_horizontal_direction").value, False)
        self._rev_p = to_bool(self.get_parameter("reverse_pitch_direction").value, True)
        self._db_x = float(self.get_parameter("deadband_x").value)
        self._db_y = float(self.get_parameter("deadband_y").value)
        self._kp = max(float(self.get_parameter("pd_kp").value), 0.0)
        self._kd = max(float(self.get_parameter("pd_kd").value), 0.0)
        self._step_alpha = clamp(float(self.get_parameter("ptz_step_smooth_alpha").value), 0.0, 1.0)
        self._max_step = float(self.get_parameter("max_ptz_step").value)
        self._ptz_alpha = clamp(float(self.get_parameter("ptz_smooth_alpha").value), 0.01, 1.0)
        self._ptz_rate = float(self.get_parameter("ptz_control_rate").value)
        self._img_rate = float(self.get_parameter("image_processing_rate_hz").value)
        self._body_rate = float(self.get_parameter("body_control_rate").value)
        self._body_kp = max(float(self.get_parameter("angular_kp").value), 0.0)
        self._body_kd = max(float(self.get_parameter("angular_kd").value), 0.0)
        self._body_share = float(self.get_parameter("body_share").value)
        self._body_ff_gain = float(self.get_parameter("body_ff_gain").value)
        self._lost_frame_threshold = max(1, int(self.get_parameter("lost_frame_threshold").value))
        self._ang_tol_deg = max(float(self.get_parameter("angular_tolerance_deg").value), 0.0)
        self._max_angular_z = max(float(self.get_parameter("max_angular_z").value), 0.0)
        self._rev_body = to_bool(self.get_parameter("reverse_body_direction").value, True)
        self._enable_distance = to_bool(self.get_parameter("enable_distance_follow").value, False)
        self._target_hand_size_px = float(self.get_parameter("target_hand_size_px").value)
        self._distance_tol_px = max(float(self.get_parameter("distance_tolerance_hand_size_px").value), 0.0)
        self._distance_kp = max(float(self.get_parameter("distance_kp").value), 0.0)
        self._distance_kd = max(float(self.get_parameter("distance_kd").value), 0.0)
        self._max_linear_x = max(float(self.get_parameter("max_linear_x").value), 0.0)
        self._rev_linear = to_bool(self.get_parameter("reverse_linear_direction").value, False)
        self._pub_init_ptz = to_bool(self.get_parameter("publish_initial_ptz_command").value, True)
        self._init_ptz_n = int(self.get_parameter("initial_publish_count").value)
        self._init_ptz_int = float(self.get_parameter("initial_publish_interval_sec").value)
        self._hand_conf = float(self.get_parameter("hand_detection_confidence").value)
        self._print_ptz = to_bool(self.get_parameter("print_ptz_angle").value, False)
        self._print_int = float(self.get_parameter("print_interval_sec").value)
        self._rqt = to_bool(self.get_parameter("enable_rqt_dynamic_parameters").value, True)

    def _on_image(self, msg: Image) -> None:
        try:
            frame = image_to_bgr(msg)
            if frame is None:
                return
            with self._image_lock:
                self._latest_frame = frame
                self._latest_seq += 1
        except Exception:
            pass

    def _process_image(self) -> None:
        with self._image_lock:
            if self._latest_frame is None:
                if self._show_win:
                    self._poll_keys()
                return
            seq = self._latest_seq

        if seq == self._processed_seq:
            if self._show_win:
                self._poll_keys()
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
        self._poll_keys()

    def _poll_keys(self) -> None:
        k = cv.waitKey(1) & 0xFF
        if k == 255:
            return
        if k in (ord("q"), ord("Q"), KEY_CODE_ESCAPE):
            self._shutdown = True
            os._exit(0)  # pylint: disable=protected-access
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
            self._hand_det.findHands(frame, draw=True)
        except TypeError:
            self._hand_det.findHands(frame)

        if len(self._hand_det.lmList) == 0:
            with self._gesture_lock:
                self._has_hand = False
                self._lost_frames += 1
                if self._lost_frames >= self._lost_frame_threshold:
                    self._has_target = False
                    self._target_x = None
                    self._target_y = None
                    self._target_size_px = None
                    self._finger_count = 0
                    self._gesture_name = "None"
                    self._finger_code = "[0,0,0,0,0]"
            return

        try:
            mx, my = self._hand_det.findPoint(LANDMARK_INDEX_FINGER_MCP)
        except Exception:
            mx, my = 0, 0

        fingers = self._hand_det.fingersUp()
        if fingers is None:
            fingers = [0, 0, 0, 0, 0]
        nf = [1 if int(v) != 0 else 0 for v in fingers]
        fc = int(sum(nf))
        fcode = "[" + ",".join(str(v) for v in nf) + "]"

        try:
            gn = str(self._hand_det.get_gesture())
        except Exception:
            gn = "Unknown"

        hand_size_px = None
        if self._hand_det.cyList:
            hand_size_px = float(max(self._hand_det.cyList) - min(self._hand_det.cyList))

        cv.circle(frame, (int(mx), int(my)), 8, (255, 0, 255), -1)
        cv.circle(frame, (int(mx), int(my)), 10, (255, 255, 0), 2)

        with self._gesture_lock:
            self._has_hand = True
            self._lost_frames = 0
            self._target_x = float(mx)
            self._target_y = float(my)
            self._target_size_px = hand_size_px
            self._finger_count = fc
            self._finger_code = fcode
            self._gesture_name = gn
            self._has_target = fc > 0 and hand_size_px is not None and hand_size_px > 0.0

    def _start_init_ptz(self) -> None:
        self._init_ptz_remain = max(0, int(self._init_ptz_n))
        if self._init_ptz_remain <= 0:
            return
        self._pub_ptz()
        self._init_ptz_remain -= 1
        if self._init_ptz_remain <= 0:
            return
        self._init_ptz_timer = self.create_timer(
            max(float(self._init_ptz_int), 0.02), self._init_ptz_tick
        )

    def _init_ptz_tick(self) -> None:
        if self._init_ptz_remain <= 0:
            if self._init_ptz_timer is not None:
                self.destroy_timer(self._init_ptz_timer)
                self._init_ptz_timer = None
            return
        self._pub_ptz()
        self._init_ptz_remain -= 1

    def _pub_ptz(self) -> None:
        m = Vector3()
        m.x = float(self._h_deg)
        m.y = float(self._p_deg)
        m.z = 0.0
        self._ptz_pub.publish(m)

    def _pub_zero_cmd(self) -> None:
        self._cmd_pub.publish(Twist())

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
        with self._gesture_lock:
            tx = self._target_x
            ty = self._target_y
            has_target = self._has_target
            fc = self._finger_count

        if not has_target or fc == 0 or tx is None or ty is None:
            self._reset_ptz_pd()
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

        ff = self._body_ff_gain * self._last_body_az
        rx = clamp((self._kp * nex + self._kd * dx - ff) * self._max_step, -self._max_step, self._max_step)
        ry = clamp((self._kp * ney + self._kd * dy) * self._max_step, -self._max_step, self._max_step)

        a = self._step_alpha
        sx = (1.0 - a) * self._last_sx + a * rx
        sy = (1.0 - a) * self._last_sy + a * ry
        if cx:
            sx = 0.0
            nex = 0.0
        if cy:
            sy = 0.0
            ney = 0.0

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
        self._log_ctrl(pex, pey)

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

        with self._gesture_lock:
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

    def _log_ctrl(self, he: float, ve: float) -> None:
        if not self._print_ptz:
            return
        n = time.monotonic()
        if n - self._last_print_t >= max(float(self._print_int), 0.01):
            self._last_print_t = n
            self.get_logger().info(
                f"PTZ err=({he:.1f},{ve:.1f}) s1={self._h_deg:.1f} s2={self._p_deg:.1f}"
            )

    def _ensure_win(self) -> None:
        if self._win_ready:
            return
        cv.namedWindow(self._win_name, cv.WINDOW_NORMAL)
        cv.resizeWindow(self._win_name, int(self._disp_w), int(self._disp_h))
        self._win_ready = True

    def _draw_guides(self, frame: np.ndarray) -> None:
        cx, cy = int(self._proc_w * 0.5), int(self._proc_h * 0.5)
        cv.line(frame, (cx, 0), (cx, self._proc_h), (255, 255, 0), 1)
        cv.line(frame, (0, cy), (self._proc_w, cy), (255, 255, 0), 1)
        cv.rectangle(
            frame,
            (int(cx - self._db_x), int(cy - self._db_y)),
            (int(cx + self._db_x), int(cy + self._db_y)),
            (0, 180, 255),
            1,
        )

    def _draw_overlay(self, frame: np.ndarray) -> None:
        with self._gesture_lock:
            tx, ty = self._target_x, self._target_y
            size_px = self._target_size_px
            fc = self._finger_count
            fcode = self._finger_code
            gn = self._gesture_name
            has_hand = self._has_hand
            has_target = self._has_target
            lost = self._lost_frames

        cx = self._proc_w * 0.5
        cy = self._proc_h * 0.5
        he = (tx - cx) if has_hand and tx is not None else 0.0
        ve = (ty - cy) if has_hand and ty is not None else 0.0
        size_err = (self._target_hand_size_px - size_px) if size_px is not None else 0.0
        lines = [
            f"State:{self._state} FPS:{int(self._measured_fps)} target:{'yes' if has_target else 'no'} lost:{lost}",
            f"Gesture:{gn} Finger:{fcode} Count:{fc} {'FIST' if fc == 0 else 'TRACK'}",
            f"Target:(LM5) Error H:{he:.1f} V:{ve:.1f}",
            f"HandSize cur:{size_px if size_px is not None else 0.0:.1f} target:{self._target_hand_size_px:.1f} err:{size_err:.1f}",
            f"PTZ kp:{self._kp:.2f} kd:{self._kd:.2f} dead:({self._db_x:.0f},{self._db_y:.0f}) step:{self._max_step:.1f}",
            f"PTZ s1:{self._h_deg:.1f} s2:{self._p_deg:.1f} body_share:{self._body_share:.2f}",
            f"Body kp:{self._body_kp:.2f} kd:{self._body_kd:.2f} max_az:{self._max_angular_z:.2f} dist:{'on' if self._enable_distance else 'off'}",
        ]
        if self._show_legend:
            lines.append("SPACE:toggle | i:idle | q:quit")
        if self._show_bg:
            bh = self._line_h * len(lines) + 8
            cv.rectangle(frame, (0, 0), (self._proc_w, min(self._proc_h, bh)), (0, 0, 0), -1)
        for i, text in enumerate(lines):
            cv.putText(
                frame, text, (6, 15 + i * self._line_h), cv.FONT_HERSHEY_SIMPLEX,
                self._font_scale, (0, 255, 255), 1, cv.LINE_AA
            )

    def _update_fps(self) -> None:
        self._fps_count += 1
        n = time.perf_counter()
        elapsed = n - self._fps_last_t
        if elapsed >= 1.0:
            self._measured_fps = self._fps_count / elapsed
            self._fps_count = 0
            self._fps_last_t = n

    def _on_param_update(self, params: List[Parameter]) -> SetParametersResult:
        if not params:
            return SetParametersResult(successful=True)
        names = {p.name for p in params}
        bad = names - DYNAMIC_PARAMETER_NAMES - RESTART_REQUIRED_PARAMETER_NAMES
        if bad:
            return SetParametersResult(successful=False, reason=f"Unsupported: {sorted(bad)}")
        if names & RESTART_REQUIRED_PARAMETER_NAMES:
            return SetParametersResult(
                successful=False, reason=f"Restart required: {sorted(names & RESTART_REQUIRED_PARAMETER_NAMES)}"
            )
        if not self._rqt and names != {"enable_rqt_dynamic_parameters"}:
            return SetParametersResult(successful=False, reason="Disabled")

        old_ptz = self._ptz_rate
        old_img = self._img_rate
        old_body = self._body_rate
        old_debug = self._show_win
        old_size = (self._disp_w, self._disp_h)

        vals = self._merge_vals(params)
        err = self._check_vals(vals)
        if err:
            return SetParametersResult(successful=False, reason=err)
        self._apply_vals(vals)

        if abs(self._ptz_rate - old_ptz) > 1e-9:
            self.destroy_timer(self._ptz_timer)
            self._ptz_timer = self.create_timer(
                1.0 / max(float(self._ptz_rate), MINIMUM_TIMER_RATE_HZ), self._ptz_tick
            )
        if abs(self._img_rate - old_img) > 1e-9:
            self.destroy_timer(self._img_timer)
            self._img_timer = self.create_timer(
                1.0 / max(float(self._img_rate), MINIMUM_TIMER_RATE_HZ), self._process_image
            )
        if abs(self._body_rate - old_body) > 1e-9:
            self._rebuild_body_timer(True)
        self._sync_win(old_debug, old_size)
        return SetParametersResult(successful=True)

    def _merge_vals(self, params: List[Parameter]) -> Dict[str, Any]:
        d: Dict[str, Any] = {
            "show_debug_window": self._show_win,
            "processing_width": self._proc_w, "processing_height": self._proc_h,
            "display_width": self._disp_w, "display_height": self._disp_h,
            "overlay_font_scale": self._font_scale, "overlay_line_height": self._line_h,
            "overlay_show_background": self._show_bg, "overlay_show_legend": self._show_legend,
            "enable_horizontal_flip": self._flip_h, "enable_vertical_flip": self._flip_v,
            "rotate_image_code": self._rot,
            "horizontal_min_deg": self._h_min, "horizontal_max_deg": self._h_max,
            "pitch_min_deg": self._p_min, "pitch_max_deg": self._p_max,
            "reverse_horizontal_direction": self._rev_h, "reverse_pitch_direction": self._rev_p,
            "deadband_x": self._db_x, "deadband_y": self._db_y,
            "pd_kp": self._kp, "pd_kd": self._kd,
            "ptz_step_smooth_alpha": self._step_alpha, "max_ptz_step": self._max_step,
            "ptz_smooth_alpha": self._ptz_alpha, "ptz_control_rate": self._ptz_rate,
            "image_processing_rate_hz": self._img_rate, "body_control_rate": self._body_rate,
            "angular_kp": self._body_kp, "angular_kd": self._body_kd,
            "angular_tolerance_deg": self._ang_tol_deg, "max_angular_z": self._max_angular_z,
            "body_share": self._body_share, "body_ff_gain": self._body_ff_gain,
            "lost_frame_threshold": self._lost_frame_threshold, "reverse_body_direction": self._rev_body,
            "enable_distance_follow": self._enable_distance,
            "target_hand_size_px": self._target_hand_size_px,
            "distance_tolerance_hand_size_px": self._distance_tol_px,
            "distance_kp": self._distance_kp, "distance_kd": self._distance_kd,
            "max_linear_x": self._max_linear_x, "reverse_linear_direction": self._rev_linear,
            "hand_detection_confidence": self._hand_conf,
            "print_ptz_angle": self._print_ptz, "print_interval_sec": self._print_int,
            "enable_rqt_dynamic_parameters": self._rqt,
        }
        for p in params:
            if p.type_ != Parameter.Type.NOT_SET:
                d[p.name] = p.value
        return d

    def _check_vals(self, v: Dict[str, Any]) -> Optional[str]:
        try:
            if int(v["processing_width"]) <= 0 or int(v["processing_height"]) <= 0:
                return "processing size > 0"
            if int(v["display_width"]) <= 0 or int(v["display_height"]) <= 0:
                return "display size > 0"
            if float(v["deadband_x"]) < 0 or float(v["deadband_y"]) < 0:
                return "deadband >= 0"
            if float(v["ptz_control_rate"]) < 1.0 or float(v["image_processing_rate_hz"]) < 1.0 or float(v["body_control_rate"]) < 1.0:
                return "rates >= 1.0"
            if int(v["lost_frame_threshold"]) < 1:
                return "lost_frame_threshold >= 1"
            if float(v["target_hand_size_px"]) <= 0 or float(v["distance_tolerance_hand_size_px"]) < 0:
                return "distance params invalid"
            if float(v["max_linear_x"]) < 0 or float(v["max_angular_z"]) < 0:
                return "max speeds >= 0"
            if float(v["hand_detection_confidence"]) < 0.0 or float(v["hand_detection_confidence"]) > 1.0:
                return "hand_detection_confidence in [0,1]"
            return None
        except Exception as exc:
            return str(exc)

    def _apply_vals(self, v: Dict[str, Any]) -> None:
        self._show_win = to_bool(v["show_debug_window"], self._show_win)
        self._proc_w = int(v["processing_width"]); self._proc_h = int(v["processing_height"])
        self._disp_w = int(v["display_width"]); self._disp_h = int(v["display_height"])
        self._font_scale = float(v["overlay_font_scale"]); self._line_h = int(v["overlay_line_height"])
        self._show_bg = to_bool(v["overlay_show_background"], self._show_bg)
        self._show_legend = to_bool(v["overlay_show_legend"], self._show_legend)
        self._flip_h = to_bool(v["enable_horizontal_flip"], self._flip_h)
        self._flip_v = to_bool(v["enable_vertical_flip"], self._flip_v)
        self._rot = str(v["rotate_image_code"]).lower()
        self._h_min = float(v["horizontal_min_deg"]); self._h_max = float(v["horizontal_max_deg"])
        self._p_min = float(v["pitch_min_deg"]); self._p_max = float(v["pitch_max_deg"])
        self._rev_h = to_bool(v["reverse_horizontal_direction"], self._rev_h)
        self._rev_p = to_bool(v["reverse_pitch_direction"], self._rev_p)
        self._db_x = float(v["deadband_x"]); self._db_y = float(v["deadband_y"])
        self._kp = max(float(v["pd_kp"]), 0.0); self._kd = max(float(v["pd_kd"]), 0.0)
        self._step_alpha = clamp(float(v["ptz_step_smooth_alpha"]), 0.0, 1.0)
        self._max_step = float(v["max_ptz_step"])
        self._ptz_alpha = clamp(float(v["ptz_smooth_alpha"]), 0.01, 1.0)
        self._ptz_rate = float(v["ptz_control_rate"])
        self._img_rate = float(v["image_processing_rate_hz"])
        self._body_rate = float(v["body_control_rate"])
        self._body_kp = max(float(v["angular_kp"]), 0.0)
        self._body_kd = max(float(v["angular_kd"]), 0.0)
        self._ang_tol_deg = max(float(v["angular_tolerance_deg"]), 0.0)
        self._max_angular_z = max(float(v["max_angular_z"]), 0.0)
        self._body_share = float(v["body_share"])
        self._body_ff_gain = float(v["body_ff_gain"])
        self._lost_frame_threshold = max(1, int(v["lost_frame_threshold"]))
        self._rev_body = to_bool(v["reverse_body_direction"], self._rev_body)
        self._enable_distance = to_bool(v["enable_distance_follow"], self._enable_distance)
        self._target_hand_size_px = float(v["target_hand_size_px"])
        self._distance_tol_px = max(float(v["distance_tolerance_hand_size_px"]), 0.0)
        self._distance_kp = max(float(v["distance_kp"]), 0.0)
        self._distance_kd = max(float(v["distance_kd"]), 0.0)
        self._max_linear_x = max(float(v["max_linear_x"]), 0.0)
        self._rev_linear = to_bool(v["reverse_linear_direction"], self._rev_linear)
        self._hand_conf = float(v["hand_detection_confidence"])
        self._print_ptz = to_bool(v["print_ptz_angle"], self._print_ptz)
        self._print_int = float(v["print_interval_sec"])
        self._rqt = to_bool(v["enable_rqt_dynamic_parameters"], self._rqt)

    def _sync_win(self, old_show: bool, old_size: tuple[int, int]) -> None:
        if self._show_win and not old_show:
            self._ensure_win()
            if self._key_timer is None:
                self._key_timer = self.create_timer(0.05, self._poll_keys)
        elif not self._show_win and old_show:
            if self._key_timer is not None:
                self.destroy_timer(self._key_timer)
                self._key_timer = None
            try:
                cv.destroyWindow(self._win_name)
            except Exception:
                pass
            self._win_ready = False
        elif self._show_win and old_size != (self._disp_w, self._disp_h) and self._win_ready:
            cv.resizeWindow(self._win_name, int(self._disp_w), int(self._disp_h))

    def destroy_node(self) -> bool:
        try:
            self._pub_zero_cmd()
        except Exception:
            pass
        try:
            self._hand_det.close()
        except Exception:
            pass
        try:
            cv.destroyAllWindows()
        except Exception:
            pass
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = GestureFollowPtzNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        finally:
            rclpy.shutdown()
