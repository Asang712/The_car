#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
object_follow_ptz_node.py

中文说明:
    ROS2 Humble 人脸跟随节点（云台 + 车身）。
    参照 colorFollow.py：HSV 颜色检测 → PTZ PD 锁定目标到画面中心 →
    车身根据 s1 角度旋转归零。云台先行锁定，车身跟随云台归零。

    架构：image_callback 直接 人脸检测 + PTZ 控制定时器 + 车身控制定时器。
    退出按 ptz-tracking-rules SKILL：os._exit(0)。

    交互: SPACE=切换追踪, q/ESC=退出。

运行前准备:
    source ~/workspaces/MicroROS-V2-ROS2-SRC/start_yolo_ros2_env.sh
"""

from __future__ import annotations

import math
import os
import threading
import time
from typing import Any, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, Vector3
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from .image_conversion import image_to_bgr
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

DEFAULT_NODE_NAME = "object_follow_ptz_node"

STATE_IDLE = "idle"
STATE_TRACKING = "tracking"

KEY_CODE_SPACE = 32
KEY_CODE_ESCAPE = 27
MINIMUM_TIMER_RATE_HZ = 1.0
MINIMUM_ALPHA = 0.01
MAXIMUM_ALPHA = 1.0

DYNAMIC_PARAMETER_NAMES = frozenset({
    "show_debug_window", "show_binary_window",
    "image_width", "image_height", "window_width", "window_height",
    "overlay_font_scale", "overlay_line_height",
    "overlay_margin_x", "overlay_margin_y", "overlay_show_background",
    "enable_horizontal_flip", "enable_vertical_flip", "rotate_image_code",
    "horizontal_min_deg", "horizontal_max_deg",
    "pitch_min_deg", "pitch_max_deg",
    "initial_horizontal_deg", "initial_pitch_deg",
    "reverse_horizontal_direction", "reverse_pitch_direction",
    "deadband_x", "deadband_y",
    "pd_kp", "pd_kd", "ptz_step_smooth_alpha", "max_ptz_step",
    "ptz_smooth_alpha", "ptz_control_rate", "body_control_rate",
"tracking_backend", "opencv_tracker_type", "tracker_min_width", "tracker_min_height",
    "angular_kp", "angular_kd", "angular_tolerance_deg", "max_angular_z",
    "body_share", "body_ff_gain", "lost_frame_threshold", "reverse_body_direction",
    "enable_distance_follow", "target_object_width",
    "distance_kp", "distance_kd", "max_linear_x",
    "distance_tolerance_obj", "reverse_linear_direction",
    "publish_initial_ptz_command", "initial_publish_count", "initial_publish_interval_sec",
    "print_ptz_angle", "print_interval_sec",
    "enable_rqt_dynamic_parameters",
})

RESTART_REQUIRED_PARAMETER_NAMES = frozenset({
    "image_topic", "ptz_command_topic", "cmd_vel_topic", "window_name", "tracking_backend", "opencv_tracker_type", "tracker_min_width", "tracker_min_height",
})



def clamp(v: float, lo: float, hi: float) -> float:
    return max(float(lo), min(float(hi), float(v)))


def to_bool(v: Any, default: bool) -> bool:
    if isinstance(v, bool): return v
    if isinstance(v, str):
        n = v.strip().lower()
        if n in ("true", "1", "yes", "y", "on"): return True
        if n in ("false", "0", "no", "n", "off"): return False
    if isinstance(v, (int, float)): return bool(v)
    return default



class ObjectFollowPtzNode(Node):

    def __init__(self) -> None:
        super().__init__(DEFAULT_NODE_NAME)
        self._tgt_lock = threading.Lock()
        self._cv_bridge = CvBridge()
        self._latest_frame: Optional[np.ndarray] = None

        self._state = STATE_IDLE
        self._shutdown = False

        # 目标状态
        self._has_target = False
        self._lost_frames = 0; self._lost_frame_threshold = 5
        self._target_x: Optional[float] = None
        self._target_y: Optional[float] = None
        self._target_r: Optional[float] = None  # 检测到的人脸框宽度

        # PTZ
        self._h_deg = 0.0; self._p_deg = 0.0
        self._prev_err_x = 0.0; self._prev_err_y = 0.0
        self._prev_ctrl_t = 0.0; self._last_sx = 0.0; self._last_sy = 0.0

        # Object tracking
        self._tracker = None; self._track_window = None
        self._camshift_hist = None; self._hsv_lower = None; self._hsv_upper = None
        self._selecting = False; self._sel_start = (0, 0); self._sel_end = (0, 0)

        # 车身 PD 状态 / Body PD state
        self._prev_s1 = 0.0; self._prev_body_t = 0.0; self._body_share = 0.5
        self._prev_s1_eff = 0.0  # 经 body_share 缩放后的 s1，用于 PD 微分项
        self._last_body_az = 0.0; self._body_ff_gain = 1.3  # 角速率前馈

        # 距离跟随状态 / Distance follow state
        self._target_object_w = 80.0
        self._prev_r_err = 0.0; self._prev_linear_t = 0.0

        self._fps = 0.0; self._fps_n = 0; self._fps_t = time.perf_counter()
        self._win_ok = False; self._last_print_t = 0.0

        self._declare_params()
        self._load_params()
        self._h_deg = self._init_h_deg; self._p_deg = self._init_p_deg



        video_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )   
        
        self._sub = self.create_subscription(
            Image, self._img_topic, self._on_image, video_qos,
        )
        self._ptz_pub = self.create_publisher(Vector3, self._ptz_topic, 10)
        self._cmd_pub = self.create_publisher(Twist, self._cmd_topic, 10)

        # 定时器
        self._ptz_timer = self.create_timer(
            1.0 / max(self._ptz_rate, MINIMUM_TIMER_RATE_HZ), self._ptz_tick,
        )
        self._body_timer = None; self._rebuild_body_timer(False)
        self._key_timer = None
        if self._show_win:
            self._init_debug_win()
            self._key_timer = self.create_timer(0.05, self._poll_keys)

        self._init_ptz_timer = None; self._init_ptz_remain = 0
        if self._pub_init: self._start_init_ptz()

        self._param_cb = self.add_on_set_parameters_callback(self._on_param_update)
        self.get_logger().info(f"Object follow PTZ+body started. SPACE=toggle, q=quit. "
                              f"ptz={self._ptz_rate:.0f}Hz body={self._body_rate:.0f}Hz")

    # ── 参数 ─────────────────────────────────────────────────
    def _declare_params(self) -> None:
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("ptz_command_topic", "/cmd_ptz_angle")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("window_name", "object_follow_ptz")
        self.declare_parameter("show_debug_window", True)
        self.declare_parameter("show_binary_window", False)
        self.declare_parameter("image_width", 640); self.declare_parameter("image_height", 480)
        self.declare_parameter("window_width", 640); self.declare_parameter("window_height", 480)
        self.declare_parameter("overlay_font_scale", 0.38); self.declare_parameter("overlay_line_height", 16)
        self.declare_parameter("overlay_margin_x", 8); self.declare_parameter("overlay_margin_y", 16)
        self.declare_parameter("overlay_show_background", True)
        self.declare_parameter("enable_horizontal_flip", False)
        self.declare_parameter("enable_vertical_flip", False)
        self.declare_parameter("rotate_image_code", "none")
        self.declare_parameter("horizontal_min_deg", -90.0); self.declare_parameter("horizontal_max_deg", 90.0)
        self.declare_parameter("pitch_min_deg", -90.0); self.declare_parameter("pitch_max_deg", 20.0)
        self.declare_parameter("initial_horizontal_deg", 0.0); self.declare_parameter("initial_pitch_deg", -45.0)
        self.declare_parameter("reverse_horizontal_direction", False)
        self.declare_parameter("reverse_pitch_direction", True)
        self.declare_parameter("deadband_x", 30.0); self.declare_parameter("deadband_y", 30.0)
        self.declare_parameter("pd_kp", 0.7); self.declare_parameter("pd_kd", 0.25)
        self.declare_parameter("ptz_step_smooth_alpha", 0.35); self.declare_parameter("max_ptz_step", 1.2)
        self.declare_parameter("ptz_smooth_alpha", 0.85)
        self.declare_parameter("ptz_control_rate", 30.0); self.declare_parameter("body_control_rate", 10.0)
        self.declare_parameter("tracking_backend", "opencv"); self.declare_parameter("opencv_tracker_type", "CSRT")
        self.declare_parameter("tracker_min_width", 20); self.declare_parameter("tracker_min_height", 20)
        self.declare_parameter("angular_kp", 0.05); self.declare_parameter("angular_kd", 0.05)
        self.declare_parameter("body_share", 0.5)
        self.declare_parameter("body_ff_gain", 1.3)
        self.declare_parameter("lost_frame_threshold", 5)
        self.declare_parameter("angular_tolerance_deg", 3.0)
        self.declare_parameter("max_angular_z", 0.12); self.declare_parameter("reverse_body_direction", True)
        self.declare_parameter("enable_distance_follow", True)
        self.declare_parameter("target_object_width", 80.0)
        self.declare_parameter("distance_kp", 0.008); self.declare_parameter("distance_kd", 0.003)
        self.declare_parameter("max_linear_x", 0.12); self.declare_parameter("reverse_linear_direction", False)
        self.declare_parameter("distance_tolerance_obj", 12.0)
        self.declare_parameter("publish_initial_ptz_command", True)
        self.declare_parameter("initial_publish_count", 20)
        self.declare_parameter("initial_publish_interval_sec", 0.2)
        self.declare_parameter("print_ptz_angle", False); self.declare_parameter("print_interval_sec", 0.5)
        self.declare_parameter("enable_rqt_dynamic_parameters", True)

    def _load_params(self) -> None:
        self._img_topic = str(self.get_parameter("image_topic").value)
        self._ptz_topic = str(self.get_parameter("ptz_command_topic").value)
        self._cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._win_name = str(self.get_parameter("window_name").value)
        self._show_win = to_bool(self.get_parameter("show_debug_window").value, True)
        self._show_bin = to_bool(self.get_parameter("show_binary_window").value, False)
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
        self._step_a = clamp(float(self.get_parameter("ptz_step_smooth_alpha").value), 0.0, 1.0)
        self._max_s = float(self.get_parameter("max_ptz_step").value)
        self._ptz_a = clamp(float(self.get_parameter("ptz_smooth_alpha").value), 0.01, 1.0)
        self._ptz_rate = float(self.get_parameter("ptz_control_rate").value)
        self._body_rate = float(self.get_parameter("body_control_rate").value)
        self._tracking_backend = str(self.get_parameter("tracking_backend").value)
        self._opencv_tracker_type = str(self.get_parameter("opencv_tracker_type").value).upper()
        if not self._tracking_backend or self._tracking_backend in ("_auto_", "none", "None"):
            self._tracking_backend = "opencv"
        self._tracker_w = int(self.get_parameter("tracker_min_width").value)
        self._tracker_h = int(self.get_parameter("tracker_min_height").value)
        self._ang_kp = float(self.get_parameter("angular_kp").value)
        self._ang_kd = float(self.get_parameter("angular_kd").value)
        self._body_share = clamp(float(self.get_parameter("body_share").value), 0.0, 1.0)
        self._body_ff_gain = clamp(float(self.get_parameter("body_ff_gain").value), 0.0, 2.0)
        self._lost_frame_threshold = max(1, int(self.get_parameter("lost_frame_threshold").value))
        self._ang_tol = float(self.get_parameter("angular_tolerance_deg").value)
        self._max_az = float(self.get_parameter("max_angular_z").value)
        self._rev_body = to_bool(self.get_parameter("reverse_body_direction").value, False)
        self._enable_dist = to_bool(self.get_parameter("enable_distance_follow").value, True)
        self._target_object_w = float(self.get_parameter("target_object_width").value)
        self._dist_kp = max(float(self.get_parameter("distance_kp").value), 0.0)
        self._dist_kd = max(float(self.get_parameter("distance_kd").value), 0.0)
        self._max_lx = float(self.get_parameter("max_linear_x").value)
        self._dist_tol = float(self.get_parameter("distance_tolerance_obj").value)
        self._rev_linear = to_bool(self.get_parameter("reverse_linear_direction").value, False)
        self._pub_init = to_bool(self.get_parameter("publish_initial_ptz_command").value, True)
        self._init_n = int(self.get_parameter("initial_publish_count").value)
        self._init_int = float(self.get_parameter("initial_publish_interval_sec").value)
        self._prn = to_bool(self.get_parameter("print_ptz_angle").value, False)
        self._prn_int = float(self.get_parameter("print_interval_sec").value)
        self._rqt = to_bool(self.get_parameter("enable_rqt_dynamic_parameters").value, True)

    # ── 图像回调 ────────────────────────────────────────
    def _on_image(self, msg: Image) -> None:
        try:
            frame = image_to_bgr(msg)
            if frame is None: return
            if self._flip_h: frame = cv2.flip(frame, 1)
            if self._flip_v: frame = cv2.flip(frame, 0)
            if self._rot == "90_clockwise": frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            elif self._rot == "90_counterclockwise": frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
            elif self._rot == "180": frame = cv2.rotate(frame, cv2.ROTATE_180)
            if self._img_w > 0 and self._img_h > 0:
                frame = cv2.resize(frame, (self._img_w, self._img_h))
            self._latest_frame = frame
        except Exception:
            pass

        disp = frame.copy()
        self._track_object(disp)
        if self._selecting:
            cv2.rectangle(disp, self._sel_start, self._sel_end, (0, 255, 0), 2)

        if self._show_win:
            self._draw_overlay(disp)
            cv2.imshow(self._win_name, disp)

        self._fps_n += 1; n = time.perf_counter()
        if n - self._fps_t >= 1.0: self._fps = self._fps_n / (n - self._fps_t); self._fps_n = 0; self._fps_t = n

    def _track_object(self, frame: np.ndarray) -> None:
        if self._tracking_backend == "opencv":
            if self._tracker is None: self._mark_lost(); return
            ok, b = self._tracker.update(frame)
            if not ok: self._tracker = None; self._mark_lost(); return
            x, y, w, h = [int(v) for v in b]
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        elif self._tracking_backend == "color_hsv":
            if self._hsv_lower is None: self._mark_lost(); return
            m = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), self._hsv_lower, self._hsv_upper)
            k = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k); m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)
            cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not cs: self._mark_lost(); return
            x, y, w, h = cv2.boundingRect(max(cs, key=cv2.contourArea))
            if w * h < self._tracker_w * self._tracker_h: self._mark_lost(); return
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        else:
            if self._camshift_hist is None or self._track_window is None: self._mark_lost(); return
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            bp = cv2.calcBackProject([hsv], [0], self._camshift_hist, [0, 180], 1)
            ret, self._track_window = cv2.CamShift(bp, self._track_window,
                (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1))
            pts = cv2.boxPoints(ret); x, y, w, h = cv2.boundingRect(pts)
            if w <= 0 or h <= 0: self._mark_lost(); return
            cv2.drawContours(frame, [np.intp(pts)], 0, (0, 255, 0), 2)
        cx, cy = x + w * 0.5, y + h * 0.5
        cv2.circle(frame, (int(cx), int(cy)), 3, (0, 0, 255), -1)
        with self._tgt_lock:
            self._lost_frames = 0; self._has_target = True
            self._target_x = float(cx); self._target_y = float(cy)
            self._target_r = float(max(w, h))

    def _mark_lost(self) -> None:
        """N 帧容错：连续丢帧达到阈值才判定目标丢失，过滤光照抖动等瞬态丢帧。"""
        with self._tgt_lock:
            self._lost_frames += 1
            if self._lost_frames >= self._lost_frame_threshold:
                self._has_target = False

    # ── PTZ 控制定时器 ──────────────────────────────────────
    def _ptz_tick(self) -> None:
        if self._state != STATE_TRACKING: return
        with self._tgt_lock:
            tx, ty, ok = self._target_x, self._target_y, self._has_target
        if not ok or tx is None or ty is None: return

        icx, icy = self._img_w * 0.5, self._img_h * 0.5
        pex, pey = tx - icx, ty - icy
        cx, cy = abs(pex) < self._db_x, abs(pey) < self._db_y
        if cx: pex = 0.0
        if cy: pey = 0.0

        n = time.perf_counter()
        dt = n - self._prev_ctrl_t if self._prev_ctrl_t > 0.0 else 1.0 / max(self._ptz_rate, MINIMUM_TIMER_RATE_HZ)
        dt = max(dt, 1e-3)
        nex, ney = pex / icx, pey / icy
        dx, dy = (nex - self._prev_err_x) / dt, (ney - self._prev_err_y) / dt
        rx = clamp((self._kp * nex + self._kd * dx) * self._max_s, -self._max_s, self._max_s)
        ry = clamp((self._kp * ney + self._kd * dy) * self._max_s, -self._max_s, self._max_s)

        # 角速率前馈：车身旋转 → PTZ 同步反向补偿，目标不被车身转动带偏。
        # 公式：车身 az(rad/s) / ptz_rate * 180/π = 每 PTZ tick 车身旋转度数，
        # PTZ 反向旋转同等度数抵消。gain 用于微调（默认 1.0）。
        body_rot_deg = self._last_body_az * (180.0 / math.pi) / max(self._ptz_rate, MINIMUM_TIMER_RATE_HZ)
        rx = clamp(rx - body_rot_deg * self._body_ff_gain, -self._max_s, self._max_s)

        sx = (1.0 - self._step_a) * self._last_sx + self._step_a * rx
        sy = (1.0 - self._step_a) * self._last_sy + self._step_a * ry
        if cx: sx = 0.0; nex = 0.0
        if cy: sy = 0.0; ney = 0.0
        self._prev_err_x, self._prev_err_y = nex, ney
        self._prev_ctrl_t, self._last_sx, self._last_sy = n, sx, sy
        if self._rev_h: sx = -sx
        if self._rev_p: sy = -sy
        self._h_deg = clamp((1.0 - self._ptz_a) * self._h_deg + self._ptz_a * clamp(self._h_deg + sx, self._h_min, self._h_max), self._h_min, self._h_max)
        self._p_deg = clamp((1.0 - self._ptz_a) * self._p_deg + self._ptz_a * clamp(self._p_deg + sy, self._p_min, self._p_max), self._p_min, self._p_max)

        m = Vector3(); m.x = float(self._h_deg); m.y = float(self._p_deg); m.z = 0.0
        self._ptz_pub.publish(m)
        self._log_ctrl(pex, pey)

    # ── 车身控制定时器（参照 colorFollow: angular ∝ s1）─────
    def _rebuild_body_timer(self, log: bool) -> None:
        p = 1.0 / max(self._body_rate, MINIMUM_TIMER_RATE_HZ)
        if self._body_timer is not None: self.destroy_timer(self._body_timer)
        self._body_timer = self.create_timer(p, self._body_tick)
        if log: self.get_logger().info(f"Body timer: {self._body_rate:.1f}Hz")

    def _body_tick(self) -> None:
        if self._state != STATE_TRACKING:
            self._prev_s1 = 0.0; self._prev_s1_eff = 0.0; self._prev_body_t = 0.0
            self._pub_zero_cmd()
            return

        # 目标丢失 → 立即冻结车身，防止基于过期 s1 自转。
        with self._tgt_lock:
            ok = self._has_target
        if not ok:
            self._prev_s1 = 0.0; self._prev_s1_eff = 0.0; self._prev_body_t = 0.0
            self._pub_zero_cmd()
            return

        s1 = self._h_deg
        if abs(s1) < self._ang_tol: s1 = 0.0

        # 误差共享分配：车身只承担 body_share 比例的归零任务，
        # 剩余误差由云台 PTZ 追踪自然消化，实现协同控制。
        s1_eff = s1 * self._body_share

        # PD 控制车身角速度：比例项驱动归零，微分项阻尼防过冲。
        n = time.perf_counter()
        dt = n - self._prev_body_t if self._prev_body_t > 0.0 else 1.0 / max(self._body_rate, MINIMUM_TIMER_RATE_HZ)
        dt = max(dt, 1e-3)
        ds1_eff = (s1_eff - self._prev_s1_eff) / dt
        az = self._ang_kp * s1_eff + self._ang_kd * ds1_eff
        self._prev_s1 = s1; self._prev_s1_eff = s1_eff; self._prev_body_t = n

        if self._rev_body: az = -az
        az = clamp(az, -self._max_az, self._max_az)
        self._last_body_az = az  # 供 PTZ 前馈使用

        # ── 线性距离跟随 PD ──
        with self._tgt_lock:
            r = self._target_r
        if self._enable_dist and r is not None and r > 0:
            r_err = self._target_object_w - r       # >0 太远需前进
            if abs(r_err) < self._dist_tol: r_err = 0.0
            dt_l = n - self._prev_linear_t if self._prev_linear_t > 0.0 else 1.0 / max(self._body_rate, MINIMUM_TIMER_RATE_HZ)
            dt_l = max(dt_l, 1e-3)
            dr = (r_err - self._prev_r_err) / dt_l
            lx = self._dist_kp * r_err + self._dist_kd * dr
            self._prev_r_err = r_err; self._prev_linear_t = n
            if self._rev_linear: lx = -lx
            lx = clamp(lx, -self._max_lx, self._max_lx)
        else:
            self._prev_r_err = 0.0; lx = 0.0

        tw = Twist(); tw.angular.z = float(az); tw.linear.x = float(lx)
        self._cmd_pub.publish(tw)

    def _pub_zero_cmd(self) -> None:
        self._cmd_pub.publish(Twist())

    # ── 键盘 + 鼠标 ─────────────────────────────────────────

    def _init_debug_win(self) -> None:
        cv2.namedWindow(self._win_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self._win_name, self._win_w, self._win_h)
        cv2.setMouseCallback(self._win_name, self._on_mouse)

    def _on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self._selecting = True; self._sel_start = (x, y); self._sel_end = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self._selecting:
            self._sel_end = (x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            self._sel_end = (x, y)
            with self._tgt_lock: f = self._latest_frame
            if f is not None: self._init_tracker(f.copy())

    def _init_tracker(self, frame: np.ndarray) -> None:
        x1, y1 = min(self._sel_start[0], self._sel_end[0]), min(self._sel_start[1], self._sel_end[1])
        x2, y2 = max(self._sel_start[0], self._sel_end[0]), max(self._sel_start[1], self._sel_end[1])
        w, h = x2 - x1, y2 - y1
        if w < self._tracker_w or h < self._tracker_h:
            self._selecting = False; self.get_logger().warn(f"ROI too small ({w}x{h})"); return
        roi = (x1, y1, w, h)
        if self._tracking_backend == "opencv":
            self._tracker = self._create_opencv_tracker_by_type(self._opencv_tracker_type)
            if self._tracker is None:
                self.get_logger().error(f"Failed to create tracker: {self._opencv_tracker_type}")
                self._selecting = False; return
            self._tracker.init(frame, roi)
        elif self._tracking_backend == "color_hsv":
            r = frame[y1:y2, x1:x2]; hsv_r = cv2.cvtColor(r, cv2.COLOR_BGR2HSV)
            self._hsv_lower = np.array([max(0,np.min(hsv_r[:,:,0])-5),max(0,np.min(hsv_r[:,:,1])-20),max(0,np.min(hsv_r[:,:,2])-20)], dtype=np.uint8)
            self._hsv_upper = np.array([min(179,np.max(hsv_r[:,:,0])+5),min(255,np.max(hsv_r[:,:,1])+20),min(255,np.max(hsv_r[:,:,2])+20)], dtype=np.uint8)
        else:
            hsv_r = cv2.cvtColor(frame[y1:y2, x1:x2], cv2.COLOR_BGR2HSV)
            hist = cv2.calcHist([hsv_r], [0], None, [180], [0, 180])
            cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX)
            self._camshift_hist = hist; self._track_window = roi
        self._selecting = False
        self.get_logger().info(f"Tracker init: backend={self._tracking_backend}, ROI={w}x{h}")

    def _create_opencv_tracker_by_type(self, tracker_type_name: str):
        tracker_constructors = {
            "CSRT": ["TrackerCSRT_create", "legacy.TrackerCSRT_create"],
            "KCF": ["TrackerKCF_create", "legacy.TrackerKCF_create"],
            "MIL": ["TrackerMIL_create", "legacy.TrackerMIL_create"],
        }
        for constructor_path in tracker_constructors.get(tracker_type_name, []):
            ctor = self._get_nested_cv2_attribute(constructor_path)
            if ctor is not None:
                try: return ctor()
                except Exception as e:
                    self.get_logger().warn(f"Tracker {constructor_path} create failed: {e}")
        return None

    @staticmethod
    def _get_nested_cv2_attribute(attribute_path: str):
        current = cv2
        for name in attribute_path.split('.'):
            if not hasattr(current, name): return None
            current = getattr(current, name)
        return current

    def _poll_keys(self) -> None:
        try: k = cv2.waitKey(1) & 0xFF
        except Exception: return
        if k in (255, -1): return
        if k in (ord("q"), ord("Q"), KEY_CODE_ESCAPE):
            self._shutdown = True
            self._pub_zero_cmd()
            try: self.get_logger().info("Force exiting.")
            except Exception: pass
            os._exit(0)
        elif k == KEY_CODE_SPACE:
            if self._state == STATE_TRACKING:
                self._state = STATE_IDLE; self._reset_pd(); self._pub_zero_cmd()
                self.get_logger().info("IDLE")
            else:
                self._state = STATE_TRACKING; self._reset_pd()
                self.get_logger().info("TRACKING (PTZ+body+face)")

    # ── PTZ 辅助 ────────────────────────────────────────────
    def _start_init_ptz(self) -> None:
        self._init_ptz_remain = max(0, self._init_n)
        if self._init_ptz_remain <= 0: return
        self._pub_ptz(); self._init_ptz_remain -= 1
        if self._init_ptz_remain <= 0: return
        self._init_ptz_timer = self.create_timer(max(self._init_int, 0.02), self._init_ptz_tick)

    def _init_ptz_tick(self) -> None:
        if self._init_ptz_remain <= 0:
            if self._init_ptz_timer is not None: self.destroy_timer(self._init_ptz_timer); self._init_ptz_timer = None
            return
        self._pub_ptz(); self._init_ptz_remain -= 1

    def _pub_ptz(self) -> None:
        m = Vector3(); m.x = float(self._h_deg); m.y = float(self._p_deg); m.z = 0.0
        self._ptz_pub.publish(m)

    def _reset_pd(self) -> None:
        self._prev_err_x = 0.0; self._prev_err_y = 0.0
        self._prev_ctrl_t = 0.0; self._last_sx = 0.0; self._last_sy = 0.0
        self._prev_s1 = 0.0; self._prev_s1_eff = 0.0; self._prev_body_t = 0.0
        self._last_body_az = 0.0; self._lost_frames = 0
        self._target_r = None; self._prev_r_err = 0.0; self._prev_linear_t = 0.0

    def _log_ctrl(self, he, ve):
        if not self._prn: return
        n = time.monotonic()
        if n - self._last_print_t >= max(self._prn_int, 0.01):
            self._last_print_t = n
            self.get_logger().info(f"PTZ err=({he:.1f},{ve:.1f}) s1={self._h_deg:.1f} s2={self._p_deg:.1f}")

    # ── 显示 ────────────────────────────────────────────────
    def _draw_overlay(self, frame: np.ndarray) -> None:
        iw, ih = frame.shape[1], frame.shape[0]; icx, icy = iw // 2, ih // 2
        cv2.drawMarker(frame, (icx, icy), (0, 255, 255), cv2.MARKER_CROSS, 20, 2)
        cv2.rectangle(frame, (icx - int(self._db_x), icy - int(self._db_y)),
                     (icx + int(self._db_x), icy + int(self._db_y)), (0, 180, 255), 2)
        # 文字
        with self._tgt_lock:
            ok = self._has_target; tx, ty = self._target_x, self._target_y
            tr = self._target_r; lf = self._lost_frames
        tgt_txt = f"Obj:{int(tx)},{int(ty)}" if ok and tx else f"Obj:None(lost{lf}/{self._lost_frame_threshold})"
        dist_txt = f"Obj sz:{int(tr)}/{int(self._target_object_w)}px" if tr else "Obj sz:--"
        lines = [
            f"State:{self._state} FPS:{int(self._fps)} {tgt_txt}",
            f"PTZ s1:{self._h_deg:.1f} s2:{self._p_deg:.1f} body:{self._ang_kp:.2f}*s1*{self._body_share:.1f}",
            f"{dist_txt}",
            f"PD kp:{self._kp:.2f} kd:{self._kd:.2f} step:{self._max_s:.1f} FF:{self._body_ff_gain:.1f}",
            "Drag:select | SPACE:toggle | q:quit",
        ]
        if self._ov_bg:
            bh = self._lh * len(lines) + 8
            cv2.rectangle(frame, (0, 0), (iw, min(ih, bh)), (0, 0, 0), -1)
        for i, t in enumerate(lines):
            cv2.putText(frame, t, (self._mx, self._my + i * self._lh), cv2.FONT_HERSHEY_SIMPLEX, self._font, (0, 255, 255), 1, cv2.LINE_AA)

    # ── 动态参数更新 ───────────────────────────────────────
    def _on_param_update(self, params: List[Parameter]) -> SetParametersResult:
        if not params: return SetParametersResult(successful=True)
        names = {p.name for p in params}
        bad = names - DYNAMIC_PARAMETER_NAMES - RESTART_REQUIRED_PARAMETER_NAMES
        if bad: return SetParametersResult(successful=False, reason=f"Unsupported: {sorted(bad)}")
        if names & RESTART_REQUIRED_PARAMETER_NAMES: return SetParametersResult(successful=False, reason="Restart required")
        if not self._rqt and names != {"enable_rqt_dynamic_parameters"}: return SetParametersResult(successful=False, reason="Disabled")

        old_ptz, old_body = self._ptz_rate, self._body_rate
        old_debug, old_size = self._show_win, (self._win_w, self._win_h)
        v = self._merge(params)
        err = self._check(v)
        if err: return SetParametersResult(successful=False, reason=err)
        self._apply(v)

        if abs(self._ptz_rate - old_ptz) > 1e-9:
            self.destroy_timer(self._ptz_timer)
            self._ptz_timer = self.create_timer(1.0 / max(self._ptz_rate, MINIMUM_TIMER_RATE_HZ), self._ptz_tick)
        if abs(self._body_rate - old_body) > 1e-9: self._rebuild_body_timer(True)
        self._sync_win(old_debug, old_size)
        return SetParametersResult(successful=True)

    def _merge(self, params):
        d = dict(show_debug_window=self._show_win, show_binary_window=self._show_bin,
                 image_width=self._img_w, image_height=self._img_h, window_width=self._win_w, window_height=self._win_h,
                 overlay_font_scale=self._font, overlay_line_height=self._lh, overlay_margin_x=self._mx, overlay_margin_y=self._my,
                 overlay_show_background=self._ov_bg, enable_horizontal_flip=self._flip_h, enable_vertical_flip=self._flip_v,
                 rotate_image_code=self._rot, horizontal_min_deg=self._h_min, horizontal_max_deg=self._h_max,
                 pitch_min_deg=self._p_min, pitch_max_deg=self._p_max, reverse_horizontal_direction=self._rev_h,
                 reverse_pitch_direction=self._rev_p, deadband_x=self._db_x, deadband_y=self._db_y,
                 pd_kp=self._kp, pd_kd=self._kd, ptz_step_smooth_alpha=self._step_a, max_ptz_step=self._max_s,
                 ptz_smooth_alpha=self._ptz_a, ptz_control_rate=self._ptz_rate, body_control_rate=self._body_rate,
                 tracking_backend=self._tracking_backend,
                 tracker_min_width=self._tracker_w, tracker_min_height=self._tracker_h,
                 angular_kp=self._ang_kp, angular_kd=self._ang_kd,
                 angular_tolerance_deg=self._ang_tol, max_angular_z=self._max_az,
                 body_share=self._body_share, body_ff_gain=self._body_ff_gain,
                 lost_frame_threshold=self._lost_frame_threshold,
                 reverse_body_direction=self._rev_body,
                 enable_distance_follow=self._enable_dist,
                 target_object_width=self._target_object_w,
                 distance_kp=self._dist_kp, distance_kd=self._dist_kd,
                 max_linear_x=self._max_lx, distance_tolerance_obj=self._dist_tol,
                 reverse_linear_direction=self._rev_linear,
                 print_ptz_angle=self._prn, print_interval_sec=self._prn_int,
                 enable_rqt_dynamic_parameters=self._rqt)
        for p in params:
            if p.type_ != Parameter.Type.NOT_SET: d[p.name] = p.value
        return d

    def _check(self, v):
        try:
            if float(v["angular_kp"]) < 0 or float(v["angular_kd"]) < 0: return "angular_kp/kd >=0"
            if not (0.0 <= float(v["body_share"]) <= 1.0): return "body_share in [0,1]"
            if not (0.0 <= float(v["body_ff_gain"]) <= 2.0): return "body_ff_gain in [0,2]"
            if int(v["lost_frame_threshold"]) < 1: return "lost_frame_threshold >=1"
            if float(v["angular_tolerance_deg"]) < 0: return "angular_tolerance >=0"
            if float(v["max_angular_z"]) <= 0: return "max_angular_z >0"
            if float(v["max_linear_x"]) <= 0: return "max_linear_x >0"
            if float(v["distance_kp"]) < 0 or float(v["distance_kd"]) < 0: return "distance_kp/kd >=0"
            
            if float(v["deadband_x"]) < 0 or float(v["deadband_y"]) < 0: return "db >=0"
            if float(v["pd_kp"]) < 0 or float(v["pd_kd"]) < 0: return "kp/kd >=0"
            if not (0.0 <= float(v["ptz_step_smooth_alpha"]) <= 1.0): return "step_a in [0,1]"
            if float(v["max_ptz_step"]) <= 0: return "max_s >0"
            if float(v["ptz_control_rate"]) < 1.0 or float(v["body_control_rate"]) < 1.0: return "rate >=1"
        except Exception as e: return str(e)
        return ""

    def _apply(self, v):
        self._show_win = to_bool(v["show_debug_window"], True); self._show_bin = to_bool(v["show_binary_window"], False)
        self._img_w = int(v["image_width"]); self._img_h = int(v["image_height"])
        self._win_w = int(v["window_width"]); self._win_h = int(v["window_height"])
        self._font = float(v["overlay_font_scale"]); self._lh = int(v["overlay_line_height"])
        self._mx = int(v["overlay_margin_x"]); self._my = int(v["overlay_margin_y"]); self._ov_bg = to_bool(v["overlay_show_background"], True)
        self._flip_h = to_bool(v["enable_horizontal_flip"], False); self._flip_v = to_bool(v["enable_vertical_flip"], False)
        self._rot = str(v["rotate_image_code"]).lower()
        self._h_min = float(v["horizontal_min_deg"]); self._h_max = float(v["horizontal_max_deg"])
        self._p_min = float(v["pitch_min_deg"]); self._p_max = float(v["pitch_max_deg"])
        self._rev_h = to_bool(v["reverse_horizontal_direction"], False); self._rev_p = to_bool(v["reverse_pitch_direction"], True)
        self._db_x = float(v["deadband_x"]); self._db_y = float(v["deadband_y"])
        self._kp = max(float(v["pd_kp"]), 0.0); self._kd = max(float(v["pd_kd"]), 0.0)
        self._step_a = clamp(float(v["ptz_step_smooth_alpha"]), 0.0, 1.0); self._max_s = float(v["max_ptz_step"])
        self._ptz_a = clamp(float(v["ptz_smooth_alpha"]), 0.01, 1.0)
        self._ptz_rate = float(v["ptz_control_rate"]); self._body_rate = float(v["body_control_rate"])
        self._tracking_backend = str(v.get("tracking_backend","opencv"))
        self._opencv_tracker_type = str(v.get("opencv_tracker_type","CSRT")).upper()
        if not self._tracking_backend or self._tracking_backend in ("_auto_", "none", "None"):
            self._tracking_backend = "opencv"
        self._tracker_w = int(v["tracker_min_width"]); self._tracker_h = int(v["tracker_min_height"])
        self._ang_kp = float(v["angular_kp"]); self._ang_kd = float(v["angular_kd"])
        self._body_share = clamp(float(v["body_share"]), 0.0, 1.0)
        self._body_ff_gain = clamp(float(v["body_ff_gain"]), 0.0, 2.0)
        self._lost_frame_threshold = max(1, int(v["lost_frame_threshold"]))
        self._ang_tol = float(v["angular_tolerance_deg"])
        self._max_az = float(v["max_angular_z"]); self._rev_body = to_bool(v["reverse_body_direction"], False)
        self._enable_dist = to_bool(v["enable_distance_follow"], True)
        self._target_object_w = float(v["target_object_width"])
        self._dist_kp = max(float(v["distance_kp"]), 0.0); self._dist_kd = max(float(v["distance_kd"]), 0.0)
        self._max_lx = float(v["max_linear_x"]); self._dist_tol = float(v["distance_tolerance_obj"])
        self._rev_linear = to_bool(v["reverse_linear_direction"], False)
        if old_cascade != self._tracking_backend:
            self._reload_face_cascade()
        self._prn = to_bool(v["print_ptz_angle"], False); self._rqt = to_bool(v["enable_rqt_dynamic_parameters"], True)
        self._h_deg = clamp(self._h_deg, self._h_min, self._h_max); self._p_deg = clamp(self._p_deg, self._p_min, self._p_max)
        self._reset_pd()

    def _sync_win(self, prev_debug, prev_size):
        try:
            chg = prev_size != (self._win_w, self._win_h)
            if self._show_win and not prev_debug: self._init_win()
            elif not self._show_win and prev_debug: cv2.destroyWindow(self._win_name)
            elif self._show_win and chg: cv2.resizeWindow(self._win_name, self._win_w, self._win_h)
        except Exception: pass

    def destroy_node(self) -> bool:
        self._pub_zero_cmd()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObjectFollowPtzNode()
    try:
        while rclpy.ok() and not node._shutdown:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt: pass
    finally:
        try: node.destroy_node()
        except Exception: pass
        if rclpy.ok():
            try: rclpy.shutdown()
            except Exception: pass


if __name__ == "__main__":
    main()
