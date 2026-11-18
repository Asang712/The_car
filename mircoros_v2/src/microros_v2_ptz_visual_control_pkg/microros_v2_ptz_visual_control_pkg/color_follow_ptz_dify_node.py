#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
color_follow_ptz_dify_node.py

Dify 专用颜色跟随节点 - PTZ 版本
- 无 GUI 窗口，后台运行
- 接受颜色名称参数 (red/green/blue/yellow)
- 从配置文件读取预标定的 HSV 值
- 云台 + 车身协同跟随
- 专为 Dify 工作流优化
"""

from __future__ import annotations

import math
import os
import threading
import time
from typing import Optional, Tuple

import cv2 as cv
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


def clamp(v: float, lo: float, hi: float) -> float:
    return max(float(lo), min(float(hi), float(v)))


def read_hsv_from_file(file_path: str, color_index: int) -> Optional[Tuple[Tuple[int, int, int], Tuple[int, int, int]]]:
    """
    从 HSV 配置文件读取指定颜色的 HSV 范围

    文件格式（每行一个颜色，共4行）：
    H_min, S_min, V_min, H_max, S_max, V_max

    color_index: 0=red, 1=green, 2=blue, 3=yellow
    """
    if not os.path.exists(file_path):
        return None

    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            lines = f.readlines()

        if color_index >= len(lines):
            return None

        line = lines[color_index].strip()
        if not line:
            return None

        vals = [int(v.strip()) for v in line.split(',')]
        if len(vals) != 6:
            return None

        return (vals[0], vals[1], vals[2]), (vals[3], vals[4], vals[5])
    except Exception:
        return None


class ColorFollowPtzDifyNode(Node):
    """
    Dify 专用颜色跟随节点 - PTZ 版本

    特点：
    - 无 GUI，纯后台运行
    - 接受颜色名称参数 (red/green/blue/yellow)
    - 从预标定配置文件读取 HSV 值
    - 云台 PD 锁定目标 + 车身跟随云台归零
    """

    def __init__(self) -> None:
        super().__init__("color_follow_ptz_dify_node")

        self._tgt_lock = threading.Lock()
        self._latest_frame: Optional[np.ndarray] = None
        self._bridge = CvBridge()

        # 目标状态
        self._has_target = False
        self._lost_frames = 0
        self._target_x: Optional[float] = None
        self._target_y: Optional[float] = None
        self._target_r: Optional[float] = None  # 检测到的目标外接圆半径
        self._last_target_seen_at = time.monotonic()
        self._finished_published = False
        self._last_stop_publish_at = 0.0

        # PTZ 状态
        self._h_deg = 0.0
        self._p_deg = 0.0
        self._prev_err_x = 0.0
        self._prev_err_y = 0.0
        self._prev_ctrl_t = 0.0
        self._last_sx = 0.0
        self._last_sy = 0.0

        # 车身 PD 状态
        self._prev_s1 = 0.0
        self._prev_s1_eff = 0.0  # 经 body_share 缩放后的 s1，用于 PD 微分项
        self._prev_body_t = 0.0
        self._last_body_az = 0.0  # 供 PTZ 前馈使用

        # 距离跟随状态
        self._prev_r_err = 0.0
        self._prev_linear_t = 0.0

        # FPS 显示
        self._fps = 0.0
        self._fps_n = 0
        self._fps_t = time.perf_counter()
        self._win_ok = False
        self._frame_count = 0
        self._diag_last_log = {}

        # 声明和加载参数
        self._declare_params()
        self._load_params()

        # 加载 HSV 颜色配置
        self._load_hsv_config()

        self._h_deg = self._init_h_deg
        self._p_deg = self._init_p_deg

        # 订阅图像话题
        self._sub = self.create_subscription(
            Image,
            self._img_topic,
            self._on_image,
            QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE
            ),
        )
        self._finished_pub = self.create_publisher(
            Bool, self._finished_topic, 5)

        # 发布云台控制
        self._pub_ptz = self.create_publisher(Vector3, self._ptz_topic, 10)

        # 发布车身控制
        self._pub_vel = self.create_publisher(
            Twist,
            self._cmd_vel_topic,
            QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE
            ),
        )

        # PTZ 控制定时器
        self._ptz_timer = self.create_timer(
            1.0 / max(self._ptz_rate, 1.0),
            self._ptz_tick,
        )

        # 车身控制定时器
        self._body_timer = self.create_timer(
            1.0 / max(self._body_rate, 1.0),
            self._body_tick,
        )
        self._target_lost_timer = self.create_timer(
            0.2, self._check_target_lost_timeout)

        # 初始化云台位置
        if self._pub_init:
            self._init_ptz_timer = None
            self._init_ptz_remain = max(0, self._init_n)
            if self._init_ptz_remain > 0:
                self._pub_ptz_cmd()
                self._init_ptz_remain -= 1
                if self._init_ptz_remain > 0:
                    self._init_ptz_timer = self.create_timer(
                        max(self._init_int, 0.02),
                        self._init_ptz_tick
                    )

        # 可视化窗口（show_debug_window 为真时开启，无鼠标/键盘交互，仅显示检测结果）
        if self._show_win:
            cv.namedWindow(self._win_name, cv.WINDOW_NORMAL)
            cv.resizeWindow(self._win_name, self._win_w, self._win_h)

        self.get_logger().info(
            f"Dify 颜色跟随节点已启动 (PTZ) | 颜色={self._color_name} | "
            f"HSV_min={self._hsv_min} HSV_max={self._hsv_max} | "
            f"image_topic={self._img_topic} | "
            f"stale={self._target_stale_timeout:.2f}s smooth={self._target_smooth_alpha:.2f} | "
            f"body_share={self._body_share:.2f} max_az={self._max_ang:.2f} "
            f"dist_follow={'on' if self._enable_dist else 'off'} | "
            f"diag={'开' if self._diag_enable else '关'} | "
            f"窗口={'开' if self._show_win else '关'}")

    def _declare_params(self) -> None:
        """声明参数"""
        # 颜色选择（Dify 必需参数），统一使用 color 字符串: red/green/blue/yellow
        self.declare_parameter("color", "red")
        self.declare_parameter("hsv_config_file",
                               "~/microros_car_v2_ws/config/color_hsv.txt")

        # 话题
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("ptz_command_topic", "/cmd_ptz_angle")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("finished_topic", "/visual_follow_finished")

        # 图像尺寸
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)

        # 可视化窗口（无鼠标/键盘交互，仅显示检测结果）
        self.declare_parameter("show_debug_window", False)
        self.declare_parameter("window_name", "color_follow_ptz_dify")
        self.declare_parameter("window_width", 640)
        self.declare_parameter("window_height", 480)
        self.declare_parameter("overlay_font_scale", 0.38)
        self.declare_parameter("overlay_line_height", 16)

        # 云台限制
        self.declare_parameter("horizontal_min_deg", -90.0)
        self.declare_parameter("horizontal_max_deg", 90.0)
        self.declare_parameter("pitch_min_deg", -90.0)
        self.declare_parameter("pitch_max_deg", 20.0)
        self.declare_parameter("initial_horizontal_deg", 0.0)
        self.declare_parameter("initial_pitch_deg", -45.0)
        self.declare_parameter("reverse_horizontal_direction", False)
        self.declare_parameter("reverse_pitch_direction", True)

        # PTZ PD 控制
        self.declare_parameter("pd_kp", 0.7)
        self.declare_parameter("pd_kd", 0.25)
        self.declare_parameter("deadband_x", 20.0)
        self.declare_parameter("deadband_y", 20.0)
        self.declare_parameter("ptz_step_smooth_alpha", 0.35)
        self.declare_parameter("max_ptz_step", 1.2)
        self.declare_parameter("ptz_smooth_alpha", 0.85)

        # 车身控制
        self.declare_parameter("angular_kp", 0.006)
        self.declare_parameter("angular_kd", 0.0)
        self.declare_parameter("angular_tolerance_deg", 3.0)
        self.declare_parameter("max_angular_z", 0.06)
        self.declare_parameter("body_share", 0.2)
        self.declare_parameter("reverse_body_direction", True)
        # 云台角速率前馈：车身旋转时 PTZ 反向补偿，防止目标被车身转动带偏
        self.declare_parameter("body_ff_gain", 0.5)

        # 距离跟随（按目标外接圆半径 PD 控制前进/后退）
        self.declare_parameter("enable_distance_follow", False)
        self.declare_parameter("target_radius", 80.0)
        self.declare_parameter("distance_kp", 0.008)
        self.declare_parameter("distance_kd", 0.003)
        self.declare_parameter("max_linear_x", 0.05)
        self.declare_parameter("distance_tolerance_r", 12.0)
        self.declare_parameter("reverse_linear_direction", False)

        # 检测参数
        self.declare_parameter("min_target_radius", 10.0)
        self.declare_parameter("min_target_area", 500.0)
        self.declare_parameter("morph_kernel_size", 5)
        self.declare_parameter("lost_frame_threshold", 5)
        self.declare_parameter("target_lost_timeout_sec", 10.0)
        self.declare_parameter("target_stale_timeout_sec", 0.5)
        self.declare_parameter("target_smooth_alpha", 0.35)
        self.declare_parameter("continuity_max_jump_px", 220.0)
        self.declare_parameter("diagnostic_log_enable", False)
        self.declare_parameter("diagnostic_log_interval_sec", 1.0)

        # 控制频率
        self.declare_parameter("ptz_control_rate", 20.0)
        self.declare_parameter("body_control_rate", 10.0)

        # 初始化
        self.declare_parameter("publish_initial_ptz_command", True)
        self.declare_parameter("initial_publish_count", 20)
        self.declare_parameter("initial_publish_interval_sec", 0.2)

    def _load_params(self) -> None:
        """加载参数"""
        # 统一使用 color 字符串参数（red/green/blue/yellow）
        self._color_name = str(self.get_parameter("color").value).lower()

        self._hsv_file = os.path.expanduser(
            str(self.get_parameter("hsv_config_file").value))

        self._img_topic = str(self.get_parameter("image_topic").value)
        self._ptz_topic = str(self.get_parameter("ptz_command_topic").value)
        self._cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._finished_topic = str(self.get_parameter("finished_topic").value)

        self._img_w = int(self.get_parameter("image_width").value)
        self._img_h = int(self.get_parameter("image_height").value)

        self._h_min = float(self.get_parameter("horizontal_min_deg").value)
        self._h_max = float(self.get_parameter("horizontal_max_deg").value)
        self._p_min = float(self.get_parameter("pitch_min_deg").value)
        self._p_max = float(self.get_parameter("pitch_max_deg").value)
        self._init_h_deg = float(self.get_parameter("initial_horizontal_deg").value)
        self._init_p_deg = float(self.get_parameter("initial_pitch_deg").value)
        self._rev_h = bool(self.get_parameter("reverse_horizontal_direction").value)
        self._rev_p = bool(self.get_parameter("reverse_pitch_direction").value)

        self._kp = max(float(self.get_parameter("pd_kp").value), 0.0)
        self._kd = max(float(self.get_parameter("pd_kd").value), 0.0)
        self._db_x = float(self.get_parameter("deadband_x").value)
        self._db_y = float(self.get_parameter("deadband_y").value)
        self._step_a = clamp(float(self.get_parameter("ptz_step_smooth_alpha").value), 0.0, 1.0)
        self._max_s = float(self.get_parameter("max_ptz_step").value)
        self._ptz_a = clamp(float(self.get_parameter("ptz_smooth_alpha").value), 0.01, 1.0)

        self._ang_kp = float(self.get_parameter("angular_kp").value)
        self._ang_kd = float(self.get_parameter("angular_kd").value)
        self._ang_tol = float(self.get_parameter("angular_tolerance_deg").value)
        self._max_ang = float(self.get_parameter("max_angular_z").value)
        self._body_share = clamp(float(self.get_parameter("body_share").value), 0.0, 1.0)
        self._rev_body = bool(self.get_parameter("reverse_body_direction").value)
        self._body_ff_gain = clamp(float(self.get_parameter("body_ff_gain").value), 0.0, 2.0)

        # 距离跟随参数
        self._enable_dist = bool(self.get_parameter("enable_distance_follow").value)
        self._target_radius = float(self.get_parameter("target_radius").value)
        self._dist_kp = max(float(self.get_parameter("distance_kp").value), 0.0)
        self._dist_kd = max(float(self.get_parameter("distance_kd").value), 0.0)
        self._max_lx = float(self.get_parameter("max_linear_x").value)
        self._dist_tol = float(self.get_parameter("distance_tolerance_r").value)
        self._rev_linear = bool(self.get_parameter("reverse_linear_direction").value)

        self._min_r = float(self.get_parameter("min_target_radius").value)
        self._min_area = float(self.get_parameter("min_target_area").value)
        self._morph_k = int(self.get_parameter("morph_kernel_size").value)
        self._lost_threshold = int(self.get_parameter("lost_frame_threshold").value)
        self._target_lost_timeout = max(
            0.0, float(self.get_parameter("target_lost_timeout_sec").value))
        self._target_stale_timeout = max(
            0.05, float(self.get_parameter("target_stale_timeout_sec").value))
        self._target_smooth_alpha = clamp(
            float(self.get_parameter("target_smooth_alpha").value), 0.0, 1.0)
        self._continuity_max_jump_px = max(
            0.0, float(self.get_parameter("continuity_max_jump_px").value))
        self._diag_enable = bool(self.get_parameter("diagnostic_log_enable").value)
        self._diag_interval = max(
            0.2, float(self.get_parameter("diagnostic_log_interval_sec").value))

        self._ptz_rate = float(self.get_parameter("ptz_control_rate").value)
        self._body_rate = float(self.get_parameter("body_control_rate").value)

        # 可视化窗口参数
        self._show_win = bool(self.get_parameter("show_debug_window").value)
        self._win_name = str(self.get_parameter("window_name").value)
        self._win_w = int(self.get_parameter("window_width").value)
        self._win_h = int(self.get_parameter("window_height").value)
        self._font = float(self.get_parameter("overlay_font_scale").value)
        self._lh = int(self.get_parameter("overlay_line_height").value)

        self._pub_init = bool(self.get_parameter("publish_initial_ptz_command").value)
        self._init_n = int(self.get_parameter("initial_publish_count").value)
        self._init_int = float(self.get_parameter("initial_publish_interval_sec").value)

    def _load_hsv_config(self) -> None:
        """从配置文件加载 HSV 值"""
        color_map = {"red": 0, "green": 1, "blue": 2, "yellow": 3}

        if self._color_name not in color_map:
            self.get_logger().error(
                f"无效颜色: {self._color_name}，必须是 red/green/blue/yellow")
            raise ValueError(f"Invalid color: {self._color_name}")

        color_index = color_map[self._color_name]
        hsv = read_hsv_from_file(self._hsv_file, color_index)

        if hsv is None:
            self.get_logger().error(
                f"无法从文件 {self._hsv_file} 加载颜色 {self._color_name} 的 HSV 配置")
            raise ValueError(f"Failed to load HSV config for {self._color_name}")

        self._hsv_min = np.array(hsv[0], dtype=np.uint8)
        self._hsv_max = np.array(hsv[1], dtype=np.uint8)

    def _diag_log(
            self, key: str, message: str,
            interval_sec: Optional[float] = None) -> None:
        """限频输出诊断日志，避免图像回调刷屏。"""
        if not self._diag_enable:
            return

        now = time.monotonic()
        min_interval = self._diag_interval if interval_sec is None else max(
            0.0, float(interval_sec))
        last_log_at = self._diag_last_log.get(key, 0.0)
        if now - last_log_at < min_interval:
            return

        self._diag_last_log[key] = now
        self.get_logger().info(f"[DIAG][{key}] {message}")

    def _reset_control_state(self) -> None:
        self._prev_err_x = 0.0
        self._prev_err_y = 0.0
        self._prev_ctrl_t = 0.0
        self._last_sx = 0.0
        self._last_sy = 0.0
        self._prev_s1 = 0.0
        self._prev_s1_eff = 0.0
        self._prev_body_t = 0.0
        self._last_body_az = 0.0
        self._prev_r_err = 0.0
        self._prev_linear_t = 0.0

    def _target_snapshot(self, clear_stale: bool = True) -> tuple:
        now = time.monotonic()
        stale = False
        with self._tgt_lock:
            age = now - self._last_target_seen_at
            fresh = (
                self._has_target
                and self._target_x is not None
                and self._target_y is not None
                and age <= self._target_stale_timeout
            )
            if not fresh and self._has_target and age > self._target_stale_timeout:
                stale = True
                if clear_stale:
                    self._has_target = False
            return (
                fresh,
                self._target_x,
                self._target_y,
                self._target_r,
                age,
                stale,
            )

    def _record_detection_miss(self) -> tuple:
        with self._tgt_lock:
            self._lost_frames += 1
            if self._lost_frames >= self._lost_threshold:
                self._has_target = False
            lost_frames = self._lost_frames
            lost_age = time.monotonic() - self._last_target_seen_at
            should_reset = lost_frames >= self._lost_threshold

        if should_reset:
            self._reset_control_state()
        return lost_frames, lost_age

    def _select_target_candidate(self, contours: list) -> tuple:
        prev_fresh, prev_x, prev_y, _, _, _ = self._target_snapshot(
            clear_stale=False)
        valid_candidates = []
        max_area = 0.0
        max_radius = 0.0

        for contour in contours:
            area = float(cv.contourArea(contour))
            max_area = max(max_area, area)
            (cx, cy), radius = cv.minEnclosingCircle(contour)
            radius = float(radius)
            max_radius = max(max_radius, radius)
            if area < self._min_area or radius < self._min_r:
                continue

            if prev_fresh and prev_x is not None and prev_y is not None:
                distance = math.hypot(float(cx) - prev_x, float(cy) - prev_y)
            else:
                distance = 0.0
            valid_candidates.append({
                "contour": contour,
                "area": area,
                "cx": float(cx),
                "cy": float(cy),
                "radius": radius,
                "distance": distance,
            })

        if not valid_candidates:
            reject_reason = "area" if max_area < self._min_area else "radius"
            return None, max_area, max_radius, reject_reason, "none"

        if prev_fresh and self._continuity_max_jump_px > 0.0:
            near_candidates = [
                item for item in valid_candidates
                if item["distance"] <= self._continuity_max_jump_px
            ]
            if near_candidates:
                selected = min(near_candidates, key=lambda item: item["distance"])
                return selected, max_area, max_radius, "", "nearest"

        selected = max(valid_candidates, key=lambda item: item["area"])
        mode = "largest_reset" if prev_fresh else "largest"
        return selected, max_area, max_radius, "", mode

    def _update_smoothed_target(
            self, raw_x: float, raw_y: float,
            raw_radius: float) -> tuple:
        now = time.monotonic()
        with self._tgt_lock:
            prev_fresh = (
                self._has_target
                and self._target_x is not None
                and self._target_y is not None
                and now - self._last_target_seen_at <= self._target_stale_timeout
            )
            alpha = self._target_smooth_alpha if prev_fresh else 1.0

            if prev_fresh:
                smoothed_x = (1.0 - alpha) * self._target_x + alpha * raw_x
                smoothed_y = (1.0 - alpha) * self._target_y + alpha * raw_y
                previous_radius = self._target_r if self._target_r is not None else raw_radius
                smoothed_radius = (
                    (1.0 - alpha) * previous_radius + alpha * raw_radius)
            else:
                smoothed_x = raw_x
                smoothed_y = raw_y
                smoothed_radius = raw_radius

            self._has_target = True
            self._target_x = float(smoothed_x)
            self._target_y = float(smoothed_y)
            self._target_r = float(smoothed_radius)
            self._lost_frames = 0
            self._last_target_seen_at = now

        return smoothed_x, smoothed_y, smoothed_radius, alpha, prev_fresh

    def _on_image(self, msg: Image) -> None:
        """图像回调 - HSV 检测"""
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if frame is None:
                return

            self._frame_count += 1
            raw_h, raw_w = frame.shape[:2]
            if self._img_w > 0 and self._img_h > 0:
                frame = cv.resize(frame, (self._img_w, self._img_h))
            frame_h, frame_w = frame.shape[:2]

            self._latest_frame = frame

            # HSV 检测
            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
            binary = cv.inRange(hsv, self._hsv_min, self._hsv_max)

            # 形态学处理
            if self._morph_k > 0:
                kernel = cv.getStructuringElement(
                    cv.MORPH_ELLIPSE,
                    (self._morph_k, self._morph_k)
                )
                binary = cv.morphologyEx(binary, cv.MORPH_OPEN, kernel)
                binary = cv.morphologyEx(binary, cv.MORPH_CLOSE, kernel)

            mask_pixels = int(cv.countNonZero(binary))
            # 查找轮廓
            contours, _ = cv.findContours(
                binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

            if not contours:
                lost_frames, lost_age = self._record_detection_miss()
                self._diag_log(
                    "DETECT",
                    f"frame={self._frame_count} raw={raw_w}x{raw_h} "
                    f"proc={frame_w}x{frame_h} mask={mask_pixels} contours=0 "
                    f"lost={lost_frames}/{self._lost_threshold} "
                    f"lost_age={lost_age:.1f}s")
                self._render_window(frame)
                return

            selected, max_area, max_radius, reject_reason, select_mode = (
                self._select_target_candidate(contours))

            if selected is None:
                lost_frames, lost_age = self._record_detection_miss()
                self._diag_log(
                    "DETECT",
                    f"frame={self._frame_count} mask={mask_pixels} "
                    f"contours={len(contours)} max_area={max_area:.1f} "
                    f"max_radius={max_radius:.1f} "
                    f"min_area={self._min_area:.1f} min_radius={self._min_r:.1f} "
                    f"rejected={reject_reason} "
                    f"lost={lost_frames}/{self._lost_threshold} "
                    f"lost_age={lost_age:.1f}s")
                self._render_window(frame)
                return

            raw_cx = selected["cx"]
            raw_cy = selected["cy"]
            raw_radius = selected["radius"]
            area = selected["area"]
            distance = selected["distance"]
            cx, cy, radius, smooth_alpha, smoothed_from_previous = (
                self._update_smoothed_target(raw_cx, raw_cy, raw_radius))

            err_x = cx - frame_w * 0.5
            err_y = cy - frame_h * 0.5
            self._diag_log(
                "DETECT",
                f"frame={self._frame_count} mask={mask_pixels} "
                f"contours={len(contours)} selected_area={area:.1f} "
                f"max_area={max_area:.1f} "
                f"raw=({raw_cx:.1f},{raw_cy:.1f},r={raw_radius:.1f}) "
                f"smooth=({cx:.1f},{cy:.1f},r={radius:.1f}) "
                f"err=({err_x:.1f},{err_y:.1f}) "
                f"mode={select_mode} dist={distance:.1f} "
                f"alpha={smooth_alpha:.2f} "
                f"prev={'yes' if smoothed_from_previous else 'no'} "
                f"radius={radius:.1f}/{self._target_radius:.1f} "
                f"fps={self._fps:.1f}")

            # 可视化：画检测圆
            if self._show_win:
                cv.circle(frame, (int(raw_cx), int(raw_cy)), int(raw_radius), (255, 0, 255), 1)
                cv.circle(frame, (int(cx), int(cy)), int(radius), (0, 255, 0), 2)
                cv.circle(frame, (int(cx), int(cy)), 3, (0, 0, 255), -1)

            self._render_window(frame)

            # FPS 统计
            self._fps_n += 1
            tnow = time.perf_counter()
            if tnow - self._fps_t >= 1.0:
                self._fps = self._fps_n / (tnow - self._fps_t)
                self._fps_n = 0
                self._fps_t = tnow

        except Exception as e:
            self.get_logger().error(f"图像处理错误: {e}")

    def _check_target_lost_timeout(self) -> None:
        """Publish completion once after the color target is lost too long."""
        if self._target_lost_timeout <= 0.0:
            return

        with self._tgt_lock:
            if self._finished_published:
                return
            target_lost_age = time.monotonic() - self._last_target_seen_at
            if target_lost_age < self._target_lost_timeout:
                return
            self._finished_published = True
            self._has_target = False

        self._pub_vel.publish(Twist())
        self._finished_pub.publish(Bool(data=True))
        self.get_logger().info(
            f"连续 {target_lost_age:.1f} 秒未检测到 {self._color_name} 色目标，"
            f"已停车并发布完成话题 {self._finished_topic}")

    def _render_window(self, frame: np.ndarray) -> None:
        """可视化窗口（仅显示，无交互）。show_debug_window 为假时直接返回。"""
        if not self._show_win:
            return
        iw, ih = frame.shape[1], frame.shape[0]
        icx, icy = iw // 2, ih // 2
        # 画面中心十字 + 死区框
        cv.drawMarker(frame, (icx, icy), (0, 255, 255), cv.MARKER_CROSS, 20, 2)
        cv.rectangle(frame, (icx - int(self._db_x), icy - int(self._db_y)),
                     (icx + int(self._db_x), icy + int(self._db_y)), (0, 180, 255), 2)
        with self._tgt_lock:
            ok = self._has_target
            tx, ty = self._target_x, self._target_y
            tr = self._target_r
            lf = self._lost_frames
        tgt_txt = f"Target:{int(tx)},{int(ty)}" if ok and tx is not None else f"Target:None(lost{lf}/{self._lost_threshold})"
        dist_txt = f"Dist r:{int(tr)}/{int(self._target_radius)}px" if tr else "Dist r:--"
        lines = [
            f"Color:{self._color_name} FPS:{int(self._fps)} {tgt_txt}",
            f"PTZ s1:{self._h_deg:.1f} s2:{self._p_deg:.1f} share:{self._body_share:.1f}",
            f"{dist_txt} dist_follow:{'on' if self._enable_dist else 'off'}",
        ]
        for i, t in enumerate(lines):
            cv.putText(frame, t, (8, self._my_line(i)), cv.FONT_HERSHEY_SIMPLEX,
                       self._font, (0, 255, 255), 1, cv.LINE_AA)
        cv.imshow(self._win_name, frame)
        cv.waitKey(1)

    def _my_line(self, i: int) -> int:
        return 16 + i * self._lh

    def _init_ptz_tick(self) -> None:
        """初始化云台位置定时器"""
        if self._init_ptz_remain <= 0:
            if hasattr(self, '_init_ptz_timer') and self._init_ptz_timer is not None:
                self.destroy_timer(self._init_ptz_timer)
                self._init_ptz_timer = None
            return
        self._pub_ptz_cmd()
        self._init_ptz_remain -= 1

    def _pub_ptz_cmd(self) -> None:
        """发布云台角度"""
        m = Vector3()
        m.x = float(self._h_deg)
        m.y = float(self._p_deg)
        m.z = 0.0
        self._pub_ptz.publish(m)

    def _ptz_tick(self) -> None:
        """PTZ PD 控制循环"""
        ok, tx, ty, _, target_age, stale = self._target_snapshot()

        if not ok or tx is None or ty is None:
            self._reset_control_state()
            state = "stale" if stale else "lost"
            self._diag_log(
                "PTZ",
                f"target_{state} age={target_age:.2f}s "
                f"stale_timeout={self._target_stale_timeout:.2f}s hold_position")
            return

        # 计算归一化误差
        icx, icy = self._img_w * 0.5, self._img_h * 0.5
        pex, pey = tx - icx, ty - icy

        # 死区
        if abs(pex) < self._db_x:
            pex = 0.0
        if abs(pey) < self._db_y:
            pey = 0.0

        # 时间步长
        n = time.perf_counter()
        dt = n - self._prev_ctrl_t if self._prev_ctrl_t > 0.0 else 1.0 / max(self._ptz_rate, 1.0)
        dt = max(dt, 1e-3)

        # 归一化误差 + 微分
        nex, ney = pex / icx, pey / icy
        dx = (nex - self._prev_err_x) / dt
        dy = (ney - self._prev_err_y) / dt

        # PD 控制
        rx = clamp((self._kp * nex + self._kd * dx) * self._max_s, -self._max_s, self._max_s)
        ry = clamp((self._kp * ney + self._kd * dy) * self._max_s, -self._max_s, self._max_s)

        # 角速率前馈：车身旋转 → PTZ 同步反向补偿，目标不被车身转动带偏。
        # body az(rad/s) / ptz_rate * 180/π = 每 PTZ tick 车身旋转度数，PTZ 反向旋转抵消。
        body_rot_deg = self._last_body_az * (180.0 / math.pi) / max(self._ptz_rate, 1.0)
        rx = clamp(rx - body_rot_deg * self._body_ff_gain, -self._max_s, self._max_s)

        # 步长平滑
        sx = (1.0 - self._step_a) * self._last_sx + self._step_a * rx
        sy = (1.0 - self._step_a) * self._last_sy + self._step_a * ry

        # 角度平滑 + 限幅
        dir_h = -1.0 if self._rev_h else 1.0
        dir_p = -1.0 if self._rev_p else 1.0

        old_h = self._h_deg
        old_p = self._p_deg
        new_h = clamp(self._h_deg + dir_h * sx, self._h_min, self._h_max)
        new_p = clamp(self._p_deg + dir_p * sy, self._p_min, self._p_max)

        self._h_deg = (1.0 - self._ptz_a) * self._h_deg + self._ptz_a * new_h
        self._p_deg = (1.0 - self._ptz_a) * self._p_deg + self._ptz_a * new_p

        self._diag_log(
            "PTZ",
            f"target=({tx:.1f},{ty:.1f}) err_px=({pex:.1f},{pey:.1f}) "
            f"err_norm=({nex:.3f},{ney:.3f}) pd=({rx:.3f},{ry:.3f}) "
            f"body_ff={body_rot_deg * self._body_ff_gain:.3f} "
            f"step=({sx:.3f},{sy:.3f}) "
            f"angle=({old_h:.2f},{old_p:.2f})->({self._h_deg:.2f},{self._p_deg:.2f})")

        # 发布
        self._pub_ptz_cmd()

        # 更新状态
        self._prev_err_x = nex
        self._prev_err_y = ney
        self._prev_ctrl_t = n
        self._last_sx = sx
        self._last_sy = sy

    def _body_tick(self) -> None:
        """车身跟随控制 - 云台水平角归零 (angular) + 距离保持 (linear)。"""
        ok, _, _, r, target_age, stale = self._target_snapshot()

        if not ok:
            # 目标丢失，冻结车身，防止基于过期状态自转
            self._pub_vel.publish(Twist())
            self._reset_control_state()
            state = "stale" if stale else "lost"
            self._diag_log(
                "BODY",
                f"target_{state} age={target_age:.2f}s "
                f"stale_timeout={self._target_stale_timeout:.2f}s "
                "cmd_vel=(0.000,0.000)")
            return

        # ── 角速度：车身分担 body_share 比例的云台归零任务 ──
        s1 = self._h_deg
        if abs(s1) < self._ang_tol:
            s1 = 0.0
        s1_eff = s1 * self._body_share

        n = time.perf_counter()
        dt = n - self._prev_body_t if self._prev_body_t > 0.0 else 1.0 / max(self._body_rate, 1.0)
        dt = max(dt, 1e-3)
        # 微分项基准用 body_share 缩放后的 s1_eff，与原版量纲一致
        ds1_eff = (s1_eff - self._prev_s1_eff) / dt
        angular_z = self._ang_kp * s1_eff + self._ang_kd * ds1_eff
        self._prev_s1 = s1
        self._prev_s1_eff = s1_eff
        self._prev_body_t = n

        dir_body = -1.0 if self._rev_body else 1.0
        angular_z = clamp(dir_body * angular_z, -self._max_ang, self._max_ang)
        self._last_body_az = angular_z  # 供 PTZ 角速率前馈使用

        # ── 线性距离跟随 PD：按目标外接圆半径保持距离 ──
        r_err = 0.0
        if self._enable_dist and r is not None and r > 0:
            r_err = self._target_radius - r  # >0 目标太远需前进
            if abs(r_err) < self._dist_tol:
                r_err = 0.0
            dt_l = n - self._prev_linear_t if self._prev_linear_t > 0.0 else 1.0 / max(self._body_rate, 1.0)
            dt_l = max(dt_l, 1e-3)
            dr = (r_err - self._prev_r_err) / dt_l
            lx = self._dist_kp * r_err + self._dist_kd * dr
            self._prev_r_err = r_err
            self._prev_linear_t = n
            if self._rev_linear:
                lx = -lx
            lx = clamp(lx, -self._max_lx, self._max_lx)
        else:
            self._prev_r_err = 0.0
            lx = 0.0

        cmd = Twist()
        cmd.linear.x = float(lx)
        cmd.angular.z = float(angular_z)
        self._pub_vel.publish(cmd)
        self._diag_log(
            "BODY",
            f"s1={s1:.2f} share={self._body_share:.2f} "
            f"s1_eff={s1_eff:.2f} angular_z={angular_z:.3f} "
            f"radius={(r or 0.0):.1f}/{self._target_radius:.1f} "
            f"r_err={r_err:.1f} linear_x={lx:.3f}")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ColorFollowPtzDifyNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"节点错误: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
