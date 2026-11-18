#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
color_follow_node.py

中文说明:
    ROS2 Humble 颜色跟随节点（No-PTZ 版，纯车身控制）。
    摄像头固定朝向正前方。HSV 颜色检测 → 车身 PD 基于像素误差控制角速度 + 色块尺寸距离跟随。
    识别逻辑完全对齐 color_follow_ptz_node.py，仅去掉 PTZ 层，车身 100% 承担水平跟踪。

English:
    ROS2 Humble color follow node (No-PTZ, body-only).
    Camera fixed forward. HSV detection → body PD uses pixel error for angular velocity + blob size distance follow.
    Detection logic fully aligned with color_follow_ptz_node.py; PTZ removed, body 100% horizontal tracking.
"""

from __future__ import annotations

import math
import os
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import cv2 as cv
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from .image_conversion import image_to_bgr

DEFAULT_NODE_NAME = "color_follow_node"

STATE_IDLE = "idle"
STATE_TRACKING = "tracking"

KEY_CODE_SPACE = 32
KEY_CODE_ESCAPE = 27
MINIMUM_TIMER_RATE_HZ = 1.0

DYNAMIC_PARAMETER_NAMES = frozenset({
    "show_debug_window", "show_binary_window",
    "image_width", "image_height", "window_width", "window_height",
    "overlay_font_scale", "overlay_line_height",
    "overlay_margin_x", "overlay_margin_y", "overlay_show_background",
    "enable_horizontal_flip", "enable_vertical_flip", "rotate_image_code",
    "deadband_x_pixel",
    "Hmin", "Smin", "Vmin", "Hmax", "Smax", "Vmax",
    "h_expand", "s_expand", "v_expand", "hsv_file",
    "min_target_radius", "min_target_area", "morph_kernel_size",
    "angular_kp", "angular_kd", "max_angular_z",
    "lost_frame_threshold", "reverse_body_direction",
    "enable_distance_follow", "target_radius",
    "distance_kp", "distance_kd", "max_linear_x",
    "distance_tolerance_r", "reverse_linear_direction",
    "body_control_rate",
    "show_deadband_lines", "show_center_line",
    "enable_rqt_dynamic_parameters",
})

RESTART_REQUIRED_PARAMETER_NAMES = frozenset({
    "image_topic", "cmd_vel_topic", "window_name",
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


def read_hsv_file(path: str) -> Optional[Tuple[Tuple[int, int, int], Tuple[int, int, int]]]:
    if not path or not os.path.exists(os.path.expanduser(path)): return None
    try:
        with open(os.path.expanduser(path), "r", encoding="utf-8") as f:
            line = f.readline().strip()
        if not line: return None
        vals = [int(v.strip()) for v in line.split(",")]
        if len(vals) != 6: return None
        return (vals[0], vals[1], vals[2]), (vals[3], vals[4], vals[5])
    except Exception:
        return None


def write_hsv_file(path: str, hsv: Tuple[Tuple[int, int, int], Tuple[int, int, int]]) -> None:
    if not path: return
    p = os.path.expanduser(path)
    os.makedirs(os.path.dirname(p), exist_ok=True) if os.path.dirname(p) else None
    with open(p, "w", encoding="utf-8") as f:
        f.write(f"{hsv[0][0]}, {hsv[0][1]}, {hsv[0][2]}, {hsv[1][0]}, {hsv[1][1]}, {hsv[1][2]}")
        f.flush()


class ColorFollowNode(Node):

    def __init__(self) -> None:
        super().__init__(DEFAULT_NODE_NAME)
        self._tgt_lock = threading.Lock()
        self._latest_frame: Optional[np.ndarray] = None

        self._state = STATE_IDLE
        self._shutdown = False

        # 目标状态
        self._has_target = False
        self._lost_frames = 0; self._lost_frame_threshold = 5
        self._target_x: Optional[float] = None
        self._target_y: Optional[float] = None
        self._target_r: Optional[float] = None

        # HSV + ROI
        self._hsv = ((0, 85, 126), (9, 253, 253))
        self._mouse_pressed = False; self._mouse_start = (0, 0); self._mouse_end = (0, 0)
        self._roi_ready = False

        # 车身 PD 状态
        self._prev_s1 = 0.0; self._prev_body_t = 0.0
        self._last_body_az = 0.0

        # 距离跟随状态
        self._target_radius = 60.0
        self._prev_r_err = 0.0; self._prev_linear_t = 0.0

        self._fps = 0.0; self._fps_n = 0; self._fps_t = time.perf_counter()
        self._win_ok = False

        self._declare_params()
        self._load_params()

        # 从文件加载 HSV
        saved = read_hsv_file(self._hsv_file)
        if saved:
            self._hsv = saved
            self._apply_hsv_to_params(saved)
            self.get_logger().info(f"Loaded HSV from file: {self._hsv_file}")

        best_effort_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._sub = self.create_subscription(
            Image, self._img_topic, self._on_image, best_effort_qos)
        self._cmd_pub = self.create_publisher(Twist, self._cmd_topic, best_effort_qos)

        # 车身控制定时器
        self._body_timer = None; self._rebuild_body_timer(False)
        self._key_timer = None
        if self._show_win:
            self._init_win()
            self._key_timer = self.create_timer(0.05, self._poll_keys)

        self._param_cb = self.add_on_set_parameters_callback(self._on_param_update)
        self.get_logger().info(
            f"Color follow body-only started. r=ROI, SPACE=toggle, i=idle, q=quit. "
            f"body={self._body_rate:.0f}Hz")

    # ── 参数 ─────────────────────────────────────────────────
    def _declare_params(self) -> None:
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("window_name", "color_follow")
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
        self.declare_parameter("deadband_x_pixel", 60.0)
        self.declare_parameter("Hmin", 0); self.declare_parameter("Smin", 85); self.declare_parameter("Vmin", 126)
        self.declare_parameter("Hmax", 9); self.declare_parameter("Smax", 253); self.declare_parameter("Vmax", 253)
        self.declare_parameter("h_expand", 5); self.declare_parameter("s_expand", 20); self.declare_parameter("v_expand", 20)
        self.declare_parameter("hsv_file", "")
        self.declare_parameter("min_target_radius", 5.0); self.declare_parameter("min_target_area", 60.0)
        self.declare_parameter("morph_kernel_size", 5)
        self.declare_parameter("angular_kp", 1.5); self.declare_parameter("angular_kd", 0.45)
        self.declare_parameter("lost_frame_threshold", 5)
        self.declare_parameter("max_angular_z", 0.6); self.declare_parameter("reverse_body_direction", True)
        self.declare_parameter("enable_distance_follow", True)
        self.declare_parameter("target_radius", 60.0)
        self.declare_parameter("distance_kp", 0.008); self.declare_parameter("distance_kd", 0.003)
        self.declare_parameter("max_linear_x", 0.12); self.declare_parameter("reverse_linear_direction", False)
        self.declare_parameter("distance_tolerance_r", 12.0)
        self.declare_parameter("body_control_rate", 10.0)
        self.declare_parameter("show_deadband_lines", False)
        self.declare_parameter("show_center_line", False)
        self.declare_parameter("enable_rqt_dynamic_parameters", True)

    def _load_params(self) -> None:
        self._img_topic = str(self.get_parameter("image_topic").value)
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
        self._db_x = float(self.get_parameter("deadband_x_pixel").value)
        self._h_min_v = int(self.get_parameter("Hmin").value); self._s_min_v = int(self.get_parameter("Smin").value)
        self._v_min_v = int(self.get_parameter("Vmin").value); self._h_max_v = int(self.get_parameter("Hmax").value)
        self._s_max_v = int(self.get_parameter("Smax").value); self._v_max_v = int(self.get_parameter("Vmax").value)
        self._h_ex = int(self.get_parameter("h_expand").value); self._s_ex = int(self.get_parameter("s_expand").value)
        self._v_ex = int(self.get_parameter("v_expand").value)
        self._hsv_file = str(self.get_parameter("hsv_file").value)
        self._min_r = float(self.get_parameter("min_target_radius").value)
        self._min_a = float(self.get_parameter("min_target_area").value)
        self._morph_k = int(self.get_parameter("morph_kernel_size").value)
        self._body_kp = float(self.get_parameter("angular_kp").value)
        self._body_kd = float(self.get_parameter("angular_kd").value)
        self._lost_frame_threshold = max(1, int(self.get_parameter("lost_frame_threshold").value))
        self._max_az = float(self.get_parameter("max_angular_z").value)
        self._rev_body = to_bool(self.get_parameter("reverse_body_direction").value, False)
        self._enable_dist = to_bool(self.get_parameter("enable_distance_follow").value, True)
        self._target_radius = float(self.get_parameter("target_radius").value)
        self._dist_kp = max(float(self.get_parameter("distance_kp").value), 0.0)
        self._dist_kd = max(float(self.get_parameter("distance_kd").value), 0.0)
        self._max_lx = float(self.get_parameter("max_linear_x").value)
        self._dist_tol = float(self.get_parameter("distance_tolerance_r").value)
        self._rev_linear = to_bool(self.get_parameter("reverse_linear_direction").value, False)
        self._rqt = to_bool(self.get_parameter("enable_rqt_dynamic_parameters").value, True)
        self._show_deadband = bool(self.get_parameter("show_deadband_lines").value)
        self._show_center = bool(self.get_parameter("show_center_line").value)
        self._body_rate = float(self.get_parameter("body_control_rate").value)

    def _apply_hsv_to_params(self, hsv):
        self._h_min_v, self._s_min_v, self._v_min_v = hsv[0]; self._h_max_v, self._s_max_v, self._v_max_v = hsv[1]
        self.set_parameters([
            Parameter("Hmin", Parameter.Type.INTEGER, int(hsv[0][0])),
            Parameter("Smin", Parameter.Type.INTEGER, int(hsv[0][1])),
            Parameter("Vmin", Parameter.Type.INTEGER, int(hsv[0][2])),
            Parameter("Hmax", Parameter.Type.INTEGER, int(hsv[1][0])),
            Parameter("Smax", Parameter.Type.INTEGER, int(hsv[1][1])),
            Parameter("Vmax", Parameter.Type.INTEGER, int(hsv[1][2])),
        ])

    # ── 图像处理（对齐 PTZ 版）───────────────────────────────

    def _on_image(self, msg: Image) -> None:
        try:
            frame = image_to_bgr(msg)
            if frame is None: return
            if self._flip_h: frame = cv.flip(frame, 1)
            if self._flip_v: frame = cv.flip(frame, 0)
            if self._rot == "90_clockwise": frame = cv.rotate(frame, cv.ROTATE_90_CLOCKWISE)
            elif self._rot == "90_counterclockwise": frame = cv.rotate(frame, cv.ROTATE_90_COUNTERCLOCKWISE)
            elif self._rot == "180": frame = cv.rotate(frame, cv.ROTATE_180)
            if self._img_w > 0 and self._img_h > 0:
                frame = cv.resize(frame, (self._img_w, self._img_h))
            self._latest_frame = frame
        except Exception:
            pass

        # ROI 取色处理
        if self._roi_ready and self._mouse_start != self._mouse_end:
            self._sample_hsv(frame)
            self._roi_ready = False

        # HSV 检测
        disp = frame.copy()
        self._detect(disp)

        if self._show_win:
            self._draw_overlay(disp)
            cv.imshow(self._win_name, disp)

        self._fps_n += 1; n = time.perf_counter()
        if n - self._fps_t >= 1.0: self._fps = self._fps_n / (n - self._fps_t); self._fps_n = 0; self._fps_t = n

    def _detect(self, frame: np.ndarray) -> None:
        hsv_img = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        lower = np.array([self._h_min_v, self._s_min_v, self._v_min_v], dtype=np.uint8)
        upper = np.array([self._h_max_v, self._s_max_v, self._v_max_v], dtype=np.uint8)

        mask = cv.inRange(hsv_img, lower, upper)
        k = max(1, self._morph_k)
        if k % 2 == 0: k += 1
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (k, k))
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)

        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if not contours:
            self._mark_lost()
            return

        best = max(contours, key=cv.contourArea)
        area = cv.contourArea(best)
        if area < self._min_a:
            self._mark_lost()
            return

        (cx, cy), r = cv.minEnclosingCircle(best)
        if r < self._min_r:
            self._mark_lost()
            return

        cv.circle(frame, (int(cx), int(cy)), int(r), (255, 0, 255), 2)
        cv.circle(frame, (int(cx), int(cy)), 3, (0, 0, 255), -1)

        with self._tgt_lock:
            self._lost_frames = 0
            self._has_target = True
            self._target_x = float(cx); self._target_y = float(cy)
            self._target_r = float(r)

    def _mark_lost(self) -> None:
        """N 帧容错：连续丢帧达到阈值才判定目标丢失，过滤光照抖动等瞬态丢帧。"""
        with self._tgt_lock:
            self._lost_frames += 1
            if self._lost_frames >= self._lost_frame_threshold:
                self._has_target = False

    # ── 车身控制（No-PTZ：像素误差直接驱动 angle PD）─────────
    def _rebuild_body_timer(self, log: bool) -> None:
        p = 1.0 / max(self._body_rate, MINIMUM_TIMER_RATE_HZ)
        if self._body_timer is not None: self.destroy_timer(self._body_timer)
        self._body_timer = self.create_timer(p, self._body_tick)
        if log: self.get_logger().info(f"Body timer: {self._body_rate:.1f}Hz")

    def _body_tick(self) -> None:
        if self._state != STATE_TRACKING:
            self._prev_s1 = 0.0; self._prev_body_t = 0.0
            self._pub_zero_cmd()
            return

        with self._tgt_lock:
            ok = self._has_target; tx = self._target_x
        if not ok or tx is None:
            self._prev_s1 = 0.0; self._prev_body_t = 0.0
            self._pub_zero_cmd()
            return

        # 像素误差 → 归一化 → 车身角速度 PD
        icx = self._img_w * 0.5
        pixel_err = tx - icx
        if abs(pixel_err) < self._db_x: pixel_err = 0.0
        s1 = pixel_err / icx

        n = time.perf_counter()
        dt = n - self._prev_body_t if self._prev_body_t > 0.0 else 1.0 / max(self._body_rate, MINIMUM_TIMER_RATE_HZ)
        dt = max(dt, 1e-3)
        ds1 = (s1 - self._prev_s1) / dt
        az = self._body_kp * s1 + self._body_kd * ds1
        self._prev_s1 = s1; self._prev_body_t = n

        if self._rev_body: az = -az
        az = clamp(az, -self._max_az, self._max_az)
        self._last_body_az = az

        # ── 线性距离跟随 PD ──
        with self._tgt_lock:
            r = self._target_r
        if self._enable_dist and r is not None and r > 0:
            r_err = self._target_radius - r
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
    def _init_win(self) -> None:
        cv.namedWindow(self._win_name, cv.WINDOW_NORMAL)
        cv.resizeWindow(self._win_name, self._win_w, self._win_h)
        cv.setMouseCallback(self._win_name, self._on_mouse)

    def _on_mouse(self, event, x, y, flags, param):
        if event == cv.EVENT_LBUTTONDOWN:
            self._mouse_pressed = True; self._roi_ready = False
            self._mouse_start = (x, y); self._mouse_end = (x, y)
        elif event == cv.EVENT_MOUSEMOVE and self._mouse_pressed:
            self._mouse_end = (x, y)
        elif event == cv.EVENT_LBUTTONUP:
            self._mouse_pressed = False; self._mouse_end = (x, y); self._roi_ready = True

    def _sample_hsv(self, frame: np.ndarray) -> None:
        # 缩放显示坐标到图像分辨率
        sx = self._img_w / max(self._win_w, 1)
        sy = self._img_h / max(self._win_h, 1)
        x1 = int(min(self._mouse_start[0], self._mouse_end[0]) * sx)
        y1 = int(min(self._mouse_start[1], self._mouse_end[1]) * sy)
        x2 = int(max(self._mouse_start[0], self._mouse_end[0]) * sx)
        y2 = int(max(self._mouse_start[1], self._mouse_end[1]) * sy)
        if x2 - x1 < 3 or y2 - y1 < 3: return

        roi = frame[y1:y2, x1:x2]
        hsv = cv.cvtColor(roi, cv.COLOR_BGR2HSV)
        h_low = int(max(0, np.min(hsv[:, :, 0]) - self._h_ex))
        h_hi = int(min(179, np.max(hsv[:, :, 0]) + self._h_ex))
        s_low = int(max(0, np.min(hsv[:, :, 1]) - self._s_ex))
        s_hi = int(min(255, max(np.max(hsv[:, :, 1]) + self._s_ex, 253)))
        v_low = int(max(0, np.min(hsv[:, :, 2]) - self._v_ex))
        v_hi = int(min(255, max(np.max(hsv[:, :, 2]) + self._v_ex, 253)))
        self._hsv = ((h_low, s_low, v_low), (h_hi, s_hi, v_hi))
        self._apply_hsv_to_params(self._hsv)
        write_hsv_file(self._hsv_file, self._hsv)
        self.get_logger().info(f"HSV sampled: {self._hsv}")

    def _poll_keys(self) -> None:
        try: k = cv.waitKey(1) & 0xFF
        except Exception: return
        if k in (255, -1): return
        if k in (ord("q"), ord("Q"), KEY_CODE_ESCAPE):
            self._shutdown = True
            self._pub_zero_cmd()
            try: self.get_logger().info("Force exiting.")
            except Exception: pass
            os._exit(0)
        elif k in (ord("r"), ord("R")):
            self._roi_ready = True
            self.get_logger().info("Drag mouse to sample target color.")
        elif k == KEY_CODE_SPACE:
            if self._state == STATE_TRACKING:
                self._state = STATE_IDLE; self._reset_pd(); self._pub_zero_cmd()
                self.get_logger().info("IDLE")
            else:
                self._state = STATE_TRACKING; self._reset_pd()
                self.get_logger().info("TRACKING (body-only)")
        elif k in (ord("i"), ord("I")):
            self._state = STATE_IDLE; self._reset_pd(); self._pub_zero_cmd()
            self.get_logger().info("IDLE")

    def _reset_pd(self) -> None:
        self._prev_s1 = 0.0; self._prev_body_t = 0.0
        self._last_body_az = 0.0; self._lost_frames = 0
        self._target_r = None; self._prev_r_err = 0.0; self._prev_linear_t = 0.0

    # ── 显示 ────────────────────────────────────────────────
    def _draw_overlay(self, frame: np.ndarray) -> None:
        iw, ih = frame.shape[1], frame.shape[0]; icx, icy = iw // 2, ih // 2
        # 画面水平正中间红色竖线
        if self._show_center:
            cv.line(frame, (icx, 0), (icx, ih), (0, 0, 255), 2)
        # 死区边界两条橙色竖线，间距 = 2 × deadband_x_pixel
        if self._show_deadband and self._db_x > 0:
            cv.line(frame, (icx - int(self._db_x), 0), (icx - int(self._db_x), ih), (0, 180, 255), 2)
            cv.line(frame, (icx + int(self._db_x), 0), (icx + int(self._db_x), ih), (0, 180, 255), 2)
        if self._mouse_pressed:
            cv.rectangle(frame, self._mouse_start, self._mouse_end, (0, 255, 0), 2)
        with self._tgt_lock:
            ok = self._has_target; tx, ty = self._target_x, self._target_y
            tr = self._target_r; lf = self._lost_frames
        tgt_txt = f"Target:{int(tx)},{int(ty)}" if ok and tx else f"Target:None(lost{lf}/{self._lost_frame_threshold})"
        dist_txt = f"Dist r:{int(tr)}/{int(self._target_radius)}px err:{int(self._target_radius - tr) if tr else 0}" if tr else "Dist r:--"
        lines = [
            f"State:{self._state} FPS:{int(self._fps)} {tgt_txt}",
            f"Body kp:{self._body_kp:.2f} kd:{self._body_kd:.2f} max_az:{self._max_az:.2f} dead:{self._db_x:.0f} dist:{'on' if self._enable_dist else 'off'}",
            f"{dist_txt}",
            "r:ROI | SPACE:toggle | q:quit",
        ]
        if self._ov_bg:
            bh = self._lh * len(lines) + 8
            cv.rectangle(frame, (0, 0), (iw, min(ih, bh)), (0, 0, 0), -1)
        for i, t in enumerate(lines):
            cv.putText(frame, t, (self._mx, self._my + i * self._lh), cv.FONT_HERSHEY_SIMPLEX, self._font, (0, 255, 255), 1, cv.LINE_AA)

    # ── 动态参数更新 ───────────────────────────────────────
    def _on_param_update(self, params: List[Parameter]) -> SetParametersResult:
        if not params: return SetParametersResult(successful=True)
        names = {p.name for p in params}
        bad = names - DYNAMIC_PARAMETER_NAMES - RESTART_REQUIRED_PARAMETER_NAMES
        if bad: return SetParametersResult(successful=False, reason=f"Unsupported: {sorted(bad)}")
        if names & RESTART_REQUIRED_PARAMETER_NAMES: return SetParametersResult(successful=False, reason="Restart required")
        if not self._rqt and names != {"enable_rqt_dynamic_parameters"}: return SetParametersResult(successful=False, reason="Disabled")

        old_body = self._body_rate
        old_debug, old_size = self._show_win, (self._win_w, self._win_h)
        v = self._merge(params)
        err = self._check(v)
        if err: return SetParametersResult(successful=False, reason=err)
        self._apply(v)

        if abs(self._body_rate - old_body) > 1e-9: self._rebuild_body_timer(True)
        self._sync_win(old_debug, old_size)
        return SetParametersResult(successful=True)

    def _merge(self, params):
        d = dict(show_debug_window=self._show_win, show_binary_window=self._show_bin,
                 image_width=self._img_w, image_height=self._img_h, window_width=self._win_w, window_height=self._win_h,
                 overlay_font_scale=self._font, overlay_line_height=self._lh, overlay_margin_x=self._mx, overlay_margin_y=self._my,
                 overlay_show_background=self._ov_bg, enable_horizontal_flip=self._flip_h, enable_vertical_flip=self._flip_v,
                 rotate_image_code=self._rot, deadband_x_pixel=self._db_x,
                 Hmin=self._h_min_v, Smin=self._s_min_v, Vmin=self._v_min_v, Hmax=self._h_max_v, Smax=self._s_max_v, Vmax=self._v_max_v,
                 h_expand=self._h_ex, s_expand=self._s_ex, v_expand=self._v_ex, hsv_file=self._hsv_file,
                 min_target_radius=self._min_r, min_target_area=self._min_a, morph_kernel_size=self._morph_k,
                 angular_kp=self._body_kp, angular_kd=self._body_kd,
                 max_angular_z=self._max_az,
                 lost_frame_threshold=self._lost_frame_threshold,
                 reverse_body_direction=self._rev_body,
                 enable_distance_follow=self._enable_dist,
                 target_radius=self._target_radius,
                 distance_kp=self._dist_kp, distance_kd=self._dist_kd,
                 max_linear_x=self._max_lx, distance_tolerance_r=self._dist_tol,
                 reverse_linear_direction=self._rev_linear,
                 body_control_rate=self._body_rate,
                 show_deadband_lines=self._show_deadband, show_center_line=self._show_center,
                 enable_rqt_dynamic_parameters=self._rqt)
        for p in params:
            if p.type_ != Parameter.Type.NOT_SET: d[p.name] = p.value
        return d

    def _check(self, v):
        try:
            if float(v["angular_kp"]) < 0 or float(v["angular_kd"]) < 0: return "angular_kp/kd >=0"
            if int(v["lost_frame_threshold"]) < 1: return "lost_frame_threshold >=1"
            if float(v["max_angular_z"]) <= 0: return "max_angular_z >0"
            if float(v["max_linear_x"]) <= 0: return "max_linear_x >0"
            if float(v["distance_kp"]) < 0 or float(v["distance_kd"]) < 0: return "distance_kp/kd >=0"
            if float(v["deadband_x_pixel"]) < 0: return "deadband >=0"
            if float(v["body_control_rate"]) < 1.0: return "rate >=1"
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
        self._db_x = float(v["deadband_x_pixel"])
        self._h_min_v = int(v["Hmin"]); self._s_min_v = int(v["Smin"]); self._v_min_v = int(v["Vmin"])
        self._h_max_v = int(v["Hmax"]); self._s_max_v = int(v["Smax"]); self._v_max_v = int(v["Vmax"])
        self._h_ex = int(v["h_expand"]); self._s_ex = int(v["s_expand"]); self._v_ex = int(v["v_expand"])
        self._hsv_file = str(v["hsv_file"]); self._min_r = float(v["min_target_radius"]); self._min_a = float(v["min_target_area"])
        self._morph_k = int(v["morph_kernel_size"])
        self._body_kp = float(v["angular_kp"]); self._body_kd = float(v["angular_kd"])
        self._lost_frame_threshold = max(1, int(v["lost_frame_threshold"]))
        self._max_az = float(v["max_angular_z"]); self._rev_body = to_bool(v["reverse_body_direction"], False)
        self._enable_dist = to_bool(v["enable_distance_follow"], True)
        self._target_radius = float(v["target_radius"])
        self._dist_kp = max(float(v["distance_kp"]), 0.0); self._dist_kd = max(float(v["distance_kd"]), 0.0)
        self._max_lx = float(v["max_linear_x"]); self._dist_tol = float(v["distance_tolerance_r"])
        self._rev_linear = to_bool(v["reverse_linear_direction"], False)
        self._body_rate = float(v["body_control_rate"])
        self._rqt = to_bool(v["enable_rqt_dynamic_parameters"], True)
        self._show_deadband = to_bool(v["show_deadband_lines"], False)
        self._show_center = to_bool(v["show_center_line"], False)
        self._reset_pd()

    def _sync_win(self, prev_debug, prev_size):
        try:
            chg = prev_size != (self._win_w, self._win_h)
            if self._show_win and not prev_debug: self._init_win()
            elif not self._show_win and prev_debug: cv.destroyWindow(self._win_name)
            elif self._show_win and chg: cv.resizeWindow(self._win_name, self._win_w, self._win_h)
        except Exception: pass

    def destroy_node(self) -> bool:
        self._pub_zero_cmd()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ColorFollowNode()
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
