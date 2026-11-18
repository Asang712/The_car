#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gesture_tracker_ptz_node.py

中文说明:
    ROS2 Humble 手势追踪云台控制节点。
    基于 MediaPipe 手势检测，锁定食指根部关节（landmark 5），通过 PD 控制舵机云台。

    状态机（极简）:
    - 仅两个状态: IDLE（初始/暂停）←→ TRACKING（追踪会话中）
    - 状态切换仅通过 SPACE 键
    - 手势检测永不改变状态，拳头/无手仅在 TRACKING 中暂停 PTZ 输出

    按键:
      SPACE: 切换 IDLE ↔ TRACKING
      i:     回到 IDLE
      q/ESC: 退出

    按 ptz-tracking-rules SKILL:
    - PD 控制（归一化误差，死区内归零强制静止）
    - q/ESC 使用 os._exit(0)

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
from geometry_msgs.msg import Vector3
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from .image_conversion import image_to_bgr

from .hand_detector import HandDetector


DEFAULT_NODE_NAME = "gesture_tracker_ptz_node"
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
    "ptz_smooth_alpha", "control_rate_hz", "image_processing_rate_hz",
    "publish_initial_ptz_command", "initial_publish_count", "initial_publish_interval_sec",
    "hand_detection_confidence",
    "print_ptz_angle", "print_interval_sec",
    "enable_rqt_dynamic_parameters",
})

RESTART_REQUIRED_PARAMETER_NAMES = frozenset({
    "image_topic", "ptz_command_topic", "window_name",
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


GESTURE_BY_FINGER_CODE = {
    (0, 0, 0, 0, 0): "FIST",
    (1, 0, 0, 0, 0): "Thumbs Up",
    (0, 1, 0, 0, 0): "Point",
    (0, 1, 1, 0, 0): "Scissors",
    (1, 1, 1, 1, 1): "Open Palm",
    (0, 1, 0, 1, 0): "Yo-Yo",
    (0, 1, 1, 1, 0): "Three",
    (1, 0, 1, 1, 1): "OK",
}


def gesture_name_from_fingers(fingers: List[int]) -> str:
    """Return the gesture name for the displayed five-finger code."""
    normalized_code = tuple(1 if int(value) != 0 else 0 for value in fingers)
    return GESTURE_BY_FINGER_CODE.get(normalized_code, "Unknown")


class GestureTrackerPtzNode(Node):

    def __init__(self) -> None:
        super().__init__(DEFAULT_NODE_NAME)
        self._image_lock = threading.Lock()
        self._gesture_lock = threading.Lock()

        # 最新帧缓存 / Latest frame cache.
        self._latest_frame: Optional[np.ndarray] = None
        self._latest_seq = 0
        self._processed_seq = 0

        # 状态：仅 IDLE / TRACKING
        self._state = STATE_IDLE

        # 手势共享数据
        self._has_hand = False
        self._finger_count = 0
        self._gesture_name = "None"
        self._finger_code = "N/A"
        self._target_x: Optional[float] = None
        self._target_y: Optional[float] = None

        # PTZ 角度
        self._h_deg = 0.0
        self._p_deg = 0.0

        # PD 历史
        self._prev_err_x = 0.0
        self._prev_err_y = 0.0
        self._prev_ctrl_t = 0.0
        self._last_sx = 0.0
        self._last_sy = 0.0

        # 显示 & FPS
        self._measured_fps = 0.0
        self._fps_count = 0
        self._fps_last_t = time.perf_counter()
        self._win_ready = False

        self._shutdown = False
        self._last_print_t = 0.0

        self._declare_params()
        self._load_params()

        self._h_deg = float(self._init_h_deg)
        self._p_deg = float(self._init_p_deg)

        self._hand_det = HandDetector(detectorCon=float(self._hand_conf))

        # 图像订阅（仅解码存帧，极轻量）
        self._img_sub = self.create_subscription(
            Image, self._img_topic, self._on_image, QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE),
        )
        # PTZ 发布
        self._ptz_pub = self.create_publisher(Vector3, self._ptz_topic, 10)

        # 图像处理定时器
        self._img_timer = self.create_timer(
            1.0 / max(float(self._img_rate), MINIMUM_TIMER_RATE_HZ),
            self._process_image,
        )
        # 云台控制定时器
        self._ctrl_timer = None
        self._rebuild_ctrl_timer(False)

        # 初始 PTZ 发布
        self._init_ptz_timer = None
        self._init_ptz_remain = 0
        if self._pub_init_ptz:
            self._start_init_ptz()

        self._param_cb = self.add_on_set_parameters_callback(self._on_param_update)

        self.get_logger().info(
            f"Gesture tracker PTZ started. "
            f"SPACE=toggle(now:{self._state}), i=idle, q=quit. "
            f"ctrl={self._ctrl_rate:.0f}Hz, img={self._img_rate:.0f}Hz"
        )

    # ── 参数 ─────────────────────────────────────────────────

    def _declare_params(self) -> None:
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("ptz_command_topic", "/cmd_ptz_angle")
        self.declare_parameter("window_name", "gesture_tracker_ptz")
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
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("image_processing_rate_hz", 15.0)
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
        self._ctrl_rate = float(self.get_parameter("control_rate_hz").value)
        self._img_rate = float(self.get_parameter("image_processing_rate_hz").value)
        self._pub_init_ptz = to_bool(self.get_parameter("publish_initial_ptz_command").value, True)
        self._init_ptz_n = int(self.get_parameter("initial_publish_count").value)
        self._init_ptz_int = float(self.get_parameter("initial_publish_interval_sec").value)
        self._hand_conf = float(self.get_parameter("hand_detection_confidence").value)
        self._print_ptz = to_bool(self.get_parameter("print_ptz_angle").value, False)
        self._print_int = float(self.get_parameter("print_interval_sec").value)
        self._rqt = to_bool(self.get_parameter("enable_rqt_dynamic_parameters").value, True)

    # ── 图像回调（仅解码存帧）──────────────────────────────

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

    # ── 图像处理定时器 ─────────────────────────────────────

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

        # MediaPipe 检测 → 仅更新共享数据，不改变状态
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
        if k != 255:
            if k in (ord("q"), ord("Q"), KEY_CODE_ESCAPE):
                self._shutdown = True
                try:
                    self.get_logger().info("Force exiting.")
                except Exception:
                    pass
                os._exit(0)  # pylint: disable=protected-access
            elif k in (ord("i"), ord("I")):
                self._state = STATE_IDLE
                self._reset_pd()
                self.get_logger().info("IDLE")
            elif k == KEY_CODE_SPACE:
                if self._state == STATE_TRACKING:
                    self._state = STATE_IDLE
                    self._reset_pd()
                    self.get_logger().info("IDLE")
                else:
                    self._state = STATE_TRACKING
                    self._reset_pd()
                    self.get_logger().info("TRACKING")

    # ── 手势检测（仅写共享数据，不碰状态）─────────────────

    def _detect(self, frame: np.ndarray) -> None:
        try:
            self._hand_det.findHands(frame, draw=True)
        except TypeError:
            self._hand_det.findHands(frame)

        if len(self._hand_det.lmList) == 0:
            with self._gesture_lock:
                self._has_hand = False
                self._target_x = None
                self._target_y = None
                self._finger_count = 0
                self._finger_code = "N/A"
                self._gesture_name = "None"
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

        gn = gesture_name_from_fingers(nf)

        cv.circle(frame, (int(mx), int(my)), 8, (255, 0, 255), -1)
        cv.circle(frame, (int(mx), int(my)), 10, (255, 255, 0), 2)

        with self._gesture_lock:
            self._has_hand = True
            self._target_x = float(mx)
            self._target_y = float(my)
            self._finger_count = fc
            self._finger_code = fcode
            self._gesture_name = gn

    # ── 云台控制定时器（TRACKING 且 有手+非拳头 → PD）───

    def _rebuild_ctrl_timer(self, log: bool) -> None:
        p = 1.0 / max(float(self._ctrl_rate), MINIMUM_TIMER_RATE_HZ)
        if self._ctrl_timer is not None:
            self.destroy_timer(self._ctrl_timer)
        self._ctrl_timer = self.create_timer(p, self._ctrl_tick)
        if log:
            self.get_logger().info(f"Ctrl timer: {self._ctrl_rate:.1f}Hz")

    def _start_init_ptz(self) -> None:
        self._init_ptz_remain = max(0, int(self._init_ptz_n))
        if self._init_ptz_remain <= 0:
            return
        self._pub_ptz()
        self._init_ptz_remain -= 1
        if self._init_ptz_remain <= 0:
            return
        self._init_ptz_timer = self.create_timer(
            max(float(self._init_ptz_int), 0.02), self._init_ptz_tick,
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

    def _reset_pd(self) -> None:
        self._prev_err_x = 0.0
        self._prev_err_y = 0.0
        self._prev_ctrl_t = 0.0
        self._last_sx = 0.0
        self._last_sy = 0.0

    def _ctrl_tick(self) -> None:
        """TRACKING 状态下：有手+非拳头→PD输出；无手/拳头→仅 return 不改变状态。"""
        if self._state != STATE_TRACKING:
            return

        with self._gesture_lock:
            tx = self._target_x
            ty = self._target_y
            has_hand = self._has_hand
            fc = self._finger_count

        # 无手 或 拳头 或 无效目标 → 不输出，不改变状态
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
        dt = n - self._prev_ctrl_t if self._prev_ctrl_t > 0.0 else 1.0 / max(float(self._ctrl_rate), MINIMUM_TIMER_RATE_HZ)
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
        self._log_ctrl(pex, pey)

    def _log_ctrl(self, he: float, ve: float) -> None:
        if not self._print_ptz:
            return
        n = time.monotonic()
        if n - self._last_print_t >= max(float(self._print_int), 0.01):
            self._last_print_t = n
            self.get_logger().info(f"PTZ err=({he:.1f},{ve:.1f}) s1={self._h_deg:.1f} s2={self._p_deg:.1f}")

    # ── 显示 ───────────────────────────────────────────────

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
        cv.rectangle(frame,
                     (int(cx - self._db_x), int(cy - self._db_y)),
                     (int(cx + self._db_x), int(cy + self._db_y)), (0, 180, 255), 1)

    def _draw_overlay(self, frame: np.ndarray) -> None:
        with self._gesture_lock:
            tx, ty = self._target_x, self._target_y
            fc = self._finger_count
            fcode = self._finger_code
            gn = self._gesture_name
            has_hand = self._has_hand
        cx = self._proc_w * 0.5; cy = self._proc_h * 0.5
        he = (tx - cx) if has_hand and tx is not None else 0.0
        ve = (ty - cy) if has_hand and ty is not None else 0.0
        tracking_status = "NO HAND"
        if has_hand:
            tracking_status = "FIST" if fc == 0 else "TRACK"

        lines = [
            f"State:{self._state} FPS:{int(self._measured_fps)}",
            f"Gesture:{gn} Finger:{fcode} Count:{fc} {tracking_status}",
            f"Target:(LM5) Error H:{he:.1f} V:{ve:.1f}",
            f"PD kp:{self._kp:.2f} kd:{self._kd:.2f} dead:({self._db_x:.0f},{self._db_y:.0f}) step:{self._max_step:.1f}",
            f"PTZ s1:{self._h_deg:.1f} s2:{self._p_deg:.1f} hz:{self._ctrl_rate:.0f}",
        ]
        if self._show_legend:
            lines.append("SPACE:toggle | i:idle | q:quit")
        if self._show_bg:
            bh = self._line_h * len(lines) + 8
            cv.rectangle(frame, (0, 0), (self._proc_w, min(self._proc_h, bh)), (0, 0, 0), -1)
        for i, t in enumerate(lines):
            cv.putText(frame, t, (6, 15 + i * self._line_h), cv.FONT_HERSHEY_SIMPLEX, self._font_scale, (0, 255, 255), 1, cv.LINE_AA)

    def _update_fps(self) -> None:
        self._fps_count += 1
        n = time.perf_counter()
        e = n - self._fps_last_t
        if e >= 1.0:
            self._measured_fps = self._fps_count / e
            self._fps_count = 0
            self._fps_last_t = n

    # ── 动态参数更新 ──────────────────────────────────────

    def _on_param_update(self, params: List[Parameter]) -> SetParametersResult:
        if not params:
            return SetParametersResult(successful=True)
        names = {p.name for p in params}
        bad = names - DYNAMIC_PARAMETER_NAMES - RESTART_REQUIRED_PARAMETER_NAMES
        if bad:
            return SetParametersResult(successful=False, reason=f"Unsupported: {sorted(bad)}")
        if names & RESTART_REQUIRED_PARAMETER_NAMES:
            return SetParametersResult(successful=False, reason=f"Restart required: {sorted(names & RESTART_REQUIRED_PARAMETER_NAMES)}")
        if not self._rqt and names != {"enable_rqt_dynamic_parameters"}:
            return SetParametersResult(successful=False, reason="Disabled")

        old_ctrl = self._ctrl_rate
        old_img = self._img_rate
        old_debug = self._show_win
        old_size = (self._disp_w, self._disp_h)

        vals = self._merge_vals(params)
        err = self._check_vals(vals)
        if err:
            return SetParametersResult(successful=False, reason=err)
        self._apply_vals(vals)

        if abs(self._ctrl_rate - old_ctrl) > 1e-9:
            self._rebuild_ctrl_timer(True)
        if abs(self._img_rate - old_img) > 1e-9:
            self.destroy_timer(self._img_timer)
            self._img_timer = self.create_timer(
                1.0 / max(float(self._img_rate), MINIMUM_TIMER_RATE_HZ), self._process_image,
            )
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
            "ptz_smooth_alpha": self._ptz_alpha, "control_rate_hz": self._ctrl_rate,
            "image_processing_rate_hz": self._img_rate,
            "hand_detection_confidence": self._hand_conf,
            "print_ptz_angle": self._print_ptz, "print_interval_sec": self._print_int,
            "enable_rqt_dynamic_parameters": self._rqt,
        }
        for p in params:
            if p.type_ != Parameter.Type.NOT_SET:
                d[p.name] = p.value
        return d

    def _check_vals(self, v: Dict[str, Any]) -> str:
        try:
            if float(v["horizontal_min_deg"]) >= float(v["horizontal_max_deg"]): return "h_min < h_max"
            if float(v["pitch_min_deg"]) >= float(v["pitch_max_deg"]): return "p_min < p_max"
            if float(v["deadband_x"]) < 0: return "db_x >= 0"
            if float(v["deadband_y"]) < 0: return "db_y >= 0"
            if float(v["pd_kp"]) < 0: return "kp >= 0"
            if float(v["pd_kd"]) < 0: return "kd >= 0"
            if not (0.0 <= float(v["ptz_step_smooth_alpha"]) <= 1.0): return "step_alpha in [0,1]"
            if float(v["max_ptz_step"]) <= 0: return "max_step > 0"
            if not (0.01 <= float(v["ptz_smooth_alpha"]) <= 1.0): return "ptz_alpha in [0.01,1]"
            if float(v["control_rate_hz"]) < 1.0: return "ctrl >= 1"
            if float(v["image_processing_rate_hz"]) < 1.0: return "img_rate >= 1"
            if int(v["processing_width"]) <= 0: return "proc_w > 0"
            if int(v["processing_height"]) <= 0: return "proc_h > 0"
            if int(v["display_width"]) <= 0: return "disp_w > 0"
            if int(v["display_height"]) <= 0: return "disp_h > 0"
        except Exception as e:
            return str(e)
        return ""

    def _apply_vals(self, v: Dict[str, Any]) -> None:
        self._show_win = to_bool(v["show_debug_window"], True)
        self._proc_w = int(v["processing_width"]); self._proc_h = int(v["processing_height"])
        self._disp_w = int(v["display_width"]); self._disp_h = int(v["display_height"])
        self._font_scale = float(v["overlay_font_scale"]); self._line_h = int(v["overlay_line_height"])
        self._show_bg = to_bool(v["overlay_show_background"], False)
        self._show_legend = to_bool(v["overlay_show_legend"], True)
        self._flip_h = to_bool(v["enable_horizontal_flip"], False)
        self._flip_v = to_bool(v["enable_vertical_flip"], False)
        self._rot = str(v["rotate_image_code"]).lower()
        self._h_min = float(v["horizontal_min_deg"]); self._h_max = float(v["horizontal_max_deg"])
        self._p_min = float(v["pitch_min_deg"]); self._p_max = float(v["pitch_max_deg"])
        self._rev_h = to_bool(v["reverse_horizontal_direction"], False)
        self._rev_p = to_bool(v["reverse_pitch_direction"], True)
        self._db_x = float(v["deadband_x"]); self._db_y = float(v["deadband_y"])
        self._kp = max(float(v["pd_kp"]), 0.0); self._kd = max(float(v["pd_kd"]), 0.0)
        self._step_alpha = clamp(float(v["ptz_step_smooth_alpha"]), 0.0, 1.0)
        self._max_step = float(v["max_ptz_step"])
        self._ptz_alpha = clamp(float(v["ptz_smooth_alpha"]), 0.01, 1.0)
        self._ctrl_rate = float(v["control_rate_hz"])
        self._img_rate = float(v["image_processing_rate_hz"])
        self._hand_conf = float(v["hand_detection_confidence"])
        self._print_ptz = to_bool(v["print_ptz_angle"], False)
        self._rqt = to_bool(v["enable_rqt_dynamic_parameters"], True)
        self._h_deg = clamp(self._h_deg, self._h_min, self._h_max)
        self._p_deg = clamp(self._p_deg, self._p_min, self._p_max)
        self._reset_pd()

    def _sync_win(self, prev_debug: bool, prev_size: Tuple[int, int]) -> None:
        try:
            chg = prev_size != (self._disp_w, self._disp_h)
            if self._show_win and not prev_debug:
                self._ensure_win()
            elif not self._show_win and prev_debug:
                cv.destroyWindow(self._win_name)
            elif self._show_win and chg:
                cv.resizeWindow(self._win_name, int(self._disp_w), int(self._disp_h))
        except Exception:
            pass

    def destroy_node(self) -> bool:
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = GestureTrackerPtzNode()
    try:
        rclpy.spin(node)
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
