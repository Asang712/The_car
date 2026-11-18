#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
kcf_tracker_ptz_node.py

中文说明:
    ROS2 Humble KCF 目标追踪云台控制节点。
    KCF 追踪器参照 monoTracker/track_common（refer/code/kfc），
    云台控制与退出按 ptz-tracking-rules SKILL。

    交互: 鼠标框选 ROI → SPACE 开始追踪 → 连续 PD 锁定目标中心。
          SPACE 暂停/恢复, i 仅识别, r 重新框选, q/ESC 退出。
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


# ── KCF 追踪器包装（参照 track_common.py）───────────────────
class KcfTracker:
    """OpenCV 内置追踪器包装 / OpenCV built-in tracker wrapper."""

    def __init__(self, tracker_type: str = "KCF"):
        self._type = tracker_type.upper()
        self._tracker = None
        self._working = False
        self._create()

    def _create(self) -> None:
        t = self._type
        if t == "BOOSTING":       self._tracker = cv.TrackerBoosting_create()
        elif t == "MIL":          self._tracker = cv.TrackerMIL_create()
        elif t == "KCF":          self._tracker = self._try_create("TrackerKCF_create")
        elif t == "TLD":          self._tracker = cv.TrackerTLD_create()
        elif t == "MEDIANFLOW":   self._tracker = cv.TrackerMedianFlow_create()
        elif t == "CSRT":         self._tracker = self._try_create("TrackerCSRT_create")
        elif t == "MOSSE":        self._tracker = self._try_create("legacy.TrackerMOSSE_create")
        if self._tracker is None:
            self._tracker = self._try_create(f"legacy.Tracker{t}_create")
        if self._tracker is None:
            raise RuntimeError(f"Tracker {t} not available")

    @staticmethod
    def _try_create(path: str):
        try:
            parts = path.split(".")
            obj = cv
            for p in parts: obj = getattr(obj, p)
            return obj()
        except Exception:
            return None

    def init(self, frame: np.ndarray, roi: Tuple[int, int, int, int]) -> bool:
        try:
            ok = self._tracker.init(frame, roi)
            self._working = ok if ok is not None else True
            return self._working
        except Exception:
            self._working = False
            return False

    def update(self, frame: np.ndarray) -> Tuple[bool, Tuple[int, int, int, int]]:
        if not self._working:
            return False, (0, 0, 0, 0)
        try:
            ok, bbox = self._tracker.update(frame)
            if ok:
                self._working = True
                return True, tuple(int(v) for v in bbox)
            else:
                self._working = False
                return False, (0, 0, 0, 0)
        except Exception:
            self._working = False
            return False, (0, 0, 0, 0)

    @property
    def is_working(self) -> bool:
        return self._working


# ── 常量 ─────────────────────────────────────────────────────
DEFAULT_NODE_NAME = "kcf_tracker_ptz_node"

STATE_INIT = "init"
STATE_IDENTIFY = "identify"
STATE_TRACKING = "tracking"

KEY_CODE_SPACE = 32
KEY_CODE_ESCAPE = 27
MINIMUM_TIMER_RATE_HZ = 1.0

DYNAMIC_PARAMETER_NAMES = frozenset({
    "show_debug_window", "window_width", "window_height",
    "overlay_font_scale", "overlay_line_height",
    "enable_horizontal_flip", "enable_vertical_flip", "rotate_image_code",
    "horizontal_min_deg", "horizontal_max_deg",
    "pitch_min_deg", "pitch_max_deg",
    "reverse_horizontal_direction", "reverse_pitch_direction",
    "pd_kp", "pd_kd", "deadband_x", "deadband_y",
    "ptz_step_smooth_alpha", "max_ptz_step", "ptz_smooth_alpha",
    "control_rate_hz", "image_processing_rate_hz",
    "publish_initial_ptz_command", "initial_publish_count", "initial_publish_interval_sec",
    "tracker_type", "img_flip",
    "print_ptz_angle", "print_interval_sec",
    "enable_rqt_dynamic_parameters",
})

RESTART_REQUIRED_PARAMETER_NAMES = frozenset({
    "image_topic", "ptz_command_topic", "window_name",
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


# ── 节点 ─────────────────────────────────────────────────────
class KcfTrackerPtzNode(Node):

    def __init__(self) -> None:
        super().__init__(DEFAULT_NODE_NAME)
        self._img_lock = threading.Lock()
        self._tgt_lock = threading.Lock()

        self._latest_frame: Optional[np.ndarray] = None
        self._latest_seq = 0
        self._processed_seq = 0

        # 参照 monoTracker 状态
        self._state = STATE_INIT
        self._select_flags = False
        self._mouse_xy: Tuple[int, int] = (0, 0)
        self._roi_init: Tuple[int, int, int, int] = (0, 0, 0, 0)

        # KCF 追踪器
        self._tracker: Optional[KcfTracker] = None
        self._tracker_ready = False

        # 目标
        self._has_target = False
        self._target_x: Optional[float] = None
        self._target_y: Optional[float] = None

        # PTZ + PD
        self._h_deg = 0.0
        self._p_deg = 0.0
        self._prev_err_x = 0.0
        self._prev_err_y = 0.0
        self._prev_ctrl_t = 0.0
        self._last_sx = 0.0
        self._last_sy = 0.0

        self._fps = 0.0; self._fps_n = 0; self._fps_t = time.perf_counter()
        self._win_ok = False; self._shutdown = False; self._last_print_t = 0.0

        self._declare_params()
        self._load_params()
        self._h_deg = self._init_h_deg; self._p_deg = self._init_p_deg

        self._sub = self.create_subscription(
            Image, self._img_topic, self._on_image, QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE),
        )
        self._pub = self.create_publisher(Vector3, self._ptz_topic, 10)

        self._img_timer = self.create_timer(
            1.0 / max(self._img_rate, MINIMUM_TIMER_RATE_HZ), self._process_image,
        )
        self._ctrl_timer = None; self._rebuild_ctrl_timer(False)
        self._key_timer = None
        if self._show_win:
            self._init_win()
            self._key_timer = self.create_timer(0.05, self._poll_keys)

        self._init_ptz_timer = None; self._init_ptz_remain = 0
        if self._pub_init: self._start_init_ptz()

        # 检查是否提供了自动 ROI 初始化坐标
        if (self._auto_roi_x1 >= 0 and self._auto_roi_y1 >= 0 and
            self._auto_roi_x2 >= 0 and self._auto_roi_y2 >= 0):
            # 延迟初始化追踪器，等待第一帧图像到达
            self._auto_init_pending = True
            self.get_logger().info(
                f"自动 ROI 模式: 等待图像后将初始化追踪 "
                f"({self._auto_roi_x1},{self._auto_roi_y1})-({self._auto_roi_x2},{self._auto_roi_y2})")
        else:
            self._auto_init_pending = False

        self._param_cb = self.add_on_set_parameters_callback(self._on_param_update)
        self.get_logger().info(f"KCF tracker started. Drag ROI → SPACE to track. ctrl={self._ctrl_rate:.1f}Hz")

    # ── 参数（Kp/Ki/Kd/scale 参照 monoTracker）───────────────
    def _declare_params(self) -> None:
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("ptz_command_topic", "/cmd_ptz_angle")
        self.declare_parameter("window_name", "kcf_tracker_ptz")
        self.declare_parameter("show_debug_window", True)
        self.declare_parameter("image_width", 960)
        self.declare_parameter("image_height", 720)
        self.declare_parameter("window_width", 960)
        self.declare_parameter("window_height", 720)
        self.declare_parameter("overlay_font_scale", 0.9)
        self.declare_parameter("overlay_line_height", 20)
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
        # PD 控制参数（SKILL）
        self.declare_parameter("pd_kp", 0.7)
        self.declare_parameter("pd_kd", 0.25)
        self.declare_parameter("deadband_x", 20.0)
        self.declare_parameter("deadband_y", 20.0)
        self.declare_parameter("ptz_step_smooth_alpha", 0.35)
        self.declare_parameter("max_ptz_step", 1.2)
        self.declare_parameter("ptz_smooth_alpha", 0.85)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("image_processing_rate_hz", 15.0)
        self.declare_parameter("publish_initial_ptz_command", True)
        self.declare_parameter("initial_publish_count", 20)
        self.declare_parameter("initial_publish_interval_sec", 0.2)
        self.declare_parameter("tracker_type", "KCF")
        self.declare_parameter("img_flip", False)
        self.declare_parameter("print_ptz_angle", False)
        self.declare_parameter("print_interval_sec", 0.5)
        self.declare_parameter("enable_rqt_dynamic_parameters", True)
        # 自动 ROI 初始化参数（用于 Dify 视觉检测后自动启动追踪）
        self.declare_parameter("auto_init_roi_x1", -1)
        self.declare_parameter("auto_init_roi_y1", -1)
        self.declare_parameter("auto_init_roi_x2", -1)
        self.declare_parameter("auto_init_roi_y2", -1)

    def _load_params(self) -> None:
        self._img_topic = str(self.get_parameter("image_topic").value)
        self._ptz_topic = str(self.get_parameter("ptz_command_topic").value)
        self._win_name = str(self.get_parameter("window_name").value)
        self._show_win = to_bool(self.get_parameter("show_debug_window").value, True)
        self._img_w = int(self.get_parameter("image_width").value)
        self._img_h = int(self.get_parameter("image_height").value)
        self._win_w = int(self.get_parameter("window_width").value)
        self._win_h = int(self.get_parameter("window_height").value)
        self._font = float(self.get_parameter("overlay_font_scale").value)
        self._lh = int(self.get_parameter("overlay_line_height").value)
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
        self._kp = max(float(self.get_parameter("pd_kp").value), 0.0)
        self._kd = max(float(self.get_parameter("pd_kd").value), 0.0)
        self._db_x = float(self.get_parameter("deadband_x").value)
        self._db_y = float(self.get_parameter("deadband_y").value)
        self._step_a = clamp(float(self.get_parameter("ptz_step_smooth_alpha").value), 0.0, 1.0)
        self._max_s = float(self.get_parameter("max_ptz_step").value)
        self._ptz_a = clamp(float(self.get_parameter("ptz_smooth_alpha").value), 0.01, 1.0)
        self._ctrl_rate = float(self.get_parameter("control_rate_hz").value)
        self._img_rate = float(self.get_parameter("image_processing_rate_hz").value)
        self._pub_init = to_bool(self.get_parameter("publish_initial_ptz_command").value, True)
        self._init_n = int(self.get_parameter("initial_publish_count").value)
        self._init_int = float(self.get_parameter("initial_publish_interval_sec").value)
        self._trk_type = str(self.get_parameter("tracker_type").value)
        self._img_flip = to_bool(self.get_parameter("img_flip").value, False)
        self._prn = to_bool(self.get_parameter("print_ptz_angle").value, False)
        self._prn_int = float(self.get_parameter("print_interval_sec").value)
        self._rqt = to_bool(self.get_parameter("enable_rqt_dynamic_parameters").value, True)
        # 自动 ROI 初始化参数
        self._auto_roi_x1 = int(self.get_parameter("auto_init_roi_x1").value)
        self._auto_roi_y1 = int(self.get_parameter("auto_init_roi_y1").value)
        self._auto_roi_x2 = int(self.get_parameter("auto_init_roi_x2").value)
        self._auto_roi_y2 = int(self.get_parameter("auto_init_roi_y2").value)

    # ── 图像回调 ────────────────────────────────────────────
    def _on_image(self, msg: Image) -> None:
        try:
            frame = image_to_bgr(msg)
            if frame is None: return
            if self._img_flip or self._flip_h: frame = cv.flip(frame, 1)
            if self._flip_v: frame = cv.flip(frame, 0)
            if self._rot == "90_clockwise": frame = cv.rotate(frame, cv.ROTATE_90_CLOCKWISE)
            elif self._rot == "90_counterclockwise": frame = cv.rotate(frame, cv.ROTATE_90_COUNTERCLOCKWISE)
            elif self._rot == "180": frame = cv.rotate(frame, cv.ROTATE_180)
            if self._img_w > 0 and self._img_h > 0:
                frame = cv.resize(frame, (self._img_w, self._img_h))
            with self._img_lock:
                self._latest_frame = frame; self._latest_seq += 1
        except Exception:
            pass

    # ── 图像处理（参照 monoTracker.process + on_timer）─────
    def _process_image(self) -> None:
        with self._img_lock:
            if self._latest_frame is None: return
            seq = self._latest_seq
        if seq == self._processed_seq: return
        self._processed_seq = seq

        with self._img_lock:
            frame = self._latest_frame.copy()

        # 自动 ROI 初始化（仅执行一次）
        if self._auto_init_pending:
            self._auto_init_pending = False
            self._roi_init = (self._auto_roi_x1, self._auto_roi_y1,
                              self._auto_roi_x2, self._auto_roi_y2)
            self._init_tracker(frame_hint=frame)
            self.get_logger().info(
                f"自动初始化追踪器完成: ROI=({self._auto_roi_x1},{self._auto_roi_y1})-"
                f"({self._auto_roi_x2},{self._auto_roi_y2})")

        result = self._track(frame)  # KCF 更新

        if self._show_win:
            self._draw_overlay(frame, result)
            cv.imshow(self._win_name, frame)

        self._fps_n += 1; n = time.perf_counter()
        e = n - self._fps_t
        if e >= 1.0: self._fps = self._fps_n / e; self._fps_n = 0; self._fps_t = n

    def _track(self, frame: np.ndarray) -> Tuple[bool, float, float]:
        """KCF 追踪更新，返回 (成功, cx, cy)。参照 monoTracker.process。"""
        # 框选中的矩形
        if self._select_flags and hasattr(self, '_ms_start') and hasattr(self, '_ms_end'):
            cv.rectangle(frame, self._ms_start, self._ms_end, (0, 255, 0), 2)

        if self._state == STATE_INIT:
            return False, 0.0, 0.0

        if not self._tracker_ready or self._tracker is None:
            return False, 0.0, 0.0

        ok, bbox = self._tracker.update(frame)
        if not ok:
            cv.putText(frame, "Tracking failure detected", (100, 80),
                       cv.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
            with self._tgt_lock: self._has_target = False
            return False, 0.0, 0.0

        x, y, w, h = bbox
        cx, cy = x + w / 2.0, y + h / 2.0
        cv.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2, 1)

        with self._tgt_lock:
            self._has_target = True
            self._target_x = float(cx); self._target_y = float(cy)
        return True, float(cx), float(cy)

    # ── 鼠标（参照 monoTracker.onMouse）─────────────────────
    def _init_win(self) -> None:
        cv.namedWindow(self._win_name, cv.WINDOW_NORMAL)
        cv.resizeWindow(self._win_name, self._win_w, self._win_h)
        cv.setMouseCallback(self._win_name, self._on_mouse)

    def _on_mouse(self, event, x, y, flags, param):
        if event == cv.EVENT_LBUTTONDOWN:
            self._state = STATE_INIT; self._select_flags = True
            self._mouse_xy = (x, y); self._ms_start = (x, y); self._ms_end = (x, y)
        elif event == cv.EVENT_MOUSEMOVE and self._select_flags:
            self._ms_start = self._mouse_xy; self._ms_end = (x, y)
            self._roi_init = (self._ms_start[0], self._ms_start[1], self._ms_end[0], self._ms_end[1])
        elif event == cv.EVENT_LBUTTONUP:
            self._select_flags = False
            self._init_tracker(frame_hint=None)  # 从 _roi_init 初始化

    def _init_tracker(self, frame_hint=None) -> None:
        x1, y1, x2, y2 = self._roi_init
        if x1 == x2 or y1 == y2: return
        rx = min(x1, x2); ry = min(y1, y2); rw = abs(x2 - x1); rh = abs(y2 - y1)
        if rw < 10 or rh < 10: return
        rx = int(clamp(rx, 0, self._img_w - 1)); ry = int(clamp(ry, 0, self._img_h - 1))
        rw = int(min(rw, self._img_w - rx)); rh = int(min(rh, self._img_h - ry))
        roi = (rx, ry, rw, rh)

        with self._img_lock:
            frame = self._latest_frame
            if frame is None: return
            init_frame = frame.copy()

        try:
            self._tracker = KcfTracker(self._trk_type)
            self._tracker.init(init_frame, roi)
            self._tracker_ready = True
            self._reset_pd()
            self._state = STATE_TRACKING
            self.get_logger().info(f"Tracker ready. ROI={roi}")
        except Exception as e:
            self.get_logger().error(f"Tracker init failed: {e}")
            self._tracker = None; self._tracker_ready = False

    # ── 键盘（参照 SKILL: os._exit(0)）─────────────────────
    def _poll_keys(self) -> None:
        try: k = cv.waitKey(1) & 0xFF
        except Exception: return
        if k in (255, -1): return
        if k in (ord("q"), ord("Q"), KEY_CODE_ESCAPE):
            self._shutdown = True
            try: self.get_logger().info("Force exiting.")
            except Exception: pass
            os._exit(0)
        elif k == KEY_CODE_SPACE:
            if self._state == STATE_TRACKING:
                self._state = STATE_IDENTIFY; self._reset_pd()
                self.get_logger().info("Paused (identify).")
            elif self._tracker_ready:
                self._state = STATE_TRACKING; self._reset_pd()
                self.get_logger().info("Tracking.")
        elif k in (ord("i"), ord("I")):
            self._state = STATE_IDENTIFY; self._reset_pd()
            self.get_logger().info("Identify mode.")
        elif k in (ord("r"), ord("R")):
            self._tracker = None; self._tracker_ready = False
            self._state = STATE_INIT; self._reset_pd()
            self.get_logger().info("Reset. Drag to select target.")

    # ── PTZ PD 控制（SKILL: 归一化误差 + 死区 + 平滑 + 限幅）───
    def _rebuild_ctrl_timer(self, log: bool) -> None:
        p = 1.0 / max(self._ctrl_rate, MINIMUM_TIMER_RATE_HZ)
        if self._ctrl_timer is not None: self.destroy_timer(self._ctrl_timer)
        self._ctrl_timer = self.create_timer(p, self._ctrl_tick)
        if log: self.get_logger().info(f"Ctrl timer: {self._ctrl_rate:.1f}Hz")

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
        self._pub.publish(m)

    def _reset_pd(self) -> None:
        self._prev_err_x = 0.0; self._prev_err_y = 0.0
        self._prev_ctrl_t = 0.0; self._last_sx = 0.0; self._last_sy = 0.0

    def _ctrl_tick(self) -> None:
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
        dt = n - self._prev_ctrl_t if self._prev_ctrl_t > 0.0 else 1.0 / max(self._ctrl_rate, MINIMUM_TIMER_RATE_HZ)
        dt = max(dt, 1e-3)
        nex, ney = pex / icx, pey / icy
        dx, dy = (nex - self._prev_err_x) / dt, (ney - self._prev_err_y) / dt

        rx = clamp((self._kp * nex + self._kd * dx) * self._max_s, -self._max_s, self._max_s)
        ry = clamp((self._kp * ney + self._kd * dy) * self._max_s, -self._max_s, self._max_s)
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
        self._pub_ptz()

    # ── 显示 ────────────────────────────────────────────────
    def _draw_overlay(self, frame: np.ndarray, result: Tuple[bool, float, float]) -> None:
        ok, cx, cy = result
        iw, ih = frame.shape[1], frame.shape[0]
        icx, icy = iw // 2, ih // 2
        cv.line(frame, (icx, 0), (icx, ih), (255, 255, 0), 1)
        cv.line(frame, (0, icy), (iw, icy), (255, 255, 0), 1)
        cv.rectangle(frame, (int(icx - self._db_x), int(icy - self._db_y)),
                     (int(icx + self._db_x), int(icy + self._db_y)), (0, 180, 255), 1)

        tgt = f"Target:{int(cx)},{int(cy)}" if ok else "Target:None"
        fps_txt = f"FPS : {int(self._fps)}"
        ptz_txt = f"s1:{self._h_deg:.1f} s2:{self._p_deg:.1f}"
        pd_txt = f"kp:{self._kp:.2f} kd:{self._kd:.2f}"
        lines = [fps_txt, tgt, ptz_txt, pd_txt,
                 f"State:{self._state} CtrlHz:{self._ctrl_rate:.0f}",
                 "SPACE:track | i:identify | r:reset | q:quit"]
        for i, t in enumerate(lines):
            cv.putText(frame, t, (20, 30 + i * self._lh), cv.FONT_HERSHEY_SIMPLEX, self._font / 2, (0, 0, 255), 1)

    # ── 动态参数更新 ───────────────────────────────────────
    def _on_param_update(self, params: List[Parameter]) -> SetParametersResult:
        if not params: return SetParametersResult(successful=True)
        names = {p.name for p in params}
        bad = names - DYNAMIC_PARAMETER_NAMES - RESTART_REQUIRED_PARAMETER_NAMES
        if bad: return SetParametersResult(successful=False, reason=f"Unsupported: {sorted(bad)}")
        if names & RESTART_REQUIRED_PARAMETER_NAMES: return SetParametersResult(successful=False, reason="Restart required")
        if not self._rqt and names != {"enable_rqt_dynamic_parameters"}: return SetParametersResult(successful=False, reason="Disabled")
        old_ctrl, old_img = self._ctrl_rate, self._img_rate
        old_debug, old_size = self._show_win, (self._win_w, self._win_h)
        v = self._merge(params)
        err = self._check(v)
        if err: return SetParametersResult(successful=False, reason=err)
        self._apply(v)
        if abs(self._ctrl_rate - old_ctrl) > 1e-9: self._rebuild_ctrl_timer(True)
        if abs(self._img_rate - old_img) > 1e-9:
            self.destroy_timer(self._img_timer)
            self._img_timer = self.create_timer(1.0 / max(self._img_rate, MINIMUM_TIMER_RATE_HZ), self._process_image)
        self._sync_win(old_debug, old_size)
        return SetParametersResult(successful=True)

    def _merge(self, params):
        d = dict(show_debug_window=self._show_win, window_width=self._win_w, window_height=self._win_h,
                 overlay_font_scale=self._font, overlay_line_height=self._lh,
                 enable_horizontal_flip=self._flip_h, enable_vertical_flip=self._flip_v, rotate_image_code=self._rot,
                 horizontal_min_deg=self._h_min, horizontal_max_deg=self._h_max,
                 pitch_min_deg=self._p_min, pitch_max_deg=self._p_max,
                 reverse_horizontal_direction=self._rev_h, reverse_pitch_direction=self._rev_p,
                 pd_kp=self._kp, pd_kd=self._kd,
                 deadband_x=self._db_x, deadband_y=self._db_y,
                 ptz_step_smooth_alpha=self._step_a, max_ptz_step=self._max_s, ptz_smooth_alpha=self._ptz_a,
                 control_rate_hz=self._ctrl_rate, image_processing_rate_hz=self._img_rate,
                 tracker_type=self._trk_type, img_flip=self._img_flip,
                 print_ptz_angle=self._prn, print_interval_sec=self._prn_int,
                 enable_rqt_dynamic_parameters=self._rqt)
        for p in params:
            if p.type_ != Parameter.Type.NOT_SET: d[p.name] = p.value
        return d

    def _check(self, v):
        try:
            if float(v["pd_kp"]) < 0 or float(v["pd_kd"]) < 0: return "kp/kd >=0"
            if float(v["deadband_x"]) < 0 or float(v["deadband_y"]) < 0: return "db >=0"
            if not (0.0 <= float(v["ptz_step_smooth_alpha"]) <= 1.0): return "step_a in [0,1]"
            if float(v["max_ptz_step"]) <= 0: return "max_s >0"
        except Exception as e: return str(e)
        return ""

    def _apply(self, v):
        self._show_win = to_bool(v["show_debug_window"], True); self._win_w = int(v["window_width"]); self._win_h = int(v["window_height"])
        self._font = float(v["overlay_font_scale"]); self._lh = int(v["overlay_line_height"])
        self._flip_h = to_bool(v["enable_horizontal_flip"], False); self._flip_v = to_bool(v["enable_vertical_flip"], False)
        self._rot = str(v["rotate_image_code"]).lower()
        self._h_min = float(v["horizontal_min_deg"]); self._h_max = float(v["horizontal_max_deg"])
        self._p_min = float(v["pitch_min_deg"]); self._p_max = float(v["pitch_max_deg"])
        self._rev_h = to_bool(v["reverse_horizontal_direction"], False); self._rev_p = to_bool(v["reverse_pitch_direction"], True)
        self._kp = max(float(v["pd_kp"]), 0.0); self._kd = max(float(v["pd_kd"]), 0.0)
        self._db_x = float(v["deadband_x"]); self._db_y = float(v["deadband_y"])
        self._step_a = clamp(float(v["ptz_step_smooth_alpha"]), 0.0, 1.0); self._max_s = float(v["max_ptz_step"])
        self._ptz_a = clamp(float(v["ptz_smooth_alpha"]), 0.01, 1.0)
        self._ctrl_rate = float(v["control_rate_hz"]); self._img_rate = float(v["image_processing_rate_hz"])
        self._trk_type = str(v["tracker_type"]); self._img_flip = to_bool(v["img_flip"], False)
        self._prn = to_bool(v["print_ptz_angle"], False); self._rqt = to_bool(v["enable_rqt_dynamic_parameters"], True)
        self._h_deg = clamp(self._h_deg, self._h_min, self._h_max); self._p_deg = clamp(self._p_deg, self._p_min, self._p_max)
        self._reset_pd()

    def _sync_win(self, prev_debug, prev_size):
        try:
            chg = prev_size != (self._win_w, self._win_h)
            if self._show_win and not prev_debug: self._init_win()
            elif not self._show_win and prev_debug: cv.destroyWindow(self._win_name)
            elif self._show_win and chg: cv.resizeWindow(self._win_name, self._win_w, self._win_h)
        except Exception: pass

    def destroy_node(self) -> bool:
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KcfTrackerPtzNode()
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
