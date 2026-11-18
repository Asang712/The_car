#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
kcf_tracker_node.py

中文说明:
    ROS2 Humble KCF 目标追踪车身控制节点（No-PTZ 版）。
    KCF 追踪器参照 monoTracker/track_common，鼠标框选 ROI → SPACE 开始追踪，
    车身 PD 基于像素误差直接控制角速度使目标保持在画面中心。
    与 kcf_tracker_ptz_node 对应，去掉 PTZ 层，车身 100% 承担水平跟踪。

    状态机: INIT(鼠标框选) → TRACKING(KCF追踪+车身PD) ↔ IDENTIFY(暂停)
    交互: SPACE 切换追踪/暂停, i 暂停识别, r 重新框选, q/ESC 退出
"""
from __future__ import annotations

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
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from .image_conversion import image_to_bgr


# ── KCF 追踪器包装 ────────────────────────────────────────────
class KcfTracker:
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
        if not self._working: return False, (0, 0, 0, 0)
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
DEFAULT_NODE_NAME = "kcf_tracker_node"

STATE_INIT = "init"
STATE_IDENTIFY = "identify"
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
    "angular_kp", "angular_kd", "max_angular_z",
    "deadband_x_pixel", "reverse_body_direction",
    "image_processing_rate_hz", "body_control_rate",
    "tracker_type",
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


class KcfTrackerNode(Node):

    def __init__(self) -> None:
        super().__init__(DEFAULT_NODE_NAME)
        self._img_lock = threading.Lock()
        self._tgt_lock = threading.Lock()

        self._latest_frame: Optional[np.ndarray] = None
        self._latest_seq = 0
        self._processed_seq = 0

        self._state = STATE_INIT
        self._shutdown = False
        self._select_flags = False
        self._mouse_xy: Tuple[int, int] = (0, 0)
        self._roi_init: Tuple[int, int, int, int] = (0, 0, 0, 0)
        self._ms_start: Tuple[int, int] = (0, 0)
        self._ms_end: Tuple[int, int] = (0, 0)

        self._tracker: Optional[KcfTracker] = None
        self._tracker_ready = False

        self._has_target = False
        self._target_x: Optional[float] = None
        self._target_y: Optional[float] = None

        # 车身 PD
        self._prev_s1 = 0.0
        self._prev_body_t = 0.0
        self._last_body_az = 0.0

        self._fps = 0.0; self._fps_n = 0; self._fps_t = time.perf_counter()
        self._win_ok = False

        self._declare_params()
        self._load_params()

        best_effort_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._sub = self.create_subscription(
            Image, self._img_topic, self._on_image, best_effort_qos)
        self._cmd_pub = self.create_publisher(Twist, self._cmd_topic, best_effort_qos)

        self._img_timer = self.create_timer(
            1.0 / max(self._img_rate, MINIMUM_TIMER_RATE_HZ), self._process_image)
        self._body_timer = None; self._rebuild_body_timer(False)
        self._key_timer = None
        if self._show_win:
            self._init_win()
            self._key_timer = self.create_timer(0.05, self._poll_keys)

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
        self.get_logger().info(
            f"KCF tracker body started. Drag ROI → SPACE to track. body={self._body_rate:.1f}Hz")

    # ── 参数 ───────────────────────────────────────────────
    def _declare_params(self) -> None:
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("window_name", "kcf_tracker")
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
        self.declare_parameter("angular_kp", 1.5)
        self.declare_parameter("angular_kd", 0.45)
        self.declare_parameter("max_angular_z", 0.6)
        self.declare_parameter("deadband_x_pixel", 40.0)
        self.declare_parameter("reverse_body_direction", True)
        self.declare_parameter("image_processing_rate_hz", 15.0)
        self.declare_parameter("body_control_rate", 10.0)
        self.declare_parameter("tracker_type", "KCF")
        self.declare_parameter("show_deadband_lines", False)
        self.declare_parameter("show_center_line", False)
        self.declare_parameter("enable_rqt_dynamic_parameters", True)
        # 自动 ROI 初始化参数（用于 Dify 视觉检测后自动启动追踪）
        self.declare_parameter("auto_init_roi_x1", -1)
        self.declare_parameter("auto_init_roi_y1", -1)
        self.declare_parameter("auto_init_roi_x2", -1)
        self.declare_parameter("auto_init_roi_y2", -1)

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
        self._body_kp = max(float(self.get_parameter("angular_kp").value), 0.0)
        self._body_kd = max(float(self.get_parameter("angular_kd").value), 0.0)
        self._max_angular_z = max(float(self.get_parameter("max_angular_z").value), 0.0)
        self._db_x = float(self.get_parameter("deadband_x_pixel").value)
        self._rev_body = bool(self.get_parameter("reverse_body_direction").value)
        self._img_rate = max(float(self.get_parameter("image_processing_rate_hz").value), MINIMUM_TIMER_RATE_HZ)
        self._body_rate = max(float(self.get_parameter("body_control_rate").value), MINIMUM_TIMER_RATE_HZ)
        self._trk_type = str(self.get_parameter("tracker_type").value)
        self._show_deadband = bool(self.get_parameter("show_deadband_lines").value)
        self._show_center = bool(self.get_parameter("show_center_line").value)
        self._dyn = bool(self.get_parameter("enable_rqt_dynamic_parameters").value)
        # 自动 ROI 初始化参数
        self._auto_roi_x1 = int(self.get_parameter("auto_init_roi_x1").value)
        self._auto_roi_y1 = int(self.get_parameter("auto_init_roi_y1").value)
        self._auto_roi_x2 = int(self.get_parameter("auto_init_roi_x2").value)
        self._auto_roi_y2 = int(self.get_parameter("auto_init_roi_y2").value)

    # ── 图像回调 ────────────────────────────────────────────
    def _on_image(self, msg: Image) -> None:
        try:
            frame = image_to_bgr(msg)
        except Exception as exception:
            self.get_logger().warn(f"Image conversion failed: {exception}")
            return
        with self._img_lock:
            self._latest_frame = frame
            self._latest_seq += 1

    # ── 图像处理 ───────────────────────────────────────────
    def _process_image(self) -> None:
        if self._shutdown: return
        with self._img_lock:
            if self._latest_frame is None: return
            seq = self._latest_seq
        if seq == self._processed_seq: return
        self._processed_seq = seq

        with self._img_lock:
            frame = self._latest_frame.copy()
        frame = cv.resize(frame, (self._proc_w, self._proc_h))
        if self._flip_h: frame = cv.flip(frame, 1)
        if self._flip_v: frame = cv.flip(frame, 0)
        if self._rot == "90_clockwise": frame = cv.rotate(frame, cv.ROTATE_90_CLOCKWISE)
        elif self._rot == "90_counterclockwise": frame = cv.rotate(frame, cv.ROTATE_90_COUNTERCLOCKWISE)
        elif self._rot == "180": frame = cv.rotate(frame, cv.ROTATE_180)

        # 自动 ROI 初始化（仅执行一次）
        if self._auto_init_pending:
            self._auto_init_pending = False
            self._roi_init = (self._auto_roi_x1, self._auto_roi_y1,
                              self._auto_roi_x2, self._auto_roi_y2)
            self._init_tracker(frame)
            self.get_logger().info(
                f"自动初始化追踪器完成: ROI=({self._auto_roi_x1},{self._auto_roi_y1})-"
                f"({self._auto_roi_x2},{self._auto_roi_y2})")

        self._track(frame)
        self._draw_guides(frame)
        self._draw_overlay(frame)

        self._fps_n += 1; n = time.perf_counter()
        e = n - self._fps_t
        if e >= 1.0: self._fps = self._fps_n / e; self._fps_n = 0; self._fps_t = n

        if self._show_win:
            self._ensure_win()
            cv.imshow(self._win_name, cv.resize(frame, (self._disp_w, self._disp_h)))

    def _track(self, frame: np.ndarray) -> None:
        if self._select_flags:
            cv.rectangle(frame, self._ms_start, self._ms_end, (0, 255, 0), 2)

        if self._state == STATE_INIT: return
        if not self._tracker_ready or self._tracker is None:
            with self._tgt_lock: self._has_target = False
            return

        ok, bbox = self._tracker.update(frame)
        if not ok:
            cv.putText(frame, "Tracking lost", (100, 80),
                       cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            with self._tgt_lock: self._has_target = False
            return

        x, y, w, h = bbox
        cx, cy = x + w / 2.0, y + h / 2.0
        cv.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2, 1)

        with self._tgt_lock:
            self._has_target = True
            self._target_x = float(cx); self._target_y = float(cy)

    # ── 鼠标 ──────────────────────────────────────────────
    def _init_win(self) -> None:
        cv.namedWindow(self._win_name, cv.WINDOW_NORMAL)
        cv.resizeWindow(self._win_name, self._disp_w, self._disp_h)
        cv.setMouseCallback(self._win_name, self._on_mouse)
        self._win_ok = True

    def _ensure_win(self) -> None:
        if self._win_ok: return
        cv.namedWindow(self._win_name, cv.WINDOW_NORMAL)
        cv.resizeWindow(self._win_name, self._disp_w, self._disp_h)
        self._win_ok = True

    def _on_mouse(self, event, x, y, flags, param):
        sx = self._proc_w / max(self._disp_w, 1)
        sy = self._proc_h / max(self._disp_h, 1)
        px, py = int(x * sx), int(y * sy)
        if event == cv.EVENT_LBUTTONDOWN:
            self._state = STATE_INIT; self._select_flags = True
            self._mouse_xy = (px, py); self._ms_start = (px, py); self._ms_end = (px, py)
        elif event == cv.EVENT_MOUSEMOVE and self._select_flags:
            self._ms_start = self._mouse_xy; self._ms_end = (px, py)
            self._roi_init = (self._ms_start[0], self._ms_start[1], self._ms_end[0], self._ms_end[1])
        elif event == cv.EVENT_LBUTTONUP:
            self._select_flags = False
            self._init_tracker()

    def _init_tracker(self, frame_hint=None) -> None:
        x1, y1, x2, y2 = self._roi_init
        if x1 == x2 or y1 == y2: return
        rx = min(x1, x2); ry = min(y1, y2); rw = abs(x2 - x1); rh = abs(y2 - y1)
        if rw < 10 or rh < 10: return
        rx = int(clamp(rx, 0, self._proc_w - 1)); ry = int(clamp(ry, 0, self._proc_h - 1))
        rw = int(min(rw, self._proc_w - rx)); rh = int(min(rh, self._proc_h - ry))
        roi = (rx, ry, rw, rh)

        with self._img_lock:
            if self._latest_frame is None: return
            init_frame = self._latest_frame.copy()
        init_frame = cv.resize(init_frame, (self._proc_w, self._proc_h))

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

    # ── 键盘 ──────────────────────────────────────────────
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
        elif k == KEY_CODE_SPACE:
            if self._state == STATE_TRACKING:
                self._state = STATE_IDENTIFY; self._reset_pd(); self._pub_zero_cmd()
                self.get_logger().info("Paused.")
            elif self._tracker_ready:
                self._state = STATE_TRACKING; self._reset_pd()
                self.get_logger().info("Tracking.")

    # ── 车身 PD 控制 ─────────────────────────────────────
    def _rebuild_body_timer(self, log: bool) -> None:
        p = 1.0 / max(self._body_rate, MINIMUM_TIMER_RATE_HZ)
        if self._body_timer is not None: self.destroy_timer(self._body_timer)
        self._body_timer = self.create_timer(p, self._body_tick)
        if log: self.get_logger().info(f"Body timer: {self._body_rate:.1f}Hz")

    def _reset_pd(self) -> None:
        self._prev_s1 = 0.0; self._prev_body_t = 0.0; self._last_body_az = 0.0

    def _body_tick(self) -> None:
        if self._state != STATE_TRACKING:
            self._reset_pd(); self._pub_zero_cmd()
            return
        with self._tgt_lock:
            has_target = self._has_target; tx = self._target_x
        if not has_target or tx is None:
            self._reset_pd(); self._pub_zero_cmd()
            return

        icx = float(self._proc_w) * 0.5
        pixel_err = float(tx - icx)
        if abs(pixel_err) < self._db_x: pixel_err = 0.0
        s1 = pixel_err / icx

        n = time.perf_counter()
        dt = n - self._prev_body_t if self._prev_body_t > 0.0 else 1.0 / max(self._body_rate, MINIMUM_TIMER_RATE_HZ)
        dt = max(dt, 1e-3)
        ds1 = (s1 - self._prev_s1) / dt
        az = self._body_kp * s1 + self._body_kd * ds1
        az = clamp(az, -self._max_angular_z, self._max_angular_z)
        if self._rev_body: az = -az

        cmd = Twist(); cmd.angular.z = float(az)
        self._cmd_pub.publish(cmd)
        self._prev_s1 = s1; self._prev_body_t = n; self._last_body_az = az

    def _pub_zero_cmd(self) -> None:
        self._cmd_pub.publish(Twist())

    # ── 显示 ──────────────────────────────────────────────
    def _draw_guides(self, frame: np.ndarray) -> None:
        cx, cy = int(self._proc_w * 0.5), int(self._proc_h * 0.5)
        cv.line(frame, (cx, 0), (cx, self._proc_h), (255, 255, 0), 1)
        cv.line(frame, (0, cy), (self._proc_w, cy), (255, 255, 0), 1)
        if self._show_deadband and self._db_x > 0:
            cv.line(frame, (cx - int(self._db_x), 0), (cx - int(self._db_x), frame.shape[0]), (0, 180, 255), 2)
            cv.line(frame, (cx + int(self._db_x), 0), (cx + int(self._db_x), frame.shape[0]), (0, 180, 255), 2)
        if self._show_center:
            cv.line(frame, (cx, 0), (cx, frame.shape[0]), (0, 0, 255), 2)

    def _draw_overlay(self, frame: np.ndarray) -> None:
        with self._tgt_lock:
            state = self._state
            has_target = self._has_target
            tx = self._target_x
        tgt_txt = f"Target:{int(tx)},{int(self._target_y)}" if has_target and tx is not None else "Target:None"
        lines = [
            f"State:{state} FPS:{int(self._fps)} {tgt_txt}",
            f"Body: az={self._last_body_az:.2f} kp={self._body_kp:.2f} kd={self._body_kd:.2f} dead={self._db_x:.0f}",
            "Mouse:drag ROI | SPACE:pause/resume | q:quit",
        ]
        if self._bg:
            cv.rectangle(frame, (0, 0), (self._proc_w, self._line_h * len(lines) + 8), (0, 0, 0), -1)
        for i, text in enumerate(lines):
            cv.putText(frame, text, (self._ovl_x, self._ovl_y + i * self._line_h),
                       cv.FONT_HERSHEY_SIMPLEX, self._font_scale, (0, 255, 255), 1, cv.LINE_AA)

    # ── 动态参数更新 ──────────────────────────────────────
    def _on_param_update(self, params: List[Parameter]) -> SetParametersResult:
        if not params: return SetParametersResult(successful=True)
        names = {p.name for p in params}
        bad = names - DYNAMIC_PARAMETER_NAMES - RESTART_REQUIRED_PARAMETER_NAMES
        if bad: return SetParametersResult(successful=False, reason=f"Unsupported: {sorted(bad)}")
        if names & RESTART_REQUIRED_PARAMETER_NAMES: return SetParametersResult(successful=False, reason="Restart required")
        if not self._dyn and names != {"enable_rqt_dynamic_parameters"}: return SetParametersResult(successful=False, reason="Disabled")
        old_body, old_img = self._body_rate, self._img_rate
        old_debug, old_size = self._show_win, (self._win_w, self._win_h)
        v = self._merge(params)
        err = self._check(v)
        if err: return SetParametersResult(successful=False, reason=err)
        self._apply(v)
        if abs(self._body_rate - old_body) > 1e-9: self._rebuild_body_timer(True)
        if abs(self._img_rate - old_img) > 1e-9:
            self.destroy_timer(self._img_timer)
            self._img_timer = self.create_timer(1.0 / max(self._img_rate, MINIMUM_TIMER_RATE_HZ), self._process_image)
        self._sync_win(old_debug, old_size)
        return SetParametersResult(successful=True)

    def _merge(self, params):
        d = dict(show_debug_window=self._show_win, processing_width=self._proc_w, processing_height=self._proc_h,
                 overlay_font_scale=self._font_scale, overlay_line_height=self._line_h,
                 enable_horizontal_flip=self._flip_h, enable_vertical_flip=self._flip_v, rotate_image_code=self._rot,
                 angular_kp=self._body_kp, angular_kd=self._body_kd,
                 max_angular_z=self._max_angular_z, deadband_x_pixel=self._db_x,
                 reverse_body_direction=self._rev_body,
                 image_processing_rate_hz=self._img_rate, body_control_rate=self._body_rate,
                 tracker_type=self._trk_type,
                 show_deadband_lines=self._show_deadband, show_center_line=self._show_center,
                 enable_rqt_dynamic_parameters=self._dyn)
        for p in params:
            if p.type_ != Parameter.Type.NOT_SET: d[p.name] = p.value
        return d

    def _check(self, v):
        try:
            if float(v["angular_kp"]) < 0 or float(v["angular_kd"]) < 0: return "kp/kd >=0"
            if float(v["deadband_x_pixel"]) < 0: return "db >=0"
            if float(v["max_angular_z"]) <= 0: return "max_az >0"
        except Exception as e: return str(e)
        return ""

    def _apply(self, v):
        self._show_win = to_bool(v["show_debug_window"], True); self._disp_w = int(v["display_width"]); self._disp_h = int(v["display_height"])
        self._font_scale = float(v["overlay_font_scale"]); self._line_h = int(v["overlay_line_height"])
        self._flip_h = to_bool(v["enable_horizontal_flip"], False); self._flip_v = to_bool(v["enable_vertical_flip"], False)
        self._rot = str(v["rotate_image_code"]).lower()
        self._body_kp = max(float(v["angular_kp"]), 0.0); self._body_kd = max(float(v["angular_kd"]), 0.0)
        self._max_angular_z = max(float(v["max_angular_z"]), 0.0)
        self._db_x = float(v["deadband_x_pixel"])
        self._rev_body = bool(v["reverse_body_direction"])
        self._img_rate = float(v["image_processing_rate_hz"]); self._body_rate = float(v["body_control_rate"])
        self._trk_type = str(v["tracker_type"]); self._img_flip = to_bool(v["img_flip"], False)
        self._show_deadband = bool(v["show_deadband_lines"]); self._show_center = bool(v["show_center_line"])
        self._dyn = to_bool(v["enable_rqt_dynamic_parameters"], True)
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
    node = KcfTrackerNode()
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
