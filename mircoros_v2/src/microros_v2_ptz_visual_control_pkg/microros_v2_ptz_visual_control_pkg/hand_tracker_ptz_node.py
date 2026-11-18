#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
hand_tracker_ptz_node.py

中文说明:
    ROS2 Humble 手掌检测 + PTZ 云台跟随节点。
    识别手掌、框选手掌包围框，以食指根部关节（LM5）为跟踪目标点，
    使用 PD 控制器驱动云台跟随。不分析手指/手势。

    PTZ 控制框架与 gesture_follow_ptz_node 完全一致：
    归一化像素误差 → PD → 步进限幅 → 步进平滑 → 角度平滑 → /cmd_ptz_angle。

English:
    ROS2 Humble hand detection + PTZ follow node.
    Detects hand, draws bounding box, tracks index finger MCP (LM5) via PD control.
    No finger/gesture analysis — pure hand tracking.
"""

from __future__ import annotations

import os

os.environ.setdefault("MEDIAPIPE_DISABLE_TF", "1")
os.environ.setdefault("MEDIAPIPE_DISABLE_GPU", "1")

import threading
import time
from typing import Any, List, Optional

import cv2 as cv
import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from .image_conversion import image_to_bgr

from .hand_detector import HandDetector


DEFAULT_NODE_NAME = "hand_tracker_ptz_node"
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
    "lost_frame_threshold",
    "enable_rqt_dynamic_parameters",
})

RESTART_REQUIRED_PARAMETER_NAMES = frozenset({
    "image_topic", "ptz_command_topic", "window_name",
})


def clamp(value: float, lo: float, hi: float) -> float:
    return max(float(lo), min(float(hi), float(value)))


class HandTrackerPtzNode(Node):

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
        self._finger_count = 0
        self._lost_frames = 0
        self._target_x: Optional[float] = None
        self._target_y: Optional[float] = None

        self._h_deg = 0.0
        self._p_deg = -45.0

        self._prev_err_x = 0.0
        self._prev_err_y = 0.0
        self._prev_ctrl_t = 0.0
        self._last_sx = 0.0
        self._last_sy = 0.0

        self._measured_fps = 0.0
        self._fps_count = 0
        self._fps_last_t = time.perf_counter()
        self._win_ready = False

        self._declare_params()
        self._load_params()

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

        # 发布初始云台角度 s1=0°, s2=-45°
        init_msg = Vector3()
        init_msg.x = float(self._h_deg)
        init_msg.y = float(self._p_deg)
        init_msg.z = 0.0
        self._ptz_pub.publish(init_msg)

        self._img_timer = self.create_timer(
            1.0 / max(float(self._img_rate), MINIMUM_TIMER_RATE_HZ), self._process_image
        )
        self._ptz_timer = self.create_timer(
            1.0 / max(float(self._ptz_rate), MINIMUM_TIMER_RATE_HZ), self._ptz_tick
        )

        self._key_timer = None
        if self._show_win:
            self._ensure_win()
            self._key_timer = self.create_timer(0.05, self._poll_keys)

        self._param_cb = self.add_on_set_parameters_callback(self._on_param_update)

        self.get_logger().info(
            "HandTracker PTZ started. SPACE=toggle, q=quit. "
            "ptz=%.0fHz img=%.0fHz kp=%.2f kd=%.2f"
            % (self._ptz_rate, self._img_rate, self._kp, self._kd)
        )

    def _declare_params(self) -> None:
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("ptz_command_topic", "/cmd_ptz_angle")
        self.declare_parameter("window_name", "hand_tracker_ptz")
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
        self.declare_parameter("lost_frame_threshold", 5)
        self.declare_parameter("enable_rqt_dynamic_parameters", True)

    def _load_params(self) -> None:
        self._img_topic = str(self.get_parameter("image_topic").value)
        self._ptz_topic = str(self.get_parameter("ptz_command_topic").value)
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
        self._lost_thr = max(1, int(self.get_parameter("lost_frame_threshold").value))
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
            self.get_logger().info("IDLE")
            return
        if k == KEY_CODE_SPACE:
            if self._state == STATE_TRACKING:
                self._state = STATE_IDLE
                self._reset_pd()
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
            return

        try:
            mx, my = self._hand_det.findPoint(LANDMARK_INDEX_FINGER_MCP)
        except Exception:
            mx, my = 0, 0

        fingers = self._hand_det.fingersUp()
        if fingers is None:
            fingers = [0, 0, 0, 0, 0]
        fc = int(sum(1 if int(v) != 0 else 0 for v in fingers))

        # draw palm bounding box
        x_vals = [pt[1] for pt in self._hand_det.lmList]
        y_vals = [pt[2] for pt in self._hand_det.lmList]
        if x_vals and y_vals:
            x1, y1 = int(min(x_vals)), int(min(y_vals))
            x2, y2 = int(max(x_vals)), int(max(y_vals))
            cv.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # draw tracking point
        cv.circle(frame, (int(mx), int(my)), 8, (255, 0, 255), -1)
        cv.circle(frame, (int(mx), int(my)), 10, (255, 255, 0), 2)

        with self._track_lock:
            self._has_hand = True
            self._lost_frames = 0
            self._finger_count = fc
            self._target_x = float(mx)
            self._target_y = float(my)

    def _reset_pd(self) -> None:
        self._prev_err_x = 0.0
        self._prev_err_y = 0.0
        self._prev_ctrl_t = 0.0
        self._last_sx = 0.0
        self._last_sy = 0.0

    def _ptz_tick(self) -> None:
        if self._state != STATE_TRACKING:
            return
        with self._track_lock:
            tx = self._target_x
            ty = self._target_y
            has_hand = self._has_hand
            fc = self._finger_count

        # 无手或握拳 → 不输出，不重置 PD 状态（与 gesture_tracker 一致）
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

        msg = Vector3()
        msg.x = float(self._h_deg)
        msg.y = float(self._p_deg)
        msg.z = 0.0
        self._ptz_pub.publish(msg)

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
            cv.rectangle(
                frame,
                (cx - int(self._db_x), cy - int(self._db_y)),
                (cx + int(self._db_x), cy + int(self._db_y)),
                (80, 80, 80),
                1,
            )

    def _draw_overlay(self, frame: np.ndarray) -> None:
        with self._track_lock:
            state = self._state
            has_hand = self._has_hand

        lines = [
            "State: %s | FPS: %.1f | Hand: %s" % (state, self._measured_fps, "YES" if has_hand else "NO"),
            "PTZ: H=%.1f P=%.1f  step=%.1f" % (self._h_deg, self._p_deg, self._max_step),
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
            for p in params:
                if p.name in RESTART_REQUIRED_PARAMETER_NAMES:
                    return SetParametersResult(successful=False, reason=f"'{p.name}' requires restart.")
                if p.name not in DYNAMIC_PARAMETER_NAMES:
                    continue
                if p.name == "ptz_control_rate":
                    should_recreate_ptz = True
                if p.name == "image_processing_rate_hz":
                    should_recreate_img = True
                if p.name == "show_debug_window":
                    if not bool(p.value) and self._win_ready:
                        cv.destroyWindow(self._win_name)
                        self._win_ready = False

            self._apply_param_values(params)
            if should_recreate_ptz:
                self.destroy_timer(self._ptz_timer)
                self._ptz_timer = self.create_timer(
                    1.0 / max(float(self._ptz_rate), MINIMUM_TIMER_RATE_HZ), self._ptz_tick)
            if should_recreate_img:
                self.destroy_timer(self._img_timer)
                self._img_timer = self.create_timer(
                    1.0 / max(float(self._img_rate), MINIMUM_TIMER_RATE_HZ), self._process_image)
            return SetParametersResult(successful=True)
        except Exception as e:
            return SetParametersResult(successful=False, reason=str(e))

    def _apply_param_values(self, params: List[Parameter]) -> None:
        for p in params:
            name, val = p.name, p.value
            if name in ("show_debug_window", "enable_horizontal_flip", "enable_vertical_flip",
                        "enable_rqt_dynamic_parameters"):
                setattr(self, "_" + name.replace("enable_", "").replace("show_", ""), bool(val))
            elif name in ("processing_width", "processing_height", "display_width", "display_height",
                          "overlay_line_height", "lost_frame_threshold"):
                setattr(self, "_" + name.replace("processing_", "proc_").replace("display_", "disp_")
                        .replace("overlay_line_height", "line_h").replace("lost_frame_threshold", "lost_thr"), int(val))
            elif name in ("horizontal_min_deg", "horizontal_max_deg", "pitch_min_deg", "pitch_max_deg",
                          "deadband_x", "deadband_y", "pd_kp", "pd_kd",
                          "ptz_step_smooth_alpha", "max_ptz_step", "ptz_smooth_alpha",
                          "ptz_control_rate", "image_processing_rate_hz", "overlay_font_scale"):
                setattr(self, "_" + name.replace("horizontal_", "h_").replace("pitch_", "p_").
                        replace("ptz_control_rate", "ptz_rate").replace("image_processing_rate_hz", "img_rate").
                        replace("deadband_", "db_").replace("overlay_font_scale", "font_scale").
                        replace("pd_", "").replace("ptz_step_smooth_alpha", "step_alpha").
                        replace("max_ptz_step", "max_step").replace("ptz_smooth_alpha", "ptz_alpha"), float(val))
            elif name == "rotate_image_code":
                self._rot = str(val)
            elif name in ("window_name",):
                setattr(self, "_win_name", str(val))
            elif name == "image_topic":
                self._img_topic = str(val)
            elif name == "ptz_command_topic":
                self._ptz_topic = str(val)
            elif name == "overlay_show_background":
                self._bg = bool(val)

    def destroy_node(self) -> bool:
        self._shutdown = True
        if self._show_win:
            cv.destroyAllWindows()
        try:
            self._hand_det.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = HandTrackerPtzNode()
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
