#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gesture_action_group_control_ptz_node.py

中文说明:
    ROS2 Humble 手势动作组控制节点（里程计驱动 + PD 闭环）。
    识别 5 种手势 → 触发对应底盘动作序列，里程计数据驱动完成判定。
    PTZ: waiting (0,-15), action (0,-45), done/waiting (0,-15).

English:
    ROS2 Humble gesture action group control node (odom-driven + PD).
    5 gestures → action sequences with odom-based completion + PD control.
"""

from __future__ import annotations

import math
import os
import threading
import time
from typing import Any, Dict, List, Optional

import cv2 as cv
import numpy as np
import rclpy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from .image_conversion import image_to_bgr

from .hand_detector import HandDetector

os.environ.setdefault("MEDIAPIPE_DISABLE_TF", "1")
os.environ.setdefault("MEDIAPIPE_DISABLE_GPU", "1")

DEFAULT_NODE_NAME = "gesture_action_group_control_ptz_node"
LANDMARK_INDEX_FINGER_MCP = 5

STATE_IDLE = "idle"
STATE_WAITING = "waiting"
STATE_ACTION = "action_running"

KEY_CODE_SPACE = 32
KEY_CODE_ESCAPE = 27
MINIMUM_TIMER_RATE_HZ = 1.0

GESTURE_STOP = "stop"
GESTURE_SQUARE = "square"
GESTURE_CIRCLE = "circle"
GESTURE_S = "s_curve"
GESTURE_BACKFORTH = "backforth"

DYNAMIC_PARAMETER_NAMES = frozenset({
    "show_debug_window",
    "processing_width", "processing_height",
    "display_width", "display_height",
    "overlay_font_scale", "overlay_line_height",
    "overlay_show_background",
    "enable_horizontal_flip", "enable_vertical_flip", "rotate_image_code",
    "waiting_horizontal_deg", "waiting_pitch_deg",
    "action_horizontal_deg", "action_pitch_deg",
    "square_forward_distance", "square_turn_angle_deg", "square_cycles",
    "backforth_distance",
    "circle_total_angle_deg",
    "s_turn_angles", "s_straight_distances", "s_linear_speed",
    "linear_kp", "linear_kd", "max_linear_x",
    "angular_kp", "angular_kd", "max_angular_z",
    "distance_tolerance", "angle_tolerance_deg", "settle_frames",
    "image_processing_rate_hz", "action_control_rate_hz",
    "gesture_hold_confirm_frames",
    "enable_rqt_dynamic_parameters",
})

RESTART_REQUIRED_PARAMETER_NAMES = frozenset({
    "image_topic", "ptz_command_topic", "cmd_vel_topic", "odom_topic", "window_name",
})


def clamp(value: float, lo: float, hi: float) -> float:
    return max(float(lo), min(float(hi), float(value)))


def quaternion_to_yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class GestureActionGroupControlPtzNode(Node):

    def __init__(self) -> None:
        super().__init__(DEFAULT_NODE_NAME)
        self._image_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._odom_lock = threading.Lock()

        self._latest_frame: Optional[np.ndarray] = None
        self._latest_seq = 0
        self._processed_seq = 0

        self._state = STATE_IDLE
        self._shutdown = False

        self._has_hand = False
        self._finger_code = "[0,0,0,0,0]"
        self._gesture_label = "None"
        self._gesture_confirm_count = 0
        self._gesture_lost_frames = 0
        self._last_gesture = ""
        self._last_gesture_key = ""

        self._h_deg = 0.0
        self._p_deg = -15.0

        # odom state
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._action_distance = 0.0

        # angle tracking — 绝对目标 yaw 方式，不累积
        self._phase_start_yaw = 0.0
        self._angle_target_deg = 0.0
        self._angle_current_deg = 0.0
        self._angle_err_deg = 0.0

        # action engine
        self._action_type = ""
        self._action_phase = 0
        self._phase_start_x = 0.0
        self._phase_start_y = 0.0
        self._phase_target_distance = 0.0
        self._square_cycle = 0
        self._phase_cooldown_counter = 0
        self._circle_cum_yaw = 0.0
        self._circle_prev_yaw = 0.0
        self._circle_need_brake = False

        # PD state
        self._prev_dist_err = 0.0
        self._prev_angle_err = 0.0
        self._prev_tick_t = 0.0
        self._settle_count = 0

        self._measured_fps = 0.0
        self._fps_count = 0
        self._fps_last_t = time.perf_counter()
        self._win_ready = False

        self._declare_params()
        self._load_params()

        self._hand_det = HandDetector()

        best_effort_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._img_sub = self.create_subscription(
            Image, self._img_topic, self._on_image, best_effort_qos)
        self._odom_sub = self.create_subscription(
            Odometry, self._odom_topic, self._odom_callback, best_effort_qos)
        self._ptz_pub = self.create_publisher(Vector3, self._ptz_topic, 10)
        self._cmd_pub = self.create_publisher(Twist, self._cmd_topic, best_effort_qos)

        self._img_timer = self.create_timer(
            1.0 / max(float(self._img_rate), MINIMUM_TIMER_RATE_HZ), self._process_image)
        self._action_timer = self.create_timer(
            1.0 / max(float(self._action_rate), MINIMUM_TIMER_RATE_HZ), self._action_tick)

        self._key_timer = None
        if self._show_win:
            self._ensure_win()
            self._key_timer = self.create_timer(0.05, self._poll_keys)

        self._param_cb = self.add_on_set_parameters_callback(self._on_param_update)

        self._pub_ptz(self._wait_h_deg, self._wait_p_deg)

        self.get_logger().info(
            "GestureActionGroup started. SPACE=wait, i=idle, q=quit. img=%.0fHz action=%.0fHz"
            % (self._img_rate, self._action_rate))

    # ── Params ──────────────────────────────────────────────

    def _declare_params(self) -> None:
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("ptz_command_topic", "/cmd_ptz_angle")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom_raw")
        self.declare_parameter("window_name", "gesture_action_ctrl")
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
        self.declare_parameter("waiting_horizontal_deg", 0.0)
        self.declare_parameter("waiting_pitch_deg", -15.0)
        self.declare_parameter("action_horizontal_deg", 0.0)
        self.declare_parameter("action_pitch_deg", -45.0)
        self.declare_parameter("square_forward_distance", 0.5)
        self.declare_parameter("square_turn_angle_deg", 90.0)
        self.declare_parameter("square_cycles", 1)
        self.declare_parameter("backforth_distance", 0.5)
        self.declare_parameter("circle_total_angle_deg", 360.0)
        self.declare_parameter("circle_kp", 0.05)
        self.declare_parameter("circle_kd", 0.002)
        self.declare_parameter("circle_max_angular_z", 0.8)
        self.declare_parameter("circle_tolerance_deg", 3.0)
        self.declare_parameter("circle_settle_frames", 3)
        self.declare_parameter("s_turn_angles", [30.0, -60.0, 60.0, -60.0, 30.0])
        self.declare_parameter("s_straight_distances", [0.288, 0.576, 0.576, 0.288, 0.5])
        self.declare_parameter("s_linear_speed", 0.15)
        self.declare_parameter("linear_kp", 0.5)
        self.declare_parameter("linear_kd", 0.1)
        self.declare_parameter("max_linear_x", 0.2)
        self.declare_parameter("angular_kp", 0.08)
        self.declare_parameter("angular_kd", 0.15)
        self.declare_parameter("max_angular_z", 1.0)
        self.declare_parameter("distance_tolerance", 0.03)
        self.declare_parameter("angle_tolerance_deg", 5.0)
        self.declare_parameter("settle_frames", 5)
        self.declare_parameter("phase_cooldown_ticks", 10)
        self.declare_parameter("image_processing_rate_hz", 15.0)
        self.declare_parameter("action_control_rate_hz", 30.0)
        self.declare_parameter("gesture_hold_confirm_frames", 3)
        self.declare_parameter("enable_rqt_dynamic_parameters", True)

    def _load_params(self) -> None:
        self._img_topic = str(self.get_parameter("image_topic").value)
        self._ptz_topic = str(self.get_parameter("ptz_command_topic").value)
        self._cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
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
        self._wait_h_deg = float(self.get_parameter("waiting_horizontal_deg").value)
        self._wait_p_deg = float(self.get_parameter("waiting_pitch_deg").value)
        self._action_h_deg = float(self.get_parameter("action_horizontal_deg").value)
        self._action_p_deg = float(self.get_parameter("action_pitch_deg").value)
        self._sq_fwd_dist = max(float(self.get_parameter("square_forward_distance").value), 0.01)
        self._sq_turn_deg = max(float(self.get_parameter("square_turn_angle_deg").value), 1.0)
        self._sq_cycles = max(int(self.get_parameter("square_cycles").value), 1)
        self._backforth_dist = max(float(self.get_parameter("backforth_distance").value), 0.01)
        self._circle_total_deg = max(float(self.get_parameter("circle_total_angle_deg").value), 1.0)
        self._circle_kp = max(float(self.get_parameter("circle_kp").value), 0.0)
        self._circle_kd = max(float(self.get_parameter("circle_kd").value), 0.0)
        self._circle_max_z = max(float(self.get_parameter("circle_max_angular_z").value), 0.0)
        self._circle_tol_deg = max(float(self.get_parameter("circle_tolerance_deg").value), 0.1)
        self._circle_settle = max(int(self.get_parameter("circle_settle_frames").value), 1)
        self._s_angles = [float(x) for x in self.get_parameter("s_turn_angles").value]
        self._s_dists = [float(x) for x in self.get_parameter("s_straight_distances").value]
        self._s_total = len(self._s_angles) + len(self._s_dists)
        self._s_linear = abs(float(self.get_parameter("s_linear_speed").value))
        self._s_phase_index = 0
        self._s_phase_dist = 0.0
        self._linear_kp = max(float(self.get_parameter("linear_kp").value), 0.0)
        self._linear_kd = max(float(self.get_parameter("linear_kd").value), 0.0)
        self._max_linear_x = max(float(self.get_parameter("max_linear_x").value), 0.0)
        self._angular_kp = max(float(self.get_parameter("angular_kp").value), 0.0)
        self._angular_kd = max(float(self.get_parameter("angular_kd").value), 0.0)
        self._max_angular_z = max(float(self.get_parameter("max_angular_z").value), 0.0)
        self._dist_tol = max(float(self.get_parameter("distance_tolerance").value), 0.001)
        self._angle_tol_deg = max(float(self.get_parameter("angle_tolerance_deg").value), 0.1)
        self._settle_frames = max(int(self.get_parameter("settle_frames").value), 1)
        self._cooldown_ticks = max(int(self.get_parameter("phase_cooldown_ticks").value), 0)
        self._img_rate = max(float(self.get_parameter("image_processing_rate_hz").value), MINIMUM_TIMER_RATE_HZ)
        self._action_rate = max(float(self.get_parameter("action_control_rate_hz").value), MINIMUM_TIMER_RATE_HZ)
        self._confirm_frames = max(int(self.get_parameter("gesture_hold_confirm_frames").value), 1)
        self._dyn = bool(self.get_parameter("enable_rqt_dynamic_parameters").value)

    # ── Odom ────────────────────────────────────────────────

    def _odom_callback(self, msg: Odometry) -> None:
        with self._odom_lock:
            self._odom_x = float(msg.pose.pose.position.x)
            self._odom_y = float(msg.pose.pose.position.y)
            self._odom_yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    # ── Image ──────────────────────────────────────────────

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
            self._pub_zero_cmd()
            os._exit(0)
        if k in (ord("i"), ord("I")):
            self._enter_idle()
            return
        if k == KEY_CODE_SPACE:
            if self._state == STATE_IDLE:
                self._enter_waiting()
            else:
                self._enter_idle()

    # ── Gesture detection ──────────────────────────────────

    def _detect(self, frame: np.ndarray) -> None:
        try:
            self._hand_det.findHands(frame, draw=True)
        except TypeError:
            self._hand_det.findHands(frame)

        if len(self._hand_det.lmList) == 0:
            with self._state_lock:
                self._has_hand = False
                self._finger_code = "[0,0,0,0,0]"
                self._gesture_label = "None"
                if self._gesture_lost_frames < 10:
                    self._gesture_lost_frames += 1
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

        # gesture label mapping
        if fc == 5:
            glabel = "5-finger"
        elif nf == [0, 1, 0, 0, 1]:
            glabel = "Rock"
        elif fc >= 3 and nf[1] == 0:
            glabel = "OK"
        elif nf == [0, 1, 1, 0, 0]:
            glabel = "Yes"
        elif fc <= 1 and self._hand_det.cyList and len(self._hand_det.cyList) > 4:
            if self._hand_det.cyList[4] == max(self._hand_det.cyList):
                glabel = "Thumb Down"
            else:
                glabel = "Unknown"
        else:
            glabel = "Unknown"

        gesture_key = ""
        if fc == 5:
            gesture_key = GESTURE_STOP
        elif nf == [0, 1, 0, 0, 1]:
            gesture_key = GESTURE_S                             # Rock: 食指+小指
        elif fc >= 3 and nf[1] == 0:
            gesture_key = GESTURE_CIRCLE                        # OK: ≥3指伸展且食指弯曲
        elif nf == [0, 1, 1, 0, 0]:
            gesture_key = GESTURE_SQUARE                        # Yes: 食指+中指
        elif fc <= 1 and self._hand_det.cyList and len(self._hand_det.cyList) > 4:
            if self._hand_det.cyList[4] == max(self._hand_det.cyList):
                gesture_key = GESTURE_BACKFORTH                 # Thumb_down: ≤1指伸展+拇指向下

        # palm bounding box
        x_vals = [pt[1] for pt in self._hand_det.lmList]
        y_vals = [pt[2] for pt in self._hand_det.lmList]
        if x_vals and y_vals:
            cv.rectangle(frame, (int(min(x_vals)), int(min(y_vals))),
                         (int(max(x_vals)), int(max(y_vals))), (0, 255, 0), 2)
        cv.circle(frame, (int(mx), int(my)), 8, (255, 0, 255), -1)
        cv.circle(frame, (int(mx), int(my)), 10, (255, 255, 0), 2)

        with self._state_lock:
            self._has_hand = True
            self._gesture_lost_frames = 0
            self._finger_code = fcode
            self._gesture_label = glabel

            if gesture_key:
                if gesture_key == self._last_gesture:
                    self._gesture_confirm_count += 1
                else:
                    self._gesture_confirm_count = 1
                    self._last_gesture = gesture_key
                self._last_gesture_key = gesture_key
            else:
                self._gesture_confirm_count = 0
                self._last_gesture = ""
                if glabel == "Unknown" and fcode == "[0,0,0,0,0]":
                    self._last_gesture_key = ""

            # "5" 手势可在任何状态中断动作
            if gesture_key == GESTURE_STOP and self._gesture_confirm_count >= self._confirm_frames:
                if self._state == STATE_ACTION:
                    self._action_type = ""
                    self._action_phase = 0
                    self._enter_waiting()
                self._pub_zero_cmd()
            elif (self._state == STATE_WAITING and self._gesture_confirm_count >= self._confirm_frames
                    and gesture_key):
                self._start_action(gesture_key)

    # ── Action engine ──────────────────────────────────────

    def _start_action(self, gesture_key: str) -> None:
        self._action_type = gesture_key
        self._action_phase = 0
        self._square_cycle = 0
        self._settle_count = 0
        self._prev_dist_err = 0.0
        self._prev_angle_err = 0.0
        self._prev_tick_t = time.perf_counter()
        self._phase_cooldown_counter = 0
        self._circle_cum_yaw = 0.0
        self._circle_need_brake = False
        self._circle_settle_count = 0
        self._s_phase_index = 0
        self._s_phase_dist = 0.0
        with self._odom_lock:
            self._phase_start_x = self._odom_x
            self._phase_start_y = self._odom_y
            self._phase_start_yaw = self._odom_yaw
            self._circle_prev_yaw = self._odom_yaw
        self._action_distance = 0.0
        self._state = STATE_ACTION
        self._pub_ptz(self._action_h_deg, self._action_p_deg)
        self.get_logger().info(
            f"Action started: {gesture_key}, "
            f"PTZ=({self._action_h_deg:.1f}, {self._action_p_deg:.1f})"
        )

    def _action_tick(self) -> None:
        if self._state == STATE_IDLE:
            self._pub_zero_cmd()
            return
        if self._state == STATE_WAITING:
            self._pub_zero_cmd()
            return
        if self._state != STATE_ACTION:
            return

        with self._odom_lock:
            ox = self._odom_x
            oy = self._odom_y
            oyaw = self._odom_yaw

        phase_dist = math.hypot(ox - self._phase_start_x, oy - self._phase_start_y)
        self._action_distance = phase_dist

        if self._action_type == GESTURE_SQUARE:
            self._square_tick(oyaw, phase_dist)
        elif self._action_type == GESTURE_CIRCLE:
            self._circle_tick(oyaw)
        elif self._action_type == GESTURE_S:
            self._s_curve_tick(oyaw)
        elif self._action_type == GESTURE_BACKFORTH:
            self._backforth_tick(phase_dist)

    # ── PD helpers ─────────────────────────────────────────

    def _distance_pd(self, target_dist: float, current_dist: float) -> float:
        err = target_dist - current_dist
        now = time.perf_counter()
        dt = now - self._prev_tick_t if self._prev_tick_t > 0.0 else 1.0 / max(float(self._action_rate), MINIMUM_TIMER_RATE_HZ)
        dt = max(dt, 1e-3)
        d_err = (err - self._prev_dist_err) / dt
        self._prev_dist_err = err
        self._prev_tick_t = now

        if abs(err) < self._dist_tol:
            self._settle_count += 1
        else:
            self._settle_count = 0

        out = self._linear_kp * err + self._linear_kd * d_err
        return clamp(out, -self._max_linear_x, self._max_linear_x)

    def _angle_pd(self, target_deg: float, current_yaw: float) -> float:
        """角度 PD：直接对比 IMU yaw 与阶段起点推算的绝对目标角度，不累积"""
        target_yaw = self._phase_start_yaw + math.radians(target_deg)
        err = target_yaw - current_yaw
        # 手动 wrap 到 ±π（不调 atan2，atan2 等价于 wrapToPi 但不如显式清晰）
        if err > math.pi:
            err -= 2.0 * math.pi
        elif err < -math.pi:
            err += 2.0 * math.pi
        err_deg = math.degrees(err)

        # 存储用于 overlay 显示（均使用绝对角度）
        self._angle_target_deg = math.degrees(target_yaw)
        self._angle_current_deg = math.degrees(current_yaw)
        self._angle_err_deg = err_deg

        now = time.perf_counter()
        dt = now - self._prev_tick_t if self._prev_tick_t > 0.0 else 1.0 / max(float(self._action_rate), MINIMUM_TIMER_RATE_HZ)
        dt = max(dt, 1e-3)
        d_err = (err_deg - self._prev_angle_err) / dt
        self._prev_angle_err = err_deg
        self._prev_tick_t = now

        if abs(err_deg) < self._angle_tol_deg:
            self._settle_count += 1
        else:
            self._settle_count = 0

        out = self._angular_kp * err_deg + self._angular_kd * d_err
        return clamp(out, -self._max_angular_z, self._max_angular_z)

    def _distance_done(self) -> bool:
        return self._settle_count >= self._settle_frames

    def _new_phase(self) -> None:
        self._phase_cooldown_counter = 0
        with self._odom_lock:
            self._phase_start_x = self._odom_x
            self._phase_start_y = self._odom_y
            self._phase_start_yaw = self._odom_yaw
        self._prev_dist_err = 0.0
        self._prev_angle_err = 0.0
        self._settle_count = 0

    def _advance_phase(self) -> None:
        """PD settle 完成 → 进入冷却期：停车等机器人稳定 → 记录 yaw → 切阶段"""
        if self._phase_cooldown_counter < self._cooldown_ticks:
            self._phase_cooldown_counter += 1
            return  # 还在冷却，继续停车
        self._action_phase += 1
        self._new_phase()

    # ── Square ──────────────────────────────────────────────

    def _square_tick(self, oyaw: float, phase_dist: float) -> None:
        cmd = Twist()
        phase = self._action_phase

        if self._phase_cooldown_counter > 0:
            # 冷却期：停车等待机器人稳定
            self._cmd_pub.publish(Twist())
            self._advance_phase()
            if self._action_phase >= 8:
                self._square_cycle += 1
                if self._square_cycle >= self._sq_cycles:
                    self._finish_action()
                    return
                self._action_phase = 0
            return

        if phase % 2 == 0:  # forward
            cmd.linear.x = self._distance_pd(self._sq_fwd_dist, phase_dist)
            if self._distance_done():
                self._advance_phase()
        else:  # turn
            cmd.angular.z = self._angle_pd(-self._sq_turn_deg, oyaw)
            if self._distance_done():
                self._advance_phase()
        self._cmd_pub.publish(cmd)

    # ── Circle ──────────────────────────────────────────────

    def _circle_tick(self, oyaw: float) -> None:
        # 累计 yaw 变化
        dyaw = oyaw - self._circle_prev_yaw
        if dyaw > math.pi:
            dyaw -= 2.0 * math.pi
        elif dyaw < -math.pi:
            dyaw += 2.0 * math.pi
        self._circle_cum_yaw += dyaw
        self._circle_prev_yaw = oyaw

        cum_deg = math.degrees(abs(self._circle_cum_yaw))
        target_deg = self._circle_total_deg

        # 转圈独立 PD 控制
        err = target_deg - cum_deg
        now = time.perf_counter()
        dt = now - self._prev_tick_t if self._prev_tick_t > 0.0 else 1.0 / max(float(self._action_rate), MINIMUM_TIMER_RATE_HZ)
        dt = max(dt, 1e-3)
        d_err = (err - self._prev_angle_err) / dt
        self._prev_angle_err = err
        self._prev_tick_t = now

        out = self._circle_kp * err + self._circle_kd * d_err
        out = clamp(out, -self._circle_max_z, self._circle_max_z)

        if abs(err) < self._circle_tol_deg:
            self._circle_settle_count += 1
        else:
            self._circle_settle_count = 0

        if self._circle_settle_count >= self._circle_settle:
            self._circle_cum_yaw = 0.0
            self._circle_settle_count = 0
            self._pub_zero_cmd()
            self._finish_action()
            return

        cmd = Twist()
        cmd.angular.z = -out
        self._cmd_pub.publish(cmd)

    # ── S-curve ─────────────────────────────────────────────

    def _s_curve_tick(self, oyaw: float) -> None:
        with self._odom_lock:
            ox = self._odom_x
            oy = self._odom_y
        phase_dist = math.hypot(ox - self._phase_start_x, oy - self._phase_start_y)
        self._action_distance = phase_dist

        if self._s_phase_index >= self._s_total:
            self._finish_action()
            return

        if self._phase_cooldown_counter > 0:
            self._cmd_pub.publish(Twist())
            self._phase_cooldown_counter += 1
            if self._phase_cooldown_counter > self._cooldown_ticks:
                self._s_phase_index += 1
                self._new_phase()
            return

        is_turn = (self._s_phase_index % 2 == 0)
        step = self._s_phase_index // 2

        cmd = Twist()
        if is_turn:
            target = float(self._s_angles[step])
            cmd.angular.z = self._angle_pd(target, oyaw)
            if self._distance_done():
                self._phase_cooldown_counter = 1
        else:
            target = float(self._s_dists[step])
            cmd.linear.x = self._distance_pd(target, phase_dist)
            if self._distance_done():
                self._phase_cooldown_counter = 1
        self._cmd_pub.publish(cmd)

    # ── Backforth ───────────────────────────────────────────

    def _backforth_tick(self, phase_dist: float) -> None:
        cmd = Twist()
        phase = self._action_phase

        if self._phase_cooldown_counter > 0:
            self._cmd_pub.publish(Twist())
            self._advance_phase()
            if self._action_phase >= 2:
                self._finish_action()
            return

        if phase == 0:  # forward
            cmd.linear.x = self._distance_pd(self._backforth_dist, phase_dist)
            if self._distance_done():
                self._advance_phase()
        elif phase == 1:  # backward
            cmd.linear.x = -self._distance_pd(self._backforth_dist, phase_dist)
            if self._distance_done():
                self._advance_phase()
        self._cmd_pub.publish(cmd)

    def _finish_action(self) -> None:
        self._action_type = ""
        self._action_phase = 0
        self._enter_waiting()
        self.get_logger().info("Action finished. Back to WAITING.")

    # ── State transitions ─────────────────────────────────

    def _enter_idle(self) -> None:
        self._state = STATE_IDLE
        self._action_type = ""
        self._action_phase = 0
        self._gesture_confirm_count = 0
        self._last_gesture = ""
        self._pub_ptz(self._wait_h_deg, self._wait_p_deg)
        self._pub_zero_cmd()
        self.get_logger().info("IDLE")

    def _enter_waiting(self) -> None:
        self._state = STATE_WAITING
        self._gesture_confirm_count = 0
        self._last_gesture = ""
        self._pub_ptz(self._wait_h_deg, self._wait_p_deg)
        self._pub_zero_cmd()
        self.get_logger().info("WAITING for gesture...")

    # ── Publish ────────────────────────────────────────────

    def _pub_ptz(self, h_deg: float, p_deg: float) -> None:
        self._h_deg = h_deg
        self._p_deg = p_deg
        msg = Vector3()
        msg.x = float(h_deg)
        msg.y = float(p_deg)
        msg.z = 0.0
        self._ptz_pub.publish(msg)

    def _pub_zero_cmd(self) -> None:
        self._cmd_pub.publish(Twist())

    # ── Display ────────────────────────────────────────────

    def _ensure_win(self) -> None:
        if self._win_ready:
            return
        cv.namedWindow(self._win_name, cv.WINDOW_NORMAL)
        cv.resizeWindow(self._win_name, self._disp_w, self._disp_h)
        self._win_ready = True

    def _draw_guides(self, frame: np.ndarray) -> None:
        cx, cy = int(frame.shape[1] * 0.5), int(frame.shape[0] * 0.5)
        cv.line(frame, (cx, 0), (cx, frame.shape[0]), (255, 255, 0), 1)
        cv.line(frame, (0, cy), (frame.shape[1], cy), (255, 255, 0), 1)

    def _draw_overlay(self, frame: np.ndarray) -> None:
        with self._state_lock:
            state = self._state
            gesture = self._gesture_label
            fcode = self._finger_code
            confirm = self._gesture_confirm_count
        with self._odom_lock:
            ox = self._odom_x
            oy = self._odom_y
        lines = [
            "State: %s | FPS: %.1f" % (state, self._measured_fps),
            "Gesture: %s %s -> %s | Confirm: %d/%d" % (gesture, fcode, self._last_gesture_key, confirm, self._confirm_frames),
            "Action: %s phase=%d dist=%.2f" % (self._action_type, self._action_phase, self._action_distance),
            "Turn: target=%.1f cur=%.1f err=%.1f settle=%d/%d" % (
                self._angle_target_deg, self._angle_current_deg, self._angle_err_deg,
                self._settle_count, self._settle_frames),
            "Odom: (%.2f,%.2f) | PTZ: H=%.1f P=%.1f" % (ox, oy, self._h_deg, self._p_deg),
            "Keys: SPACE wait, i idle, q quit",
        ]
        if self._bg:
            cv.rectangle(frame, (0, 0), (min(frame.shape[1], 680), self._line_h * len(lines) + 8), (0, 0, 0), -1)
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
            should_recreate_img = False
            should_recreate_action = False
            ptz_angle_params = {}
            for p in params:
                if p.name in RESTART_REQUIRED_PARAMETER_NAMES:
                    return SetParametersResult(successful=False, reason=f"'{p.name}' requires restart.")
                if p.name not in DYNAMIC_PARAMETER_NAMES:
                    continue
                if p.name == "image_processing_rate_hz":
                    should_recreate_img = True
                if p.name == "action_control_rate_hz":
                    should_recreate_action = True
                if p.name in {
                    "waiting_horizontal_deg",
                    "waiting_pitch_deg",
                    "action_horizontal_deg",
                    "action_pitch_deg",
                }:
                    ptz_angle_params[p.name] = float(p.value)
                if p.name == "show_debug_window":
                    if not bool(p.value) and self._win_ready:
                        cv.destroyWindow(self._win_name)
                        self._win_ready = False
            self._load_params()
            if ptz_angle_params:
                self._wait_h_deg = ptz_angle_params.get(
                    "waiting_horizontal_deg", self._wait_h_deg)
                self._wait_p_deg = ptz_angle_params.get(
                    "waiting_pitch_deg", self._wait_p_deg)
                self._action_h_deg = ptz_angle_params.get(
                    "action_horizontal_deg", self._action_h_deg)
                self._action_p_deg = ptz_angle_params.get(
                    "action_pitch_deg", self._action_p_deg)
                if self._state == STATE_ACTION:
                    self._pub_ptz(self._action_h_deg, self._action_p_deg)
                else:
                    self._pub_ptz(self._wait_h_deg, self._wait_p_deg)
            if should_recreate_img:
                self.destroy_timer(self._img_timer)
                self._img_timer = self.create_timer(
                    1.0 / max(float(self._img_rate), MINIMUM_TIMER_RATE_HZ), self._process_image)
            if should_recreate_action:
                self.destroy_timer(self._action_timer)
                self._action_timer = self.create_timer(
                    1.0 / max(float(self._action_rate), MINIMUM_TIMER_RATE_HZ), self._action_tick)
            return SetParametersResult(successful=True)
        except Exception as e:
            return SetParametersResult(successful=False, reason=str(e))

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
    node = GestureActionGroupControlPtzNode()
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
