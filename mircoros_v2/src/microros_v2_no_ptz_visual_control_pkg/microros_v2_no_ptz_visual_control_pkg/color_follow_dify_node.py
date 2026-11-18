#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
color_follow_dify_node.py

Dify 专用颜色跟随节点 - No-PTZ 版本（车身控制）
- 无 GUI 窗口，后台运行
- 接受颜色名称参数 (red/green/blue/yellow)
- 从配置文件读取预标定的 HSV 值
- 仅车身控制，不涉及云台
- 专为 Dify 工作流优化
"""

from __future__ import annotations

import os
import threading
import time
from typing import Optional, Tuple

import cv2 as cv
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
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


class ColorFollowDifyNode(Node):
    """
    Dify 专用颜色跟随节点 - No-PTZ 版本

    特点：
    - 无 GUI，纯后台运行
    - 接受颜色名称参数 (red/green/blue/yellow)
    - 从预标定配置文件读取 HSV 值
    - 仅车身 PD 控制角速度，保持目标在画面中心
    """

    def __init__(self) -> None:
        super().__init__("color_follow_dify_node")

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

        # 车身 PD 状态
        self._prev_s1 = 0.0
        self._prev_body_t = 0.0

        # 距离跟随状态
        self._prev_r_err = 0.0
        self._prev_linear_t = 0.0

        # FPS 显示
        self._fps = 0.0
        self._fps_n = 0
        self._fps_t = time.perf_counter()

        # 声明和加载参数
        self._declare_params()
        self._load_params()

        # 加载 HSV 颜色配置
        self._load_hsv_config()

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

        # 车身控制定时器
        self._body_timer = self.create_timer(
            1.0 / max(self._body_rate, 1.0),
            self._body_tick,
        )
        self._target_lost_timer = self.create_timer(
            0.2, self._check_target_lost_timeout)

        # 可视化窗口（show_debug_window 为真时开启，无鼠标/键盘交互，仅显示检测结果）
        if self._show_win:
            cv.namedWindow(self._win_name, cv.WINDOW_NORMAL)
            cv.resizeWindow(self._win_name, self._win_w, self._win_h)

        self.get_logger().info(
            f"Dify 颜色跟随节点已启动 (No-PTZ) | 颜色={self._color_name} | "
            f"HSV_min={self._hsv_min} HSV_max={self._hsv_max} | 窗口={'开' if self._show_win else '关'}")

    def _declare_params(self) -> None:
        """声明参数"""
        # 颜色选择（Dify 必需参数）
        self.declare_parameter("color", "red")  # red/green/blue/yellow
        self.declare_parameter("hsv_config_file",
                               "~/microros_car_v2_ws/config/color_hsv.txt")

        # 话题
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("finished_topic", "/visual_follow_finished")

        # 图像尺寸
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)

        # 车身角速度控制
        self.declare_parameter("angular_kp", 0.005)
        self.declare_parameter("angular_kd", 0.002)
        self.declare_parameter("deadband_x_pixel", 30.0)
        self.declare_parameter("max_angular_z", 0.8)
        self.declare_parameter("reverse_body_direction", False)

        # 距离跟随（按目标外接圆半径 PD 控制前进/后退）
        self.declare_parameter("enable_distance_follow", True)
        self.declare_parameter("target_radius", 60.0)
        self.declare_parameter("distance_kp", 0.008)
        self.declare_parameter("distance_kd", 0.003)
        self.declare_parameter("max_linear_x", 0.12)
        self.declare_parameter("distance_tolerance_r", 12.0)
        self.declare_parameter("reverse_linear_direction", False)

        # 检测参数
        self.declare_parameter("min_target_radius", 10.0)
        self.declare_parameter("min_target_area", 500.0)
        self.declare_parameter("morph_kernel_size", 5)
        self.declare_parameter("lost_frame_threshold", 5)
        self.declare_parameter("target_lost_timeout_sec", 10.0)

        # 控制频率
        self.declare_parameter("body_control_rate", 10.0)

        # 可视化窗口（无鼠标/键盘交互，仅显示检测结果）
        self.declare_parameter("show_debug_window", False)
        self.declare_parameter("window_name", "color_follow_dify")
        self.declare_parameter("window_width", 640)
        self.declare_parameter("window_height", 480)
        self.declare_parameter("overlay_font_scale", 0.38)
        self.declare_parameter("overlay_line_height", 16)

    def _load_params(self) -> None:
        """加载参数"""
        # 颜色名由 Dify 通过 color 参数传入（red/green/blue/yellow）
        self._color_name = str(self.get_parameter("color").value).strip().lower()

        self._hsv_file = os.path.expanduser(
            str(self.get_parameter("hsv_config_file").value))

        self._img_topic = str(self.get_parameter("image_topic").value)
        self._cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._finished_topic = str(self.get_parameter("finished_topic").value)

        self._img_w = int(self.get_parameter("image_width").value)
        self._img_h = int(self.get_parameter("image_height").value)

        self._ang_kp = float(self.get_parameter("angular_kp").value)
        self._ang_kd = float(self.get_parameter("angular_kd").value)
        self._db_x = float(self.get_parameter("deadband_x_pixel").value)
        self._max_ang = float(self.get_parameter("max_angular_z").value)
        self._rev_body = bool(self.get_parameter("reverse_body_direction").value)

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

        self._body_rate = float(self.get_parameter("body_control_rate").value)

        # 可视化窗口参数
        self._show_win = bool(self.get_parameter("show_debug_window").value)
        self._win_name = str(self.get_parameter("window_name").value)
        self._win_w = int(self.get_parameter("window_width").value)
        self._win_h = int(self.get_parameter("window_height").value)
        self._font = float(self.get_parameter("overlay_font_scale").value)
        self._lh = int(self.get_parameter("overlay_line_height").value)

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

    def _on_image(self, msg: Image) -> None:
        """图像回调 - HSV 检测"""
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if frame is None:
                return

            if self._img_w > 0 and self._img_h > 0:
                frame = cv.resize(frame, (self._img_w, self._img_h))

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

            # 查找轮廓
            contours, _ = cv.findContours(
                binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

            if not contours:
                with self._tgt_lock:
                    self._lost_frames += 1
                    if self._lost_frames >= self._lost_threshold:
                        self._has_target = False
                self._render_window(frame)
                return

            # 找最大轮廓
            max_contour = max(contours, key=cv.contourArea)
            area = cv.contourArea(max_contour)

            if area < self._min_area:
                with self._tgt_lock:
                    self._lost_frames += 1
                    if self._lost_frames >= self._lost_threshold:
                        self._has_target = False
                self._render_window(frame)
                return

            # 外接圆
            (cx, cy), radius = cv.minEnclosingCircle(max_contour)

            if radius < self._min_r:
                with self._tgt_lock:
                    self._lost_frames += 1
                    if self._lost_frames >= self._lost_threshold:
                        self._has_target = False
                self._render_window(frame)
                return

            # 更新目标位置（含外接圆半径，供距离跟随使用）
            with self._tgt_lock:
                self._has_target = True
                self._target_x = float(cx)
                self._target_r = float(radius)
                self._lost_frames = 0
                self._last_target_seen_at = time.monotonic()

            # 可视化：画检测圆
            if self._show_win:
                cv.circle(frame, (int(cx), int(cy)), int(radius), (255, 0, 255), 2)
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

    def _body_tick(self) -> None:
        """车身控制循环 - PD 控制角速度保持目标在中心"""
        with self._tgt_lock:
            tx, ok = self._target_x, self._has_target

        if not ok or tx is None:
            # 目标丢失，停止车身
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self._pub_vel.publish(cmd)
            self._prev_s1 = 0.0
            self._prev_body_t = 0.0
            return

        # 计算像素误差
        icx = self._img_w * 0.5
        pex = tx - icx

        # 时间步长
        n = time.perf_counter()
        dt = n - self._prev_body_t if self._prev_body_t > 0.0 else 1.0 / max(self._body_rate, 1.0)
        dt = max(dt, 1e-3)

        # 死区：仅归零角速度，不 return，距离跟随（前进/后退）继续执行
        if abs(pex) < self._db_x:
            pex = 0.0

        # PD 控制（角速度）
        d_pex = (pex - self._prev_s1) / dt
        angular_z = self._ang_kp * pex + self._ang_kd * d_pex

        # 方向 + 限幅
        dir_body = -1.0 if self._rev_body else 1.0
        angular_z = clamp(dir_body * angular_z, -self._max_ang, self._max_ang)

        # ── 线性距离跟随 PD（按目标外接圆半径控制前进/后退）──
        with self._tgt_lock:
            r = self._target_r
        if self._enable_dist and r is not None and r > 0:
            r_err = self._target_radius - r       # >0 太远需前进
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

        # 发布
        cmd = Twist()
        cmd.linear.x = float(lx)
        cmd.angular.z = angular_z
        self._pub_vel.publish(cmd)

        # 更新状态
        self._prev_s1 = pex
        self._prev_body_t = n

    def _render_window(self, frame: np.ndarray) -> None:
        """可视化窗口（仅显示，无交互）。show_debug_window 为假时直接返回。"""
        if not self._show_win:
            return
        iw, ih = frame.shape[1], frame.shape[0]
        icx = iw // 2
        # 画面竖直中心线 + 死区带
        cv.line(frame, (icx, 0), (icx, ih), (0, 255, 255), 1)
        cv.line(frame, (icx - int(self._db_x), 0), (icx - int(self._db_x), ih), (0, 180, 255), 1)
        cv.line(frame, (icx + int(self._db_x), 0), (icx + int(self._db_x), ih), (0, 180, 255), 1)
        with self._tgt_lock:
            ok = self._has_target
            tx = self._target_x
            tr = self._target_r
            lf = self._lost_frames
        tgt_txt = f"Target:{int(tx)}" if ok and tx is not None else f"Target:None(lost{lf}/{self._lost_threshold})"
        dist_txt = f"Dist r:{int(tr)}/{int(self._target_radius)}px" if tr else "Dist r:--"
        lines = [
            f"Color:{self._color_name} FPS:{int(self._fps)} {tgt_txt}",
            f"{dist_txt} dist_follow:{'on' if self._enable_dist else 'off'}",
        ]
        for i, t in enumerate(lines):
            cv.putText(frame, t, (8, 16 + i * self._lh), cv.FONT_HERSHEY_SIMPLEX,
                       self._font, (0, 255, 255), 1, cv.LINE_AA)
        cv.imshow(self._win_name, frame)
        cv.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ColorFollowDifyNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"节点错误: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
