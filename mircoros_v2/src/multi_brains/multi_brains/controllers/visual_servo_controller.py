# -*- coding: utf-8 -*-
"""视觉伺服：颜色目标搜索 → 对准 → 靠近 → 到位。

对应 L1 演示级能力："帮我找红色的杯子"。
摄像头固定朝前（无云台），因此搜索阶段靠车体原地旋转扫描。

状态机::

    SEARCH  --发现目标-->  ALIGN  --对准-->  APPROACH  --够近-->  ARRIVED
       ^                      |                  |
       +----- 目标丢失 --------+------------------+

所有速度都经过限幅，任何异常退出路径都会先发零速度。
"""

import math
import time

import cv2
import numpy as np
from geometry_msgs.msg import Twist


# HSV 颜色范围：OpenCV 的 H 取值 0~179，红色在首尾各有一段
DEFAULT_COLOR_RANGES = {
    "red": [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (180, 255, 255))],
    "orange": [((10, 100, 100), (22, 255, 255))],
    "yellow": [((22, 100, 100), (35, 255, 255))],
    "green": [((40, 70, 70), (85, 255, 255))],
    "blue": [((95, 80, 70), (130, 255, 255))],
    "purple": [((130, 60, 60), (165, 255, 255))],
}

# 中文/英文颜色词 → 标准颜色名，按子串匹配（"红色的杯子" 也能命中）
COLOR_KEYWORDS = (
    ("红", "red"),
    ("橙", "orange"),
    ("黄", "yellow"),
    ("绿", "green"),
    ("蓝", "blue"),
    ("紫", "purple"),
)

STATE_SEARCH = "SEARCH"
STATE_ALIGN = "ALIGN"
STATE_APPROACH = "APPROACH"
STATE_ARRIVED = "ARRIVED"


def clamp(value, low, high):
    return max(low, min(high, value))


class VisualServoController:
    """基于颜色检测的视觉伺服控制器。"""

    def __init__(self, node, cmd_vel_pub, config=None):
        self.node = node
        self.cmd_vel_pub = cmd_vel_pub
        config = config or {}
        servo = config.get("VISION_SERVO", {})

        self.enabled = bool(servo.get("enabled", True))
        self.control_rate = float(servo.get("control_rate_hz", 10.0))
        self.search_angular_speed = float(servo.get("search_angular_speed", 0.4))
        self.align_deadband_px = float(servo.get("align_deadband_px", 40.0))
        self.align_kp = float(servo.get("align_kp", 0.004))
        self.max_angular_speed = float(servo.get("max_angular_speed", 0.5))
        self.min_target_area = float(servo.get("min_target_area", 300.0))
        self.min_target_radius = float(servo.get("min_target_radius", 8.0))
        self.target_radius_px = float(servo.get("target_radius_px", 70.0))
        self.approach_kp = float(servo.get("approach_kp", 0.004))
        self.max_linear_speed = float(servo.get("max_linear_speed", 0.12))
        self.arrive_tolerance_px = float(servo.get("arrive_tolerance_px", 12.0))
        self.detect_confirm_frames = max(
            1, int(servo.get("detect_confirm_frames", 2))
        )
        self.lost_timeout = float(servo.get("lost_timeout_sec", 1.0))
        self.search_timeout = float(servo.get("search_timeout_sec", 40.0))
        self.task_timeout = float(servo.get("task_timeout_sec", 90.0))
        self.morph_kernel = int(servo.get("morph_kernel_size", 5))

        # 允许在 YAML 里覆盖/扩展颜色范围：
        # colors: {red: [[0,120,70,10,255,255], [170,120,70,180,255,255]]}
        self.color_ranges = dict(DEFAULT_COLOR_RANGES)
        for name, ranges in (servo.get("colors") or {}).items():
            parsed = []
            for item in ranges:
                if len(item) == 6:
                    parsed.append((tuple(int(v) for v in item[:3]),
                                   tuple(int(v) for v in item[3:])))
            if parsed:
                self.color_ranges[str(name).lower()] = parsed

        self._last_seq = -1
        self._last_detection = None
        self._last_detection_time = 0.0

    # ---- 对外动作入口 ---------------------------------------------

    def execute_find_object(self, target: str = "red") -> bool:
        """搜索并靠近指定颜色的目标，到位返回 True。"""
        if not self.enabled:
            self.node.get_logger().warn("[SERVO] 视觉伺服未启用")
            return False

        color = self._resolve_color(target)
        if color is None:
            self.node.get_logger().error(f"[SERVO] 不支持的目标颜色: {target}")
            return False

        ranges = self.color_ranges[color]
        self.node.get_logger().info(f"[SERVO] 开始寻找目标: {color} (原始输入: {target})")

        state = STATE_SEARCH
        search_dir = 1.0
        confirmed = 0
        arrived_since = None
        lost_since = None
        start_time = time.monotonic()
        search_start = start_time
        deadline = start_time + self.task_timeout
        period = 1.0 / max(self.control_rate, 1.0)

        try:
            while time.monotonic() < deadline:
                if self.node.should_interrupt():
                    self.node.get_logger().info("[SERVO] 收到中断，停止视觉伺服")
                    self._stop()
                    return False

                now = time.monotonic()
                detection = self._update_detection(ranges)
                fresh = self._is_fresh(now)

                if state == STATE_SEARCH:
                    if detection is not None and fresh:
                        confirmed += 1
                        if confirmed >= self.detect_confirm_frames:
                            state = STATE_ALIGN
                            confirmed = 0
                            self._stop()
                            self.node.get_logger().info("[SERVO] 发现目标，进入对准")
                            continue
                        self._publish(0.0, 0.0)
                    else:
                        confirmed = 0
                        self._publish(0.0, self.search_angular_speed * search_dir)

                    if now - search_start > self.search_timeout:
                        self.node.get_logger().warn("[SERVO] 搜索超时，未发现目标")
                        self._stop()
                        return False

                elif state == STATE_ALIGN:
                    if detection is None or not fresh:
                        if lost_since is None:
                            lost_since = now
                        elif now - lost_since > self.lost_timeout:
                            self.node.get_logger().info("[SERVO] 目标丢失，重新搜索")
                            state = STATE_SEARCH
                            search_dir = -search_dir
                            search_start = now
                            lost_since = None
                            confirmed = 0
                        self._publish(0.0, 0.0)
                    else:
                        lost_since = None
                        cx, _cy, _radius = detection
                        error = cx - self._frame_width() * 0.5
                        if abs(error) <= self.align_deadband_px:
                            state = STATE_APPROACH
                            self._stop()
                            self.node.get_logger().info("[SERVO] 已完成对准，开始靠近")
                            continue
                        angular = clamp(
                            error * self.align_kp,
                            -self.max_angular_speed,
                            self.max_angular_speed,
                        )
                        self._publish(0.0, angular)

                elif state == STATE_APPROACH:
                    if detection is None or not fresh:
                        if lost_since is None:
                            lost_since = now
                        elif now - lost_since > self.lost_timeout:
                            self.node.get_logger().info("[SERVO] 靠近中目标丢失，重新搜索")
                            state = STATE_SEARCH
                            search_dir = -search_dir
                            search_start = now
                            lost_since = None
                            confirmed = 0
                        self._publish(0.0, 0.0)
                    else:
                        lost_since = None
                        cx, _cy, radius = detection
                        error = cx - self._frame_width() * 0.5
                        angular = clamp(
                            error * self.align_kp,
                            -self.max_angular_speed,
                            self.max_angular_speed,
                        )
                        radius_error = self.target_radius_px - radius

                        if abs(radius_error) <= self.arrive_tolerance_px:
                            if arrived_since is None:
                                arrived_since = now
                            elif now - arrived_since > 0.5:
                                state = STATE_ARRIVED
                                continue
                        else:
                            arrived_since = None

                        if abs(error) > 2.0 * self.align_deadband_px:
                            linear = 0.0
                        else:
                            linear = clamp(
                                radius_error * self.approach_kp,
                                -0.05,
                                self.max_linear_speed,
                            )
                        self._publish(linear, angular)

                elif state == STATE_ARRIVED:
                    self._stop()
                    self.node.get_logger().info(
                        f"[SERVO] 已到达目标 {color}，自行走耗时 "
                        f"{time.monotonic() - start_time:.1f}s"
                    )
                    return True

                time.sleep(period)

            self.node.get_logger().warn("[SERVO] 任务超时，未能在限定时间内到位")
            self._stop()
            return False
        finally:
            if self.node.should_interrupt() or True:
                self._stop()

    # ---- 检测 -----------------------------------------------------

    def _update_detection(self, ranges):
        """在最新一帧上做颜色检测；图像未更新时复用上次结果。"""
        frame = self._latest_frame()
        if frame is None:
            self._last_detection = None
            return None

        detection = self._detect_blob(frame, ranges)
        if detection is not None:
            self._last_detection = detection
            self._last_detection_time = time.monotonic()
        else:
            self._last_detection = None
            self._last_detection_time = 0.0
        return detection

    def _latest_frame(self):
        lock = getattr(self.node, "latest_image_lock", None)
        if lock is None:
            return None
        with lock:
            image = getattr(self.node, "latest_cv_image", None)
            seq = getattr(self.node, "latest_image_seq", 0)
        if image is None:
            return None
        self._last_seq = seq
        return image

    def _detect_blob(self, frame, ranges):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            mask = cv2.bitwise_or(
                mask,
                cv2.inRange(hsv, np.array(lower, dtype=np.uint8),
                            np.array(upper, dtype=np.uint8)),
            )

        kernel_size = max(1, self.morph_kernel)
        if kernel_size % 2 == 0:
            kernel_size += 1
        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (kernel_size, kernel_size)
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return None

        best = max(contours, key=cv2.contourArea)
        if cv2.contourArea(best) < self.min_target_area:
            return None

        (cx, cy), radius = cv2.minEnclosingCircle(best)
        if radius < self.min_target_radius:
            return None
        return float(cx), float(cy), float(radius)

    def _is_fresh(self, now):
        return (
            self._last_detection is not None
            and now - self._last_detection_time <= self.lost_timeout
        )

    def _frame_width(self):
        frame = self._latest_frame()
        if frame is None:
            return 640.0
        return float(frame.shape[1])

    # ---- 目标解析 / 输出 ------------------------------------------

    def _resolve_color(self, target):
        text = str(target or "").strip().lower()
        if text in self.color_ranges:
            return text
        for keyword, color in COLOR_KEYWORDS:
            if keyword in text:
                return color
        for color in self.color_ranges:
            if color in text:
                return color
        return None

    def _publish(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(twist)

    def _stop(self):
        self._publish(0.0, 0.0)
