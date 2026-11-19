# -*- coding: utf-8 -*-
"""Chassis velocity control for differential-drive motion.

所有动作统一为 **时间基控制**：参数顺序都是 ``(duration, speed)``，
duration 单位是秒，speed 是可选的覆盖默认速度（不传就用默认值）。
单参数调用（``move_forward(3.0)``）自动视为 duration，speed 取默认值。

这样 System Prompt 里写的 ``move_forward(时长秒)`` 就能直接对应代码签名，
LLM 不会再把"速度"当成"时长"用。
"""

import math
import time

from geometry_msgs.msg import Twist


def parse_positive_float(value, name: str, maximum: float) -> float:
    """Return a finite positive float no greater than ``maximum``.

    Motion parameters originate in model-generated action strings, so they
    must be validated before any value reaches a physical controller.
    """
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name}不是有效数字: {value}") from error

    if not math.isfinite(number):
        raise ValueError(f"{name}必须是有限数字: {value}")
    if number <= 0.0:
        raise ValueError(f"{name}必须大于0: {number}")
    if number > maximum:
        raise ValueError(f"{name}超过安全上限{maximum}: {number}")
    return number


class MotionController:
    """Publish timed chassis velocities on ``/cmd_vel``."""

    def __init__(self, node, cmd_vel_pub, config=None):
        self.node = node
        self.cmd_vel_pub = cmd_vel_pub
        motion_config = (config or {}).get("MOTION", {})
        self.max_linear_speed = float(motion_config.get("max_linear_speed", 0.5))
        self.max_angular_speed = float(motion_config.get("max_angular_speed", 1.5))
        self.max_duration = float(motion_config.get("max_duration_sec", 120.0))
        self.max_turn_angle = float(motion_config.get("max_turn_angle_deg", 720.0))
        # 默认速度，LLM 不传第二参数时用
        self.default_linear_speed = float(
            motion_config.get("default_linear_speed", 0.2)
        )
        self.default_angular_speed = float(
            motion_config.get("default_angular_speed", 0.6)
        )

    # ---- 前进 / 后退（时间基）-------------------------------------

    def execute_move_forward(self, duration: str = "1.0", speed: str = "") -> bool:
        """前进: ``duration`` 秒, ``speed`` m/s（空则用默认 0.2m/s）。

        单参数调用如 ``move_forward(5.0)`` → 前进 5 秒, 速度 0.2m/s。
        """
        values = self._validate_motion(speed, duration, "线速度")
        if values is None:
            return False
        linear_speed, run_duration = values
        self.node.get_logger().info(
            f"[MOTION] 前进: speed={linear_speed}m/s, duration={run_duration}s, "
            f"距离≈{linear_speed * run_duration:.2f}m"
        )
        return self._execute_velocity(linear_speed, 0.0, run_duration)

    def execute_move_backward(self, duration: str = "1.0", speed: str = "") -> bool:
        """后退: 同前进，速度反向。"""
        values = self._validate_motion(speed, duration, "线速度")
        if values is None:
            return False
        linear_speed, run_duration = values
        self.node.get_logger().info(
            f"[MOTION] 后退: speed={linear_speed}m/s, duration={run_duration}s, "
            f"距离≈{linear_speed * run_duration:.2f}m"
        )
        return self._execute_velocity(-linear_speed, 0.0, run_duration)

    # ---- 左转 / 右转（时间基）-------------------------------------

    def execute_turn_left(self, duration: str = "2.0", speed: str = "") -> bool:
        """原地左转: ``duration`` 秒, ``speed`` rad/s（空则默认 0.6）。

        单参数 ``turn_left(3.0)`` → 左转 3 秒（约 103° at 0.6rad/s）。
        """
        values = self._validate_motion(speed, duration, "角速度")
        if values is None:
            return False
        angular_speed, run_duration = values
        estimated_deg = math.degrees(angular_speed * run_duration)
        self.node.get_logger().info(
            f"[MOTION] 左转: speed={angular_speed}rad/s, duration={run_duration}s, "
            f"角度≈{estimated_deg:.0f}°"
        )
        return self._execute_turn_time(angular_speed, run_duration, 1.0)

    def execute_turn_right(self, duration: str = "2.0", speed: str = "") -> bool:
        """原地右转: 同左转，方向反向。"""
        values = self._validate_motion(speed, duration, "角速度")
        if values is None:
            return False
        angular_speed, run_duration = values
        estimated_deg = math.degrees(angular_speed * run_duration)
        self.node.get_logger().info(
            f"[MOTION] 右转: speed={angular_speed}rad/s, duration={run_duration}s, "
            f"角度≈{estimated_deg:.0f}°"
        )
        return self._execute_turn_time(angular_speed, run_duration, -1.0)

    # ---- 底层执行 ------------------------------------------------

    def _execute_velocity(
        self, linear_x: float, angular_z: float, duration: float
    ) -> bool:
        """Publish a fixed Twist for ``duration`` seconds, honoring interrupts."""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z

        start_time = time.time()
        while time.time() - start_time < duration:
            if self.node.should_interrupt():
                self.node.execute_stop(recenter_camera=False)
                return False
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)

        self.node.execute_stop(recenter_camera=False)
        return True

    def _execute_turn_time(
        self, angular_speed: float, duration: float, direction: float
    ) -> bool:
        """原地转动 ``duration`` 秒。"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_speed * direction

        start_time = time.time()
        while time.time() - start_time < duration:
            if self.node.should_interrupt():
                self.node.execute_stop(recenter_camera=False)
                return False
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)

        self.node.execute_stop(recenter_camera=False)
        return True

    # ---- 参数校验 ------------------------------------------------

    def _validate_motion(self, speed_str, duration_str, speed_name):
        """统一参数校验。speed_str 为空 → 用默认速度。"""
        try:
            speed_max = (
                self.max_linear_speed
                if speed_name == "线速度"
                else self.max_angular_speed
            )
            speed_default = (
                self.default_linear_speed
                if speed_name == "线速度"
                else self.default_angular_speed
            )
            speed_val = (
                speed_default
                if speed_str == "" or speed_str is None
                else parse_positive_float(speed_str, speed_name, speed_max)
            )
            duration_val = parse_positive_float(
                duration_str, "运动时长", self.max_duration
            )
            return speed_val, duration_val
        except ValueError as error:
            self.node.get_logger().error(f"[MOTION] 参数无效: {error}")
            return None
