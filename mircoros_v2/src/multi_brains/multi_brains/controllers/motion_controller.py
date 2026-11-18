# -*- coding: utf-8 -*-
"""Chassis velocity control for differential-drive motion.

Owns the timed ``Twist`` publication loops for forward/backward/turn actions.
Interrupt and stop semantics are delegated back to the owning node so the
"latest command wins" policy and KCF-tracking teardown stay centralized.
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
        self.max_duration = float(motion_config.get("max_duration_sec", 60.0))
        self.max_turn_angle = float(motion_config.get("max_turn_angle_deg", 720.0))

    def execute_move_forward(self, speed: str = "0.2", duration: str = "1.0") -> bool:
        """Drive forward at ``speed`` m/s for ``duration`` seconds."""
        values = self._validate_linear_action(speed, duration)
        if values is None:
            return False
        linear_speed, run_duration = values
        self.node.get_logger().info(
            f"[MOTION] 前进: speed={linear_speed}m/s, duration={run_duration}s, "
            f"距离≈{linear_speed * run_duration:.2f}m"
        )
        return self._execute_velocity(linear_speed, 0.0, run_duration)

    def execute_move_backward(self, speed: str = "0.2", duration: str = "1.0") -> bool:
        """Drive backward at ``speed`` m/s for ``duration`` seconds."""
        values = self._validate_linear_action(speed, duration)
        if values is None:
            return False
        linear_speed, run_duration = values
        self.node.get_logger().info(
            f"[MOTION] 后退: speed={linear_speed}m/s, duration={run_duration}s, "
            f"距离≈{linear_speed * run_duration:.2f}m"
        )
        return self._execute_velocity(-linear_speed, 0.0, run_duration)

    def execute_turn_left(self, angle: str = "90", speed: str = "0.5") -> bool:
        """Turn left in place by ``angle`` degrees at ``speed`` rad/s."""
        values = self._validate_turn_action(angle, speed)
        if values is None:
            return False
        turn_angle, angular_speed = values
        self.node.get_logger().info(
            f"[MOTION] 左转: angle={turn_angle}°, speed={angular_speed}rad/s"
        )
        return self._execute_turn(turn_angle, angular_speed, 1.0)

    def execute_turn_right(self, angle: str = "90", speed: str = "0.5") -> bool:
        """Turn right in place by ``angle`` degrees at ``speed`` rad/s."""
        values = self._validate_turn_action(angle, speed)
        if values is None:
            return False
        turn_angle, angular_speed = values
        self.node.get_logger().info(
            f"[MOTION] 右转: angle={turn_angle}°, speed={angular_speed}rad/s"
        )
        return self._execute_turn(turn_angle, angular_speed, -1.0)

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

    def _validate_linear_action(self, speed, duration):
        try:
            return (
                parse_positive_float(speed, "线速度", self.max_linear_speed),
                parse_positive_float(duration, "运动时长", self.max_duration),
            )
        except ValueError as error:
            self.node.get_logger().error(f"[MOTION] 参数无效: {error}")
            return None

    def _validate_turn_action(self, angle, speed):
        try:
            turn_angle = parse_positive_float(angle, "转向角度", self.max_turn_angle)
            angular_speed = parse_positive_float(
                speed, "角速度", self.max_angular_speed
            )
            turn_duration = math.radians(turn_angle) / angular_speed
            if not math.isfinite(turn_duration) or turn_duration > self.max_duration:
                raise ValueError(
                    f"预计转向时长超过安全上限{self.max_duration}: "
                    f"{turn_duration:.2f}"
                )
            return turn_angle, angular_speed
        except ValueError as error:
            self.node.get_logger().error(f"[MOTION] 参数无效: {error}")
            return None

    def _execute_turn(self, angle_deg: float, speed: float, direction: float) -> bool:
        """Turn in place for the duration implied by angle and speed."""
        angle_rad = math.radians(angle_deg)
        angular_speed = speed * direction
        duration = abs(angle_rad / angular_speed)

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_speed

        start_time = time.time()
        while time.time() - start_time < duration:
            if self.node.should_interrupt():
                self.node.execute_stop(recenter_camera=False)
                return False
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)

        self.node.execute_stop(recenter_camera=False)
        return True

