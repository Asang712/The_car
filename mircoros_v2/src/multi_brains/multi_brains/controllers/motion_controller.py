# -*- coding: utf-8 -*-
"""Chassis velocity control for differential-drive motion.

秒数指令使用时间基控制；米/厘米指令使用 ``/odom_raw`` 的平面位移闭环，
避免把未经实测的“速度 × 时间”当作实际运动距离。
"""

import math
import threading
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


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
        self.max_distance = float(motion_config.get("max_distance_m", 5.0))
        self.odom_distance_scale = float(
            motion_config.get("odom_distance_scale", 1.0)
        )
        self.distance_tolerance = float(
            motion_config.get("distance_tolerance_m", 0.03)
        )
        self.distance_timeout = float(
            motion_config.get("distance_timeout_sec", 60.0)
        )
        self.odom_wait_timeout = float(
            motion_config.get("odom_wait_timeout_sec", 2.0)
        )
        self.odom_stale_timeout = float(
            motion_config.get("odom_stale_timeout_sec", 1.0)
        )
        self.no_progress_timeout = float(
            motion_config.get("distance_no_progress_timeout_sec", 4.0)
        )
        push_config = (config or {}).get("PUSH", {})
        self.push_enabled = bool(push_config.get("enabled", True))
        self.push_distance = float(push_config.get("distance_m", 0.10))
        self.push_speed = float(push_config.get("speed", 0.20))
        self.push_stall_timeout = float(
            push_config.get("stall_timeout_sec", 0.6)
        )
        self.push_timeout = float(push_config.get("timeout_sec", 5.0))
        self.push_contact_distance = float(
            push_config.get("contact_distance_m", 0.50)
        )
        self.push_contact_timeout = float(
            push_config.get("contact_timeout_sec", 15.0)
        )
        self.push_min_progress = float(
            push_config.get("min_progress_m", 0.03)
        )

        self._odom_lock = threading.Lock()
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_time = 0.0
        odom_topic = motion_config.get("odom_topic", "/odom_raw")
        self.odom_sub = node.create_subscription(
            Odometry,
            odom_topic,
            self._odom_callback,
            QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
            ),
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

    # ---- 前进 / 后退（里程计距离闭环）-----------------------------

    def execute_move_forward_distance(
        self, distance: str = "1.0", speed: str = ""
    ) -> bool:
        """按 ``/odom_raw`` 的位移前进指定米数。"""
        return self._execute_distance(distance, speed, 1.0, "前进")

    def execute_move_backward_distance(
        self, distance: str = "1.0", speed: str = ""
    ) -> bool:
        """按 ``/odom_raw`` 的位移后退指定米数。"""
        return self._execute_distance(distance, speed, -1.0, "后退")

    def execute_push(self, distance: str = "", speed: str = "") -> bool:
        """低速向前推进，走完设定距离或顶住目标即视为成功。

        没有力/碰撞传感器，用“已发速度但里程计无位移”作为顶住的判据。
        """
        if not self.push_enabled:
            self.node.get_logger().warn("[MOTION] 推动作未启用")
            return False
        return self._execute_distance(
            str(self.push_distance) if distance in ("", None) else distance,
            str(self.push_speed) if speed in ("", None) else speed,
            1.0,
            "推进",
            stall_timeout=self.push_stall_timeout,
            stall_is_success=True,
            timeout=self.push_timeout,
            min_progress_for_stall=self.push_min_progress,
        )

    def execute_push_contact(self) -> bool:
        """找物后继续向前推进，直到顶住目标或走满接触推进距离。

        与 :meth:`execute_push` 的区别：距离更长（默认 0.5m），且要求至少
        产生 ``min_progress_m`` 的位移才算“顶住”；完全推不动时判失败。
        """
        if not self.push_enabled:
            self.node.get_logger().warn("[MOTION] 推动作未启用")
            return False
        return self._execute_distance(
            str(self.push_contact_distance),
            str(self.push_speed),
            1.0,
            "接触推进",
            stall_timeout=self.push_stall_timeout,
            stall_is_success=True,
            timeout=self.push_contact_timeout,
            min_progress_for_stall=self.push_min_progress,
        )

    def _execute_distance(
        self,
        distance: str,
        speed: str,
        direction: float,
        label: str,
        stall_timeout=None,
        stall_is_success=False,
        timeout=None,
        min_progress_for_stall=0.0,
    ) -> bool:
        try:
            target = parse_positive_float(
                distance, "运动距离", self.max_distance
            )
            linear_speed = (
                self.default_linear_speed
                if speed == "" or speed is None
                else parse_positive_float(speed, "线速度", self.max_linear_speed)
            )
        except ValueError as error:
            self.node.get_logger().error(f"[MOTION] 参数无效: {error}")
            return False

        if (
            not math.isfinite(self.odom_distance_scale)
            or self.odom_distance_scale <= 0
        ):
            self.node.get_logger().error("[MOTION] odom_distance_scale 必须大于0")
            return False

        self.node.get_logger().info(
            f"[MOTION] 距离{label}: target={target:.3f}m, "
            f"speed={linear_speed}m/s, scale={self.odom_distance_scale:.3f}"
        )
        try:
            initial = self._wait_for_fresh_odom()
            if initial is None:
                if not self.node.should_interrupt():
                    self.node.get_logger().error(
                        "[MOTION] 未收到新鲜 /odom_raw，取消距离运动"
                    )
                return False

            start_x, start_y, _ = initial
            start_time = time.monotonic()
            deadline = start_time + min(
                timeout if timeout is not None else self.distance_timeout,
                self.max_duration,
            )
            stall_limit = (
                self.no_progress_timeout
                if stall_timeout is None
                else stall_timeout
            )
            last_progress_time = start_time
            best_distance = 0.0
            last_log_time = start_time
            tolerance = min(self.distance_tolerance, target * 0.2)
            twist = Twist()
            twist.linear.x = direction * linear_speed
            twist.angular.z = 0.0

            while time.monotonic() < deadline:
                if self.node.should_interrupt():
                    self.node.get_logger().warn(f"[MOTION] 距离{label}被中断")
                    return False

                now = time.monotonic()
                current = self._odom_snapshot()
                if (
                    current is None
                    or now - current[2] > self.odom_stale_timeout
                ):
                    self.node.get_logger().error(
                        "[MOTION] 里程计中断，立即停止距离运动"
                    )
                    return False

                raw_distance = math.hypot(
                    current[0] - start_x, current[1] - start_y
                )
                measured_distance = raw_distance * self.odom_distance_scale
                if measured_distance >= target - tolerance:
                    self.node.get_logger().info(
                        f"[MOTION] 距离{label}完成: "
                        f"odom={measured_distance:.3f}m, "
                        f"raw={raw_distance:.3f}m, "
                        f"耗时={now - start_time:.2f}s"
                    )
                    return True

                if measured_distance > best_distance + 0.005:
                    best_distance = measured_distance
                    last_progress_time = now
                elif now - last_progress_time > stall_limit:
                    if (
                        stall_is_success
                        and measured_distance >= min_progress_for_stall
                    ):
                        self.node.get_logger().info(
                            f"[MOTION] 距离{label}已顶住目标: "
                            f"odom={measured_distance:.3f}m"
                        )
                        return True
                    if stall_is_success:
                        self.node.get_logger().error(
                            f"[MOTION] 距离{label}推不动: "
                            f"{stall_limit:.1f}s内仅移动"
                            f"{measured_distance:.3f}m"
                        )
                        return False
                    self.node.get_logger().error(
                        f"[MOTION] {stall_limit:.1f}s内无明显位移，"
                        f"停止运动: odom={measured_distance:.3f}m"
                    )
                    return False

                if now - last_log_time >= 1.0:
                    self.node.get_logger().info(
                        f"[MOTION] 距离{label}进度: "
                        f"{measured_distance:.3f}/{target:.3f}m"
                    )
                    last_log_time = now

                self._publish_twist(twist)
                time.sleep(0.05)

            self.node.get_logger().error(
                f"[MOTION] 距离{label}超时: {best_distance:.3f}/{target:.3f}m"
            )
            return False
        finally:
            self.node.execute_stop(recenter_camera=False)

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

        start_time = time.monotonic()
        while time.monotonic() - start_time < duration:
            if self.node.should_interrupt():
                self.node.execute_stop(recenter_camera=False)
                return False
            self._publish_twist(twist)
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

        start_time = time.monotonic()
        while time.monotonic() - start_time < duration:
            if self.node.should_interrupt():
                self.node.execute_stop(recenter_camera=False)
                return False
            self._publish_twist(twist)
            time.sleep(0.1)

        self.node.execute_stop(recenter_camera=False)
        return True

    # ---- 里程计与发布 ---------------------------------------------

    def _odom_callback(self, msg: Odometry):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        if not math.isfinite(x) or not math.isfinite(y):
            return
        with self._odom_lock:
            self._odom_x = x
            self._odom_y = y
            self._odom_time = time.monotonic()

    def _odom_snapshot(self):
        with self._odom_lock:
            if self._odom_time <= 0.0:
                return None
            return self._odom_x, self._odom_y, self._odom_time

    def _wait_for_fresh_odom(self):
        deadline = time.monotonic() + max(0.0, self.odom_wait_timeout)
        while True:
            now = time.monotonic()
            snapshot = self._odom_snapshot()
            if (
                snapshot is not None
                and now - snapshot[2] <= self.odom_stale_timeout
            ):
                return snapshot
            if now >= deadline or self.node.should_interrupt():
                return None
            time.sleep(0.02)

    def _publish_twist(self, twist: Twist):
        started = time.monotonic()
        self.cmd_vel_pub.publish(twist)
        elapsed = time.monotonic() - started
        if elapsed > 0.5:
            self.node.get_logger().warn(
                f"[MOTION] cmd_vel发布耗时异常: {elapsed:.3f}s"
            )

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
