# -*- coding: utf-8 -*-
"""Pan/tilt (PTZ) servo control.

Owns the two camera servos: publishing absolute angles on the PTZ topic,
clamping to configured limits, startup re-initialization, and the gesture
actions (shake/nod). Servo state (``servo1_angle`` / ``servo2_angle``) lives
here so the KCF follow controller can read the current horizontal angle when
deriving chassis turn commands.
"""

import math
import time

from geometry_msgs.msg import Vector3

from ..core.hardware_profile import CAR_TYPE_BASIC, has_ptz


def clamp(value: float, min_value: float, max_value: float) -> float:
    """Clamp ``value`` to ``[min_value, max_value]``."""
    return max(min_value, min(max_value, value))


class PtzController:
    """Control the two camera servos via a Vector3 PTZ topic."""

    def __init__(self, node, config, car_type: str):
        self.node = node
        self.car_type = car_type

        ptz_config = config.get("PTZ", {})
        self.ptz_angle_topic = ptz_config.get("angle_topic", "/cmd_ptz_angle")
        self.ptz_publish_count = int(ptz_config.get("publish_count", 5))
        self.ptz_publish_interval_sec = float(
            ptz_config.get("publish_interval_sec", 0.05)
        )
        self.ptz_initialize_on_startup = bool(
            ptz_config.get("initialize_on_startup", True)
        )
        self.ptz_startup_publish_count = max(
            0, int(ptz_config.get("startup_publish_count", 20))
        )
        self.ptz_startup_publish_interval_sec = max(
            0.02, float(ptz_config.get("startup_publish_interval_sec", 0.1))
        )
        self.servo1_min_angle = float(ptz_config.get("servo1_min_angle", -90.0))
        self.servo1_max_angle = float(ptz_config.get("servo1_max_angle", 90.0))
        self.servo2_min_angle = float(ptz_config.get("servo2_min_angle", -90.0))
        self.servo2_max_angle = float(ptz_config.get("servo2_max_angle", 20.0))
        self.servo1_initial_angle = float(ptz_config.get("servo1_initial_angle", 0.0))
        self.servo2_initial_angle = float(ptz_config.get("servo2_initial_angle", -45.0))
        self.servo_gesture_offset = max(
            0.0, float(ptz_config.get("gesture_offset", 30.0))
        )
        self.servo_gesture_delay_sec = max(
            0.0, float(ptz_config.get("gesture_delay_sec", 0.3))
        )
        self.servo_gesture_cycles = max(1, int(ptz_config.get("gesture_cycles", 2)))
        self.ptz_recenter_after_stop = bool(ptz_config.get("recenter_after_stop", True))
        self.ptz_settle_time_sec = float(ptz_config.get("settle_time_sec", 0.3))

        self.servo1_angle = self.servo1_initial_angle
        self.servo2_angle = self.servo2_initial_angle

        self.ptz_angle_pub = node.create_publisher(Vector3, self.ptz_angle_topic, 10)

        self._startup_ptz_init_timer = None
        self._startup_ptz_init_remaining = 0
        self.schedule_startup_init()

    # ---- startup re-initialization -------------------------------------

    def schedule_startup_init(self) -> None:
        """Publish initial PTZ angles repeatedly after node startup."""
        if not has_ptz(self.car_type) or not self.ptz_initialize_on_startup:
            return
        if self.ptz_startup_publish_count <= 0:
            return

        self._startup_ptz_init_remaining = self.ptz_startup_publish_count
        self._startup_ptz_init_timer = self.node.create_timer(
            self.ptz_startup_publish_interval_sec,
            self._startup_ptz_init_tick,
        )
        self.node.get_logger().info(
            "[PTZ] 启动初始化已启用: "
            f"x={self.servo1_initial_angle:.1f}°, "
            f"y={self.servo2_initial_angle:.1f}°, "
            f"count={self.ptz_startup_publish_count}, "
            f"interval={self.ptz_startup_publish_interval_sec:.2f}s"
        )

    def _startup_ptz_init_tick(self) -> None:
        if self._startup_ptz_init_remaining <= 0:
            self._cancel_startup_timer()
            return

        self.publish_servo_angles(
            servo1_angle=self.servo1_initial_angle,
            servo2_angle=self.servo2_initial_angle,
            repeat=False,
        )
        self._startup_ptz_init_remaining -= 1
        if self._startup_ptz_init_remaining <= 0:
            self._cancel_startup_timer()
            self.node.get_logger().info(
                "[PTZ] 启动舵机初始化完成: "
                f"x={self.servo1_angle:.1f}°, y={self.servo2_angle:.1f}°"
            )

    def _cancel_startup_timer(self) -> None:
        if self._startup_ptz_init_timer is not None:
            self.node.destroy_timer(self._startup_ptz_init_timer)
            self._startup_ptz_init_timer = None

    # ---- capability gating ---------------------------------------------

    def ensure_ptz_action(self, action: str) -> bool:
        """Validate that the active car has controllable camera servos."""
        if has_ptz(self.car_type):
            return True
        reason = (
            "当前车型不带云台和摄像头，没有可控制舵机"
            if self.car_type == CAR_TYPE_BASIC
            else "当前车型使用固定摄像头，没有可控制舵机"
        )
        self.node.reject_unsupported_action(action, reason)
        return False

    # ---- servo publishing ----------------------------------------------

    def publish_servo_angles(
        self, servo1_angle=None, servo2_angle=None, repeat: bool = True
    ) -> bool:
        """Publish servo 1 on Vector3.x and servo 2 on Vector3.y.

        Returns False if interrupted mid-publish.
        """
        try:
            new_servo1_angle = (
                float(servo1_angle) if servo1_angle is not None else self.servo1_angle
            )
            new_servo2_angle = (
                float(servo2_angle) if servo2_angle is not None else self.servo2_angle
            )
        except (TypeError, ValueError) as error:
            self.node.get_logger().error(f"[PTZ] 舵机角度格式无效: {error}")
            return False

        if not math.isfinite(new_servo1_angle) or not math.isfinite(new_servo2_angle):
            self.node.get_logger().error(
                "[PTZ] 舵机角度必须是有限数字: "
                f"x={new_servo1_angle}, y={new_servo2_angle}"
            )
            return False

        self.servo1_angle = clamp(
            new_servo1_angle,
            self.servo1_min_angle,
            self.servo1_max_angle,
        )
        self.servo2_angle = clamp(
            new_servo2_angle,
            self.servo2_min_angle,
            self.servo2_max_angle,
        )

        msg = Vector3()
        msg.x = float(self.servo1_angle)
        msg.y = float(self.servo2_angle)
        msg.z = 0.0

        publish_count = max(1, self.ptz_publish_count) if repeat else 1
        for publish_index in range(publish_count):
            if self.node.should_interrupt():
                return False
            self.ptz_angle_pub.publish(msg)
            if publish_index + 1 < publish_count:
                time.sleep(max(0.0, self.ptz_publish_interval_sec))
        return True

    # ---- recenter after stop -------------------------------------------

    def recenter_after_stop(self) -> bool:
        """Reset the camera to its initial angles after a stop, if enabled."""
        if not has_ptz(self.car_type) or not self.ptz_recenter_after_stop:
            return True

        self.node.publish_progress("[停止] 摄像头归中...")
        ok = self.publish_servo_angles(
            servo1_angle=self.servo1_initial_angle,
            servo2_angle=self.servo2_initial_angle,
        )
        if ok:
            time.sleep(self.ptz_settle_time_sec)
            self.node.get_logger().info(
                f"[STOP] 摄像头已归中: x={self.servo1_angle:.1f}°, "
                f"y={self.servo2_angle:.1f}°"
            )
        else:
            self.node.get_logger().warn("[STOP] 摄像头归中被中断")
        return ok

    # ---- action handlers -----------------------------------------------

    def execute_servo1_move(self, angle: str) -> bool:
        """Move servo 1, mapped to the horizontal Vector3.x axis."""
        if not self.ensure_ptz_action("servo1_move()"):
            return False
        try:
            target_angle = float(angle)
        except (TypeError, ValueError):
            self.node.get_logger().error(f"[SERVO] 1号舵机角度无效: {angle}")
            return False

        if not self.publish_servo_angles(servo1_angle=target_angle):
            return False
        self.node.get_logger().info(
            f"[SERVO] 1号舵机移动完成: x={self.servo1_angle:.1f}°"
        )
        return True

    def execute_servo2_move(self, angle: str) -> bool:
        """Move servo 2, mapped to the pitch Vector3.y axis."""
        if not self.ensure_ptz_action("servo2_move()"):
            return False
        try:
            target_angle = float(angle)
        except (TypeError, ValueError):
            self.node.get_logger().error(f"[SERVO] 2号舵机角度无效: {angle}")
            return False

        if not self.publish_servo_angles(servo2_angle=target_angle):
            return False
        self.node.get_logger().info(
            f"[SERVO] 2号舵机移动完成: y={self.servo2_angle:.1f}°"
        )
        return True

    def execute_servo_init(self) -> bool:
        """Reset both camera servos to their configured initial angles."""
        if not self.ensure_ptz_action("servo_init()"):
            return False
        if not self.publish_servo_angles(
            servo1_angle=self.servo1_initial_angle,
            servo2_angle=self.servo2_initial_angle,
        ):
            return False
        self.node.get_logger().info(
            f"[SERVO] 云台复位完成: x={self.servo1_angle:.1f}°, "
            f"y={self.servo2_angle:.1f}°"
        )
        return True

    def execute_servo_shake(self) -> bool:
        """Shake the camera horizontally with servo 1 and then reset it."""
        if not self.ensure_ptz_action("servo_shake()"):
            return False

        for _ in range(self.servo_gesture_cycles):
            for offset in (-self.servo_gesture_offset, self.servo_gesture_offset):
                if not self.publish_servo_angles(
                    servo1_angle=self.servo1_initial_angle + offset, repeat=False
                ):
                    return False
                time.sleep(self.servo_gesture_delay_sec)

        if not self.publish_servo_angles(
            servo1_angle=self.servo1_initial_angle,
            servo2_angle=self.servo2_initial_angle,
        ):
            return False
        self.node.get_logger().info("[SERVO] 摇头动作完成")
        return True

    def execute_servo_nod(self) -> bool:
        """Nod the camera vertically with servo 2 and then reset it."""
        if not self.ensure_ptz_action("servo_nod()"):
            return False

        for _ in range(self.servo_gesture_cycles):
            for offset in (-self.servo_gesture_offset, self.servo_gesture_offset):
                if not self.publish_servo_angles(
                    servo2_angle=self.servo2_initial_angle + offset, repeat=False
                ):
                    return False
                time.sleep(self.servo_gesture_delay_sec)

        if not self.publish_servo_angles(
            servo1_angle=self.servo1_initial_angle,
            servo2_angle=self.servo2_initial_angle,
        ):
            return False
        self.node.get_logger().info("[SERVO] 点头动作完成")
        return True

