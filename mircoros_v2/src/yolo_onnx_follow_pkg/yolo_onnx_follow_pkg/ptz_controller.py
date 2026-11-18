# -*- coding: utf-8 -*-
"""Small local PTZ controller for the YOLO ONNX follow demo."""

import math
import time

from geometry_msgs.msg import Vector3


def clamp(value: float, min_value: float, max_value: float) -> float:
    """Clamp ``value`` to ``[min_value, max_value]``."""
    return max(min_value, min(max_value, value))


class PtzController:
    """Publish absolute pan/tilt servo angles on a Vector3 topic."""

    def __init__(self, node, config):
        self.node = node

        ptz_config = config.get("PTZ", {})
        self.ptz_angle_topic = ptz_config.get("angle_topic", "/cmd_ptz_angle")
        self.ptz_publish_count = int(ptz_config.get("publish_count", 1))
        self.ptz_publish_interval_sec = float(
            ptz_config.get("publish_interval_sec", 0.05)
        )
        self.ptz_initialize_on_startup = bool(
            ptz_config.get("initialize_on_startup", True)
        )
        self.ptz_startup_publish_count = max(
            0, int(ptz_config.get("startup_publish_count", 10))
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
        self.ptz_recenter_after_stop = bool(ptz_config.get("recenter_after_stop", True))
        self.ptz_settle_time_sec = float(ptz_config.get("settle_time_sec", 0.3))

        self.servo1_angle = self.servo1_initial_angle
        self.servo2_angle = self.servo2_initial_angle

        self.ptz_angle_pub = node.create_publisher(Vector3, self.ptz_angle_topic, 10)
        self._startup_ptz_init_timer = None
        self._startup_ptz_init_remaining = 0
        self.schedule_startup_init()

    def schedule_startup_init(self) -> None:
        if not self.ptz_initialize_on_startup or self.ptz_startup_publish_count <= 0:
            return

        self._startup_ptz_init_remaining = self.ptz_startup_publish_count
        self._startup_ptz_init_timer = self.node.create_timer(
            self.ptz_startup_publish_interval_sec,
            self._startup_ptz_init_tick,
        )
        self.node.get_logger().info(
            "[PTZ] startup init: "
            f"x={self.servo1_initial_angle:.1f}deg, "
            f"y={self.servo2_initial_angle:.1f}deg, "
            f"count={self.ptz_startup_publish_count}"
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
                "[PTZ] startup init done: "
                f"x={self.servo1_angle:.1f}deg, y={self.servo2_angle:.1f}deg"
            )

    def _cancel_startup_timer(self) -> None:
        if self._startup_ptz_init_timer is not None:
            self.node.destroy_timer(self._startup_ptz_init_timer)
            self._startup_ptz_init_timer = None

    def publish_servo_angles(
        self, servo1_angle=None, servo2_angle=None, repeat: bool = True
    ) -> bool:
        try:
            new_servo1_angle = (
                float(servo1_angle) if servo1_angle is not None else self.servo1_angle
            )
            new_servo2_angle = (
                float(servo2_angle) if servo2_angle is not None else self.servo2_angle
            )
        except (TypeError, ValueError) as error:
            self.node.get_logger().error(f"[PTZ] invalid servo angle: {error}")
            return False

        if not math.isfinite(new_servo1_angle) or not math.isfinite(new_servo2_angle):
            self.node.get_logger().error(
                "[PTZ] servo angles must be finite: "
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
            if hasattr(self.node, "should_interrupt") and self.node.should_interrupt():
                return False
            self.ptz_angle_pub.publish(msg)
            if publish_index + 1 < publish_count:
                time.sleep(max(0.0, self.ptz_publish_interval_sec))
        return True

    def recenter_after_stop(self) -> bool:
        if not self.ptz_recenter_after_stop:
            return True

        if hasattr(self.node, "publish_progress"):
            self.node.publish_progress("[STOP] PTZ recenter...")
        ok = self.publish_servo_angles(
            servo1_angle=self.servo1_initial_angle,
            servo2_angle=self.servo2_initial_angle,
        )
        if ok:
            time.sleep(max(0.0, self.ptz_settle_time_sec))
            self.node.get_logger().info(
                f"[STOP] PTZ centered: x={self.servo1_angle:.1f}deg, "
                f"y={self.servo2_angle:.1f}deg"
            )
        return ok
