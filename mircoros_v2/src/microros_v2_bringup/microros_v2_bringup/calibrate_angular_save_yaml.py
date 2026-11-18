#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Angular odometry calibration node based on /odom_raw.

This node does NOT use TF. It subscribes to nav_msgs/Odometry directly:
    default odom_topic = /odom_raw

Left/right turns are calibrated independently.

Parameters:
    turn_left:
        true  -> left turn, positive yaw / positive angular.z
        false -> right turn, negative yaw / negative angular.z

    odom_angular_left_scale_correction:
        scale used and saved for left turns

    odom_angular_right_scale_correction:
        scale used and saved for right turns

Workflow:
    ros2 launch microros_v2_bringup calibrate_angular.launch.py

    # Left turn calibration:
    ros2 param set /calibrate_angular_node turn_left true
    ros2 param set /calibrate_angular_node test_angle 360.0
    ros2 param set /calibrate_angular_node speed 0.4
    ros2 param set /calibrate_angular_node start_test true
    ros2 param set /calibrate_angular_node measured_angle 382.0
    ros2 param set /calibrate_angular_node save_calibration true

    # Right turn calibration:
    ros2 param set /calibrate_angular_node turn_left false
    ros2 param set /calibrate_angular_node test_angle 360.0
    ros2 param set /calibrate_angular_node speed 0.4
    ros2 param set /calibrate_angular_node start_test true
    ros2 param set /calibrate_angular_node measured_angle 376.0
    ros2 param set /calibrate_angular_node save_calibration true

Formula:
    new_selected_scale = test_start_selected_scale * measured_angle / abs(test_angle)
"""

from enum import Enum
from math import atan2, copysign, pi, radians

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from microros_v2_bringup.calibration_yaml_utils import (
    read_yaml_value,
    update_odom_calibration_yaml,
)


class CalibrationState(Enum):
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    WAIT_SAVE = "WAIT_SAVE"


class CalibrateAngularSaveYaml(Node):
    def __init__(self):
        super().__init__("calibrate_angular_node")
        self.cmd_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.cmd_vel = self.create_publisher(Twist, "cmd_vel", self.cmd_qos)

        # Raw odometry source.
        self.declare_parameter("odom_topic", "/odom_raw")

        # YAML output.
        self.declare_parameter("calibration_yaml_path", "odom_angular_calibration.yaml")
        self.declare_parameter("target_node_name", "microros_v2_patrol_node")

        # Motion test.
        self.declare_parameter("test_angle", 360.0)    # degrees, magnitude is used
        self.declare_parameter("speed", 0.4)           # rad/s, sign is controlled by turn_left
        self.declare_parameter("tolerance", 1.5)       # degrees
        self.declare_parameter("start_test", False)

        # Direction switch for rqt.
        # checked  -> left turn, positive yaw
        # unchecked -> right turn, negative yaw
        self.declare_parameter("turn_left", True)

        # Deprecated compatibility parameter. It is used only as fallback default.
        self.declare_parameter("odom_angular_scale_correction", 1.0)

        # Save request.
        self.declare_parameter("measured_angle", 0.0)
        self.declare_parameter("save_calibration", False)

        # Safety checks. Measured/target must be inside this range.
        self.declare_parameter("min_valid_ratio", 0.5)
        self.declare_parameter("max_valid_ratio", 1.5)

        yaml_path = self.get_parameter("calibration_yaml_path").value
        target_node_name = self.get_parameter("target_node_name").value
        legacy_scale = float(read_yaml_value(
            yaml_path,
            target_node_name,
            "odom_angular_scale_correction",
            self.get_parameter("odom_angular_scale_correction").value,
        ))

        yaml_left_scale = float(read_yaml_value(
            yaml_path,
            target_node_name,
            "odom_angular_left_scale_correction",
            legacy_scale,
        ))
        yaml_right_scale = float(read_yaml_value(
            yaml_path,
            target_node_name,
            "odom_angular_right_scale_correction",
            legacy_scale,
        ))

        self.declare_parameter("odom_angular_left_scale_correction", yaml_left_scale)
        self.declare_parameter("odom_angular_right_scale_correction", yaml_right_scale)

        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            qos_profile_sensor_data,
        )

        self.latest_odom = None

        self.state = CalibrationState.IDLE
        self.last_angle = 0.0
        self.turn_angle = 0.0
        self.target_angle_rad = 0.0

        # Frozen values for the current/last finished test.
        self.test_start_angular_scale = 1.0
        self.last_target_angle_deg = 0.0
        self.last_test_valid = False
        self.last_turn_left = True

        # One-shot save guard.
        self.saving_calibration = False

        # Stop command burst after a test finishes.
        self.stop_burst_remaining = 0

        self.timer = self.create_timer(0.02, self.on_timer)
        self.get_logger().info(
            f"calibrate_angular node is ready, using odom_topic={self.odom_topic}, "
            f"left_scale={yaml_left_scale:.10f}, right_scale={yaml_right_scale:.10f}"
        )

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    def read_runtime_params(self):
        self.calibration_yaml_path = str(self.get_parameter("calibration_yaml_path").value)
        self.target_node_name = str(self.get_parameter("target_node_name").value)

        self.test_angle_deg = abs(float(self.get_parameter("test_angle").value))
        self.test_angle_rad = radians(self.test_angle_deg)
        self.speed = abs(float(self.get_parameter("speed").value))
        self.tolerance_rad = radians(abs(float(self.get_parameter("tolerance").value)))
        self.start_test = bool(self.get_parameter("start_test").value)
        self.turn_left = bool(self.get_parameter("turn_left").value)

        self.min_valid_ratio = float(self.get_parameter("min_valid_ratio").value)
        self.max_valid_ratio = float(self.get_parameter("max_valid_ratio").value)

        left_scale = float(self.get_parameter("odom_angular_left_scale_correction").value)
        right_scale = float(self.get_parameter("odom_angular_right_scale_correction").value)

        if left_scale <= 0.0:
            self.get_logger().warning("odom_angular_left_scale_correction must be > 0; using 1.0 temporarily")
            left_scale = 1.0
        if right_scale <= 0.0:
            self.get_logger().warning("odom_angular_right_scale_correction must be > 0; using 1.0 temporarily")
            right_scale = 1.0

        self.odom_angular_left_scale_correction = left_scale
        self.odom_angular_right_scale_correction = right_scale

    def selected_scale(self) -> float:
        return (
            self.odom_angular_left_scale_correction
            if self.turn_left
            else self.odom_angular_right_scale_correction
        )

    def selected_direction_sign(self) -> float:
        return 1.0 if self.turn_left else -1.0

    def selected_direction_name(self) -> str:
        return "left" if self.turn_left else "right"

    def on_timer(self):
        self.read_runtime_params()

        if self.stop_burst_remaining > 0:
            self.publish_stop()
            self.stop_burst_remaining -= 1

        # One-shot save request. Keep it out of parameter callbacks to avoid repeated saves.
        if bool(self.get_parameter("save_calibration").value):
            self.handle_save_request()
            return

        if self.state == CalibrationState.IDLE:
            self.publish_stop()
            if self.start_test:
                self.start_angular_test()
            return

        if self.state == CalibrationState.WAIT_SAVE:
            self.publish_stop()
            if self.start_test:
                self.get_logger().warning(
                    "A previous angular test result is waiting for save. "
                    "Starting a new test will discard the unsaved baseline."
                )
                self.start_angular_test()
            return

        if self.state == CalibrationState.RUNNING:
            if not self.start_test:
                self.finish_test("angular calibration cancelled", wait_for_save=False)
                return
            self.update_angular_test()

    def get_latest_yaw(self):
        if self.latest_odom is None:
            self.get_logger().warning(
                f"No odometry received yet from {self.odom_topic}",
                throttle_duration_sec=1.0,
            )
            return None

        q = self.latest_odom.pose.pose.orientation
        return self.quaternion_to_yaw(q)

    def start_angular_test(self):
        if self.test_angle_deg <= 1e-9:
            self.get_logger().error("test_angle is zero; cannot start angular calibration")
            self.set_start_test(False)
            return

        if self.speed <= 1e-9:
            self.get_logger().error("speed is zero; cannot start angular calibration")
            self.set_start_test(False)
            return

        angle = self.get_latest_yaw()
        if angle is None:
            self.publish_stop()
            return

        self.last_angle = angle
        self.turn_angle = 0.0

        self.target_angle_rad = self.test_angle_rad * self.selected_direction_sign()
        self.test_start_angular_scale = self.selected_scale()
        self.last_target_angle_deg = self.test_angle_deg
        self.last_turn_left = self.turn_left
        self.last_test_valid = False
        self.state = CalibrationState.RUNNING

        self.get_logger().info(
            f"angular {self.selected_direction_name()} test started from {self.odom_topic}: "
            f"target={self.target_angle_rad:.4f} rad ({self.test_angle_deg:.2f} deg), "
            f"speed={self.speed:.4f} rad/s, "
            f"baseline_scale={self.test_start_angular_scale:.10f}"
        )

    def update_angular_test(self):
        current_angle = self.get_latest_yaw()
        if current_angle is None:
            self.publish_stop()
            return

        delta_angle = self.normalize_angle(current_angle - self.last_angle)
        self.turn_angle += delta_angle * self.test_start_angular_scale
        self.last_angle = current_angle

        direction = copysign(1.0, self.target_angle_rad)
        remaining = direction * (self.target_angle_rad - self.turn_angle)

        self.get_logger().info(
            f"angular {self.selected_direction_name()} turn={self.turn_angle:.4f} rad, "
            f"target={self.target_angle_rad:.4f} rad, remaining={remaining:.4f} rad",
            throttle_duration_sec=0.5,
        )

        if remaining <= self.tolerance_rad:
            self.finish_test(
                f"angular {self.selected_direction_name()} motion finished; "
                "measure real angle, then set measured_angle and save_calibration",
                wait_for_save=True,
            )
            return

        move_cmd = Twist()
        move_cmd.angular.z = direction * self.speed
        self.cmd_vel.publish(move_cmd)

    def handle_save_request(self):
        if self.saving_calibration:
            return

        self.saving_calibration = True
        try:
            self.save_angular_calibration()
        finally:
            # Always reset the one-shot flag.
            self.set_parameters([
                Parameter("save_calibration", Parameter.Type.BOOL, False),
            ])
            self.saving_calibration = False

    def save_angular_calibration(self):
        if self.state != CalibrationState.WAIT_SAVE or not self.last_test_valid:
            self.get_logger().error(
                "No completed angular test is waiting to be saved. "
                "Run start_test:=true first, wait for it to finish, then save."
            )
            return

        target_angle_deg = self.last_target_angle_deg
        measured_angle_deg = abs(float(self.get_parameter("measured_angle").value))
        baseline_scale = self.test_start_angular_scale

        if target_angle_deg <= 1e-9:
            self.get_logger().error("last target angle is zero; cannot save")
            return

        if measured_angle_deg <= 1e-9:
            self.get_logger().error("measured_angle must be > 0 before saving")
            return

        ratio = measured_angle_deg / target_angle_deg
        if ratio < self.min_valid_ratio or ratio > self.max_valid_ratio:
            self.get_logger().error(
                f"measured/target ratio={ratio:.4f} is outside valid range "
                f"[{self.min_valid_ratio:.4f}, {self.max_valid_ratio:.4f}]. "
                "Check your measured_angle or adjust min_valid_ratio/max_valid_ratio."
            )
            return

        new_scale = baseline_scale * ratio

        if self.last_turn_left:
            update_odom_calibration_yaml(
                yaml_path=self.calibration_yaml_path,
                target_node_name=self.target_node_name,
                linear_scale=None,
                angular_scale=None,
                angular_left_scale=new_scale,
                angular_right_scale=None,
                enable_odom_calibration=None,
                scale_odom_covariance=None,
            )
            self.set_parameters([
                Parameter("odom_angular_left_scale_correction", Parameter.Type.DOUBLE, float(new_scale)),
                Parameter("measured_angle", Parameter.Type.DOUBLE, 0.0),
            ])
            saved_name = "left"
        else:
            update_odom_calibration_yaml(
                yaml_path=self.calibration_yaml_path,
                target_node_name=self.target_node_name,
                linear_scale=None,
                angular_scale=None,
                angular_left_scale=None,
                angular_right_scale=new_scale,
                enable_odom_calibration=None,
                scale_odom_covariance=None,
            )
            self.set_parameters([
                Parameter("odom_angular_right_scale_correction", Parameter.Type.DOUBLE, float(new_scale)),
                Parameter("measured_angle", Parameter.Type.DOUBLE, 0.0),
            ])
            saved_name = "right"

        self.last_test_valid = False
        self.state = CalibrationState.IDLE

        self.get_logger().info(
            f"[SAVE ONCE] angular {saved_name} calibration saved: "
            f"baseline={baseline_scale:.10f}, "
            f"target={target_angle_deg:.4f} deg, measured={measured_angle_deg:.4f} deg, "
            f"ratio={ratio:.8f}, new={new_scale:.10f}, "
            f"yaml={self.calibration_yaml_path}"
        )

    @staticmethod
    def quaternion_to_yaw(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

    def finish_test(self, message: str, wait_for_save: bool):
        self.publish_stop()
        self.stop_burst_remaining = 10
        self.set_start_test(False)

        if wait_for_save:
            self.last_test_valid = True
            self.state = CalibrationState.WAIT_SAVE
        else:
            self.last_test_valid = False
            self.state = CalibrationState.IDLE

        self.get_logger().info(message)

    def set_start_test(self, value: bool):
        self.set_parameters([
            Parameter("start_test", Parameter.Type.BOOL, value),
        ])

    def publish_stop(self):
        self.cmd_vel.publish(Twist())

    def destroy_node(self):
        self.publish_stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CalibrateAngularSaveYaml()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
