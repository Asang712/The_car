#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Linear odometry calibration node based on /odom_raw.

This version does NOT use TF. It subscribes to nav_msgs/Odometry directly:
    default odom_topic = /odom_raw

Workflow:
    ros2 launch microros_v2_bringup calibrate_linear.launch.py

    ros2 param set /calibrate_linear test_distance 1.0
    ros2 param set /calibrate_linear speed 0.2
    ros2 param set /calibrate_linear start_test true

    # After the robot stops, measure the real distance, for example 1.08 m:
    ros2 param set /calibrate_linear measured_distance 1.08
    ros2 param set /calibrate_linear save_calibration true

Formula:
    new_linear_scale = test_start_linear_scale * measured_distance / abs(test_distance)
"""

from enum import Enum
from math import copysign, sqrt

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


class CalibrateLinearSaveYaml(Node):
    def __init__(self):
        super().__init__("calibrate_linear_node")

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
        self.declare_parameter("calibration_yaml_path", "odom_linear_calibration.yaml")
        self.declare_parameter("target_node_name", "microros_v2_patrol_node")

        # Motion test.
        self.declare_parameter("test_distance", 1.0)      # meters; negative means reverse
        self.declare_parameter("speed", 0.2)              # m/s
        self.declare_parameter("tolerance", 0.03)         # meters
        self.declare_parameter("direction", True)         # True: linear.x, False: linear.y
        self.declare_parameter("start_test", False)

        # Keep these parameters for compatibility with older launch files; this node does not use TF.
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("odom_frame", "odom")

        # Save request.
        self.declare_parameter("measured_distance", 0.0)
        self.declare_parameter("save_calibration", False)

        # Safety checks. Measured/target must be inside this range.
        self.declare_parameter("min_valid_ratio", 0.5)
        self.declare_parameter("max_valid_ratio", 1.5)

        # Load initial linear scale from YAML if available.
        yaml_path = self.get_parameter("calibration_yaml_path").value
        target_node_name = self.get_parameter("target_node_name").value
        yaml_linear_scale = float(read_yaml_value(
            yaml_path,
            target_node_name,
            "odom_linear_scale_correction",
            1.0,
        ))
        self.declare_parameter("odom_linear_scale_correction", yaml_linear_scale)

        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            qos_profile_sensor_data,
        )

        self.latest_odom = None

        self.state = CalibrationState.IDLE
        self.x_start = 0.0
        self.y_start = 0.0

        # Frozen values for the current/last finished test.
        self.test_start_linear_scale = yaml_linear_scale
        self.last_target_distance = 0.0
        self.last_test_valid = False

        # One-shot save guard.
        self.saving_calibration = False

        # Stop command burst after a test finishes.
        self.stop_burst_remaining = 0

        self.timer = self.create_timer(0.02, self.on_timer)
        self.get_logger().info(f"calibrate_linear node is ready, using odom_topic={self.odom_topic}")

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    def read_runtime_params(self):
        self.calibration_yaml_path = str(self.get_parameter("calibration_yaml_path").value)
        self.target_node_name = str(self.get_parameter("target_node_name").value)

        self.test_distance = float(self.get_parameter("test_distance").value)
        self.speed = abs(float(self.get_parameter("speed").value))
        self.tolerance = abs(float(self.get_parameter("tolerance").value))
        self.direction = bool(self.get_parameter("direction").value)
        self.start_test = bool(self.get_parameter("start_test").value)

        self.min_valid_ratio = float(self.get_parameter("min_valid_ratio").value)
        self.max_valid_ratio = float(self.get_parameter("max_valid_ratio").value)

        scale = float(self.get_parameter("odom_linear_scale_correction").value)
        if scale <= 0.0:
            self.get_logger().warning("odom_linear_scale_correction must be > 0; using 1.0 temporarily")
            scale = 1.0
        self.odom_linear_scale_correction = scale

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
                self.start_linear_test()
            return

        if self.state == CalibrationState.WAIT_SAVE:
            self.publish_stop()
            if self.start_test:
                self.get_logger().warning(
                    "A previous linear test result is waiting for save. "
                    "Starting a new test will discard the unsaved baseline."
                )
                self.start_linear_test()
            return

        if self.state == CalibrationState.RUNNING:
            if not self.start_test:
                self.finish_test("linear calibration cancelled", wait_for_save=False)
                return
            self.update_linear_test()

    def get_latest_pose_xy(self):
        if self.latest_odom is None:
            self.get_logger().warning(
                f"No odometry received yet from {self.odom_topic}",
                throttle_duration_sec=1.0,
            )
            return None

        pose = self.latest_odom.pose.pose
        return float(pose.position.x), float(pose.position.y)

    def start_linear_test(self):
        if abs(self.test_distance) <= 1e-9:
            self.get_logger().error("test_distance is zero; cannot start linear calibration")
            self.set_start_test(False)
            return

        if self.speed <= 1e-9:
            self.get_logger().error("speed is zero; cannot start linear calibration")
            self.set_start_test(False)
            return

        xy = self.get_latest_pose_xy()
        if xy is None:
            self.publish_stop()
            return

        self.x_start, self.y_start = xy

        self.test_start_linear_scale = self.odom_linear_scale_correction
        self.last_target_distance = abs(self.test_distance)
        self.last_test_valid = False
        self.state = CalibrationState.RUNNING

        self.get_logger().info(
            f"linear test started from {self.odom_topic}: "
            f"target={self.test_distance:.4f} m, "
            f"speed={self.speed:.4f} m/s, "
            f"axis={'x' if self.direction else 'y'}, "
            f"baseline_scale={self.test_start_linear_scale:.10f}"
        )

    def update_linear_test(self):
        xy = self.get_latest_pose_xy()
        if xy is None:
            self.publish_stop()
            return

        x_now, y_now = xy

        raw_distance = sqrt((x_now - self.x_start) ** 2 + (y_now - self.y_start) ** 2)
        corrected_distance = raw_distance * self.test_start_linear_scale
        remaining = self.last_target_distance - corrected_distance

        self.get_logger().info(
            f"linear distance={corrected_distance:.4f} m, "
            f"target={self.last_target_distance:.4f} m, remaining={remaining:.4f} m",
            throttle_duration_sec=0.5,
        )

        if remaining <= self.tolerance:
            self.finish_test(
                "linear motion finished; measure real distance, then set measured_distance and save_calibration",
                wait_for_save=True,
            )
            return

        move_cmd = Twist()
        move_sign = copysign(1.0, self.test_distance)
        if self.direction:
            move_cmd.linear.x = move_sign * self.speed
            move_cmd.angular.z = 0.0
        else:
            move_cmd.linear.y = move_sign * self.speed
            move_cmd.angular.z = 0.0

        self.cmd_vel.publish(move_cmd)

    def handle_save_request(self):
        if self.saving_calibration:
            return

        self.saving_calibration = True
        try:
            self.save_linear_calibration()
        finally:
            # Always reset the one-shot flag.
            self.set_parameters([
                Parameter("save_calibration", Parameter.Type.BOOL, False),
            ])
            self.saving_calibration = False

    def save_linear_calibration(self):
        if self.state != CalibrationState.WAIT_SAVE or not self.last_test_valid:
            self.get_logger().error(
                "No completed linear test is waiting to be saved. "
                "Run start_test:=true first, wait for it to finish, then save."
            )
            return

        target_distance = self.last_target_distance
        measured_distance = abs(float(self.get_parameter("measured_distance").value))
        baseline_scale = self.test_start_linear_scale

        if target_distance <= 1e-9:
            self.get_logger().error("last target distance is zero; cannot save")
            return

        if measured_distance <= 1e-9:
            self.get_logger().error("measured_distance must be > 0 before saving")
            return

        ratio = measured_distance / target_distance
        if ratio < self.min_valid_ratio or ratio > self.max_valid_ratio:
            self.get_logger().error(
                f"measured/target ratio={ratio:.4f} is outside valid range "
                f"[{self.min_valid_ratio:.4f}, {self.max_valid_ratio:.4f}]. "
                "Check your measured_distance or adjust min_valid_ratio/max_valid_ratio."
            )
            return

        new_scale = baseline_scale * ratio

        update_odom_calibration_yaml(
            yaml_path=self.calibration_yaml_path,
            target_node_name=self.target_node_name,
            linear_scale=new_scale,
            angular_scale=None,
            enable_odom_calibration=None,
            scale_odom_covariance=None,
        )

        self.set_parameters([
            Parameter("odom_linear_scale_correction", Parameter.Type.DOUBLE, float(new_scale)),
            Parameter("measured_distance", Parameter.Type.DOUBLE, 0.0),
        ])

        self.last_test_valid = False
        self.state = CalibrationState.IDLE

        self.get_logger().info(
            f"[SAVE ONCE] linear calibration saved: "
            f"baseline={baseline_scale:.10f}, "
            f"target={target_distance:.4f} m, measured={measured_distance:.4f} m, "
            f"ratio={ratio:.8f}, new={new_scale:.10f}, "
            f"yaml={self.calibration_yaml_path}"
        )

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
    node = CalibrateLinearSaveYaml()
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
