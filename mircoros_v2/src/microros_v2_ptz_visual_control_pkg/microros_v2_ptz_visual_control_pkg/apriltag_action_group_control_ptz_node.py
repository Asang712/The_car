#!/usr/bin/env python3
"""AprilTag machine-code chassis control node (PTZ package)."""

from __future__ import annotations

import threading
from typing import Optional

import cv2 as cv
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image

from .image_conversion import image_to_bgr


MACHINE_CODE_FORWARD = 1
MACHINE_CODE_BACKWARD = 2
MACHINE_CODE_FORWARD_RIGHT = 3
MACHINE_CODE_FORWARD_LEFT = 4


class MachineCodeActionGroupControlPtzNode(Node):
    def __init__(self) -> None:
        super().__init__("machine_code_action_group_control_ptz_node")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("tag_family", "tag36h11")
        self.declare_parameter("quad_decimate", 2.0)
        self.declare_parameter("processing_width", 320)
        self.declare_parameter("processing_height", 240)
        self.declare_parameter("image_processing_rate_hz", 15.0)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("linear_speed", 0.2)
        self.declare_parameter("angular_speed", 0.8)
        self.declare_parameter("show_debug_window", True)
        self.declare_parameter("window_name", "machine_code_action_group")

        self._image_topic = str(self.get_parameter("image_topic").value)
        self._cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._tag_family = str(self.get_parameter("tag_family").value)
        self._quad_decimate = float(self.get_parameter("quad_decimate").value)
        self._width = int(self.get_parameter("processing_width").value)
        self._height = int(self.get_parameter("processing_height").value)
        self._linear_speed = abs(float(self.get_parameter("linear_speed").value))
        self._angular_speed = abs(float(self.get_parameter("angular_speed").value))
        self._show_window = bool(self.get_parameter("show_debug_window").value)
        self._window_name = str(self.get_parameter("window_name").value)
        self._lock = threading.Lock()
        self._latest_frame: Optional[np.ndarray] = None
        self._machine_code: Optional[int] = None
        self._shutdown = False

        try:
            import dt_apriltags
            self._detector = dt_apriltags.Detector(
                families=self._tag_family, nthreads=1, quad_decimate=self._quad_decimate)
        except ImportError:
            self._detector = None
            self.get_logger().error("dt_apriltags is not installed")

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)
        self._image_sub = self.create_subscription(Image, self._image_topic, self._on_image, qos)
        self._cmd_pub = self.create_publisher(Twist, self._cmd_topic, qos)
        self._image_timer = self.create_timer(
            1.0 / max(float(self.get_parameter("image_processing_rate_hz").value), 1.0),
            self._process_image)
        self._control_timer = self.create_timer(
            1.0 / max(float(self.get_parameter("control_rate_hz").value), 1.0),
            self._publish_control)
        if self._show_window:
            cv.namedWindow(self._window_name, cv.WINDOW_NORMAL)
        self.get_logger().info("Machine-code AprilTag control started")

    def _on_image(self, msg: Image) -> None:
        try:
            frame = image_to_bgr(msg)
        except Exception as exc:
            self.get_logger().warning(f"Image conversion failed: {exc}")
            return
        with self._lock:
            self._latest_frame = frame

    def _process_image(self) -> None:
        with self._lock:
            if self._latest_frame is None:
                return
            frame = cv.resize(self._latest_frame, (self._width, self._height))
        code = None
        if self._detector is not None:
            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            tags = self._detector.detect(gray, estimate_tag_pose=False)
            valid_ids = [int(tag.tag_id) for tag in tags if int(tag.tag_id) in (1, 2, 3, 4)]
            if valid_ids:
                code = min(valid_ids)
                for tag in tags:
                    if int(tag.tag_id) == code:
                        points = np.asarray(tag.corners, dtype=np.int32)
                        cv.polylines(frame, [points], True, (0, 255, 0), 2)
                        break
        with self._lock:
            self._machine_code = code
        if self._show_window:
            cv.putText(frame, f"Machine code: {code if code is not None else 'None'}",
                       (6, 20), cv.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv.LINE_AA)
            cv.imshow(self._window_name, frame)
            if cv.waitKey(1) & 0xFF in (ord("q"), 27):
                self._shutdown = True
                rclpy.shutdown()

    def _publish_control(self) -> None:
        with self._lock:
            code = self._machine_code
        cmd = Twist()
        if code == MACHINE_CODE_FORWARD:
            cmd.linear.x = self._linear_speed
        elif code == MACHINE_CODE_BACKWARD:
            cmd.linear.x = -self._linear_speed
        elif code == MACHINE_CODE_FORWARD_RIGHT:
            cmd.linear.x = self._linear_speed
            cmd.angular.z = -self._angular_speed
        elif code == MACHINE_CODE_FORWARD_LEFT:
            cmd.linear.x = self._linear_speed
            cmd.angular.z = self._angular_speed
        self._cmd_pub.publish(cmd)

    def destroy_node(self) -> None:
        self._machine_code = None
        self._cmd_pub.publish(Twist())
        cv.destroyAllWindows()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MachineCodeActionGroupControlPtzNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

