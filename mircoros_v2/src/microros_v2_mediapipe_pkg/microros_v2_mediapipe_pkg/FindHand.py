#!/usr/bin/env python3
# encoding: utf-8

import os
os.environ.setdefault("MEDIAPIPE_DISABLE_TF", "1")
os.environ.setdefault("MEDIAPIPE_DISABLE_GPU", "1")

import cv2 as cv
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image

from .media_library import HandDetector


class FindHand(Node):
    def __init__(self):
        super().__init__("find_hand_node")
        self.declare_parameter("rgb_topic", "/camera/image_raw")
        self.declare_parameter("max_hands", 2)
        self.declare_parameter("detection_con", 0.5)
        self.declare_parameter("tracking_con", 0.5)
        self.declare_parameter("window_name", "find_hand")

        self.rgb_topic = self.get_parameter("rgb_topic").value
        max_hands = int(self.get_parameter("max_hands").value)
        detection_con = float(self.get_parameter("detection_con").value)
        tracking_con = float(self.get_parameter("tracking_con").value)
        self.window_name = str(self.get_parameter("window_name").value)

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.hand_detector = HandDetector(maxHands=max_hands, detectorCon=detection_con, trackCon=tracking_con)
        self.rgb_bridge = CvBridge()
        self.sub_rgb = self.create_subscription(Image, self.rgb_topic, self.get_rgb_image_callback, self.qos_profile)
        self.get_logger().info(f"FindHand node started. Subscribe image topic: {self.rgb_topic}")

    def get_rgb_image_callback(self, msg: Image):
        try:
            rgb_image = self.rgb_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert ROS Image to OpenCV image: {e}")
            return
        frame = self.process(rgb_image)
        cv.imshow(self.window_name, frame)
        key = cv.waitKey(1) & 0xFF
        if key == ord("q"):
            self.get_logger().info("Pressed q, shutting down FindHand node.")
            os._exit(0)  # pylint: disable=protected-access

    def process(self, frame):
        frame, lmList, bbox = self.hand_detector.findHands(frame)
        self.hand_detector.draw = True
        if len(lmList) != 0:
            _ = self.hand_detector.fingersUp(lmList)
            indexX = (bbox[0] + bbox[2]) / 2
            indexY = (bbox[1] + bbox[3]) / 2
            self.get_logger().info("indexX: %.1f, indexY: %.1f" % (indexX, indexY))
        return frame

    def destroy_node(self):
        try:
            self.hand_detector.close()
        except Exception:
            pass
        try:
            cv.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FindHand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, shutting down.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
