#!/usr/bin/env python3
# encoding: utf-8

import os

# Must be set before importing mediapipe.
# TF means TensorFlow here, not ROS tf/tf2.
os.environ.setdefault("MEDIAPIPE_DISABLE_TF", "1")
os.environ.setdefault("MEDIAPIPE_DISABLE_GPU", "1")

import cv2 as cv
import mediapipe as mp
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image


class Objectron(Node):
    """MediaPipe Objectron ROS 2 node.

    Notes:
    - Objectron is not a general object detector.
    - Supported model names are only: Shoe, Chair, Cup, Camera.
    - Press f/F in the OpenCV window to switch model.
    - Press q to quit.
    """

    SUPPORTED_MODELS = ["Shoe", "Chair", "Cup", "Camera"]

    def __init__(self):
        super().__init__("objectron_node")

        # =========================
        # Parameters
        # =========================
        self.declare_parameter("rgb_topic", "/camera/image_raw")
        self.declare_parameter("static_image_mode", False)
        self.declare_parameter("max_objects", 1)
        self.declare_parameter("detection_con", 0.30)
        self.declare_parameter("tracking_con", 0.50)
        self.declare_parameter("model_name", "Cup")
        self.declare_parameter("window_name", "objectron")
        self.declare_parameter("flip_image", False)
        self.declare_parameter("show_help", True)

        self.rgb_topic = str(self.get_parameter("rgb_topic").value)
        self.static_image_mode = self._as_bool(self.get_parameter("static_image_mode").value)
        self.max_objects = int(self.get_parameter("max_objects").value)
        self.detection_con = float(self.get_parameter("detection_con").value)
        self.tracking_con = float(self.get_parameter("tracking_con").value)
        self.window_name = str(self.get_parameter("window_name").value)
        self.flip_image = self._as_bool(self.get_parameter("flip_image").value)
        self.show_help = self._as_bool(self.get_parameter("show_help").value)

        model_name = self._normalize_model_name(str(self.get_parameter("model_name").value))
        self.model_index = self.SUPPORTED_MODELS.index(model_name)

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.mp_objectron = mp.solutions.objectron
        self.mp_draw = mp.solutions.drawing_utils
        self.objectron = None
        self.bridge = CvBridge()
        self.frame_count = 0
        self.detect_count = 0

        self.recreate_detector()

        self.sub_rgb = self.create_subscription(
            Image,
            self.rgb_topic,
            self.get_rgb_image_callback,
            self.qos_profile,
        )

        self.get_logger().info("Objectron node started.")
        self.get_logger().info(f"Subscribe image topic: {self.rgb_topic}")
        self.get_logger().info(
            f"model_name={self.current_model}, static_image_mode={self.static_image_mode}, "
            f"max_objects={self.max_objects}, detection_con={self.detection_con}, "
            f"tracking_con={self.tracking_con}"
        )
        self.get_logger().info("Objectron only supports: Shoe, Chair, Cup, Camera. Press f to switch model.")

    @property
    def current_model(self):
        return self.SUPPORTED_MODELS[self.model_index]

    def _as_bool(self, value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _normalize_model_name(self, model_name: str) -> str:
        for name in self.SUPPORTED_MODELS:
            if model_name.lower() == name.lower():
                return name
        self.get_logger().warn(
            f"Unsupported model_name '{model_name}', fallback to Cup. "
            f"Supported: {self.SUPPORTED_MODELS}"
        )
        return "Cup"

    def recreate_detector(self):
        try:
            if self.objectron is not None:
                self.objectron.close()
        except Exception:
            pass

        self.objectron = self.mp_objectron.Objectron(
            static_image_mode=self.static_image_mode,
            max_num_objects=self.max_objects,
            min_detection_confidence=self.detection_con,
            min_tracking_confidence=self.tracking_con,
            model_name=self.current_model,
        )
        self.get_logger().info(f"Objectron detector created: model={self.current_model}")

    def get_rgb_image_callback(self, msg: Image):
        if msg.data is None or len(msg.data) == 0:
            self.get_logger().warn("Received empty image message.")
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert ROS Image to OpenCV image: {e}")
            return

        if self.flip_image:
            frame = cv.flip(frame, 1)

        frame = self.find_objectron(frame)
        cv.imshow(self.window_name, frame)

        key = cv.waitKey(1) & 0xFF
        if key in (ord("f"), ord("F")):
            self.switch_model()
        elif key == ord("q"):
            self.get_logger().info("Pressed q, shutting down Objectron node.")
            os._exit(0)  # pylint: disable=protected-access

    def find_objectron(self, frame):
        self.frame_count += 1

        # MediaPipe expects RGB input.
        rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        results = self.objectron.process(rgb)
        rgb.flags.writeable = True

        detect_count = 0
        if results.detected_objects:
            detect_count = len(results.detected_objects)
            for detected_object in results.detected_objects:
                self.mp_draw.draw_landmarks(
                    frame,
                    detected_object.landmarks_2d,
                    self.mp_objectron.BOX_CONNECTIONS,
                )
                self.mp_draw.draw_axis(
                    frame,
                    detected_object.rotation,
                    detected_object.translation,
                )

        self.detect_count = detect_count
        self.draw_status(frame, detect_count)
        return frame

    def draw_status(self, frame, detect_count: int):
        cv.putText(
            frame,
            f"Objectron: {self.current_model}",
            (20, 30),
            cv.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2,
        )
        cv.putText(
            frame,
            f"Detected: {detect_count}",
            (20, 62),
            cv.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0) if detect_count > 0 else (0, 0, 255),
            2,
        )
        if self.show_help:
            cv.putText(
                frame,
                "f: switch model | q: quit | supported: Shoe/Chair/Cup/Camera",
                (20, frame.shape[0] - 20),
                cv.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1,
            )

    def switch_model(self):
        self.model_index += 1
        if self.model_index >= len(self.SUPPORTED_MODELS):
            self.model_index = 0
        self.recreate_detector()
        self.get_logger().info(f"Switched Objectron model to {self.current_model}")

    def destroy_node(self):
        try:
            if self.objectron is not None:
                self.objectron.close()
        except Exception:
            pass
        try:
            cv.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Objectron()
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
