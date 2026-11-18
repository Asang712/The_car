#!/usr/bin/env python3
# encoding: utf-8

import os
os.environ.setdefault("MEDIAPIPE_DISABLE_TF", "1")
os.environ.setdefault("MEDIAPIPE_DISABLE_GPU", "1")

import cv2 as cv
import mediapipe as mp
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image


class FaceDetector(Node):
    def __init__(self):
        super().__init__("face_detection_node")
        self.declare_parameter("rgb_topic", "/camera/image_raw")
        self.declare_parameter("detection_con", 0.5)
        self.declare_parameter("window_name", "face_detection")

        self.rgb_topic = self.get_parameter("rgb_topic").value
        detection_con = float(self.get_parameter("detection_con").value)
        self.window_name = str(self.get_parameter("window_name").value)

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.mpFaceDetection = mp.solutions.face_detection
        self.facedetection = self.mpFaceDetection.FaceDetection(min_detection_confidence=detection_con)
        self.rgb_bridge = CvBridge()
        self.sub_rgb = self.create_subscription(Image, self.rgb_topic, self.get_rgb_image_callback, self.qos_profile)
        self.get_logger().info(f"FaceDetection node started. Subscribe image topic: {self.rgb_topic}")

    def get_rgb_image_callback(self, msg: Image):
        try:
            rgb_image = self.rgb_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert ROS Image to OpenCV image: {e}")
            return
        frame, _ = self.find_faces(rgb_image)
        cv.imshow(self.window_name, frame)
        key = cv.waitKey(1) & 0xFF
        if key == ord("q"):
            self.get_logger().info("Pressed q, shutting down FaceDetection node.")
            os._exit(0)  # pylint: disable=protected-access

    def find_faces(self, frame):
        img_rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        results = self.facedetection.process(img_rgb)
        bboxs = []
        if results.detections:
            for idx, detection in enumerate(results.detections):
                bboxC = detection.location_data.relative_bounding_box
                ih, iw, _ = frame.shape
                bbox = (
                    int(bboxC.xmin * iw),
                    int(bboxC.ymin * ih),
                    int(bboxC.width * iw),
                    int(bboxC.height * ih),
                )
                bboxs.append([idx, bbox, detection.score])
                frame = self.fancy_draw(frame, bbox)
                score = int(detection.score[0] * 100) if detection.score else 0
                cv.putText(frame, f"{score}%", (bbox[0], bbox[1] - 20), cv.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 2)
        return frame, bboxs

    def fancy_draw(self, frame, bbox, l=30, t=10):
        x, y, w, h = bbox
        x1, y1 = x + w, y + h
        cv.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 255), 2)
        cv.line(frame, (x, y), (x + l, y), (255, 0, 255), t)
        cv.line(frame, (x, y), (x, y + l), (255, 0, 255), t)
        cv.line(frame, (x1, y), (x1 - l, y), (255, 0, 255), t)
        cv.line(frame, (x1, y), (x1, y + l), (255, 0, 255), t)
        cv.line(frame, (x, y1), (x + l, y1), (255, 0, 255), t)
        cv.line(frame, (x, y1), (x, y1 - l), (255, 0, 255), t)
        cv.line(frame, (x1, y1), (x1 - l, y1), (255, 0, 255), t)
        cv.line(frame, (x1, y1), (x1, y1 - l), (255, 0, 255), t)
        return frame

    def destroy_node(self):
        try:
            self.facedetection.close()
        except Exception:
            pass
        try:
            cv.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FaceDetector()
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
