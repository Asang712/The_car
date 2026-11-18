#!/usr/bin/env python3
# encoding: utf-8

import os
os.environ.setdefault("MEDIAPIPE_DISABLE_TF", "1")
os.environ.setdefault("MEDIAPIPE_DISABLE_GPU", "1")

import cv2 as cv
import dlib
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image


class FaceLandmarks(Node):
    def __init__(self):
        super().__init__("face_landmarks_node")
        self.declare_parameter("rgb_topic", "/camera/image_raw")
        self.declare_parameter("landmark_model_path", os.path.join(os.path.expanduser("~"), "MODELS", "shape_predictor_68_face_landmarks.dat"))
        self.declare_parameter("draw_indexes", False)
        self.declare_parameter("eye", True)
        self.declare_parameter("lips", True)
        self.declare_parameter("eyebrow", True)
        self.declare_parameter("window_name", "face_landmarks")

        self.rgb_topic = self.get_parameter("rgb_topic").value
        self.dat_file = os.path.expanduser(str(self.get_parameter("landmark_model_path").value))
        self.draw_indexes = bool(self.get_parameter("draw_indexes").value)
        self.eye = bool(self.get_parameter("eye").value)
        self.lips = bool(self.get_parameter("lips").value)
        self.eyebrow = bool(self.get_parameter("eyebrow").value)
        self.window_name = str(self.get_parameter("window_name").value)

        if not os.path.exists(self.dat_file):
            raise FileNotFoundError(f"dlib face landmark model not found: {self.dat_file}")

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.hog_face_detector = dlib.get_frontal_face_detector()
        self.dlib_facelandmark = dlib.shape_predictor(self.dat_file)
        self.faces = []
        self.face_landmarks = None
        self.rgb_bridge = CvBridge()
        self.sub_rgb = self.create_subscription(Image, self.rgb_topic, self.get_rgb_image_callback, self.qos_profile)
        self.get_logger().info(f"FaceLandmarks node started. Subscribe image topic: {self.rgb_topic}")

    def get_rgb_image_callback(self, msg: Image):
        try:
            rgb_image = self.rgb_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert ROS Image to OpenCV image: {e}")
            return
        frame = self.get_face(rgb_image, draw=self.draw_indexes)
        frame = self.prettify_face(frame, eye=self.eye, lips=self.lips, eyebrow=self.eyebrow, draw=True)
        cv.imshow(self.window_name, frame)
        key = cv.waitKey(1) & 0xFF
        if key == ord("q"):
            self.get_logger().info("Pressed q, shutting down FaceLandmarks node.")
            os._exit(0)  # pylint: disable=protected-access

    def get_face(self, frame, draw=True):
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        self.faces = self.hog_face_detector(gray)
        self.face_landmarks = None
        for face in self.faces:
            self.face_landmarks = self.dlib_facelandmark(gray, face)
            if draw:
                for n in range(68):
                    x = self.face_landmarks.part(n).x
                    y = self.face_landmarks.part(n).y
                    cv.circle(frame, (x, y), 2, (0, 255, 255), 2)
                    cv.putText(frame, str(n), (x, y), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        return frame

    def get_lm_list(self, frame, p1, p2, draw=True):
        lmList = []
        if len(self.faces) != 0 and self.face_landmarks is not None:
            for n in range(p1, p2):
                x = self.face_landmarks.part(n).x
                y = self.face_landmarks.part(n).y
                lmList.append([x, y])
                if draw:
                    next_point = p1 if n == p2 - 1 else n + 1
                    x2 = self.face_landmarks.part(next_point).x
                    y2 = self.face_landmarks.part(next_point).y
                    cv.line(frame, (x, y), (x2, y2), (0, 255, 0), 1)
        return lmList

    def get_lip_list(self, frame, lipIndexlist, draw=True):
        lmList = []
        if len(self.faces) != 0 and self.face_landmarks is not None:
            for n in range(len(lipIndexlist)):
                x = self.face_landmarks.part(lipIndexlist[n]).x
                y = self.face_landmarks.part(lipIndexlist[n]).y
                lmList.append([x, y])
                if draw:
                    next_point = 0 if n == len(lipIndexlist) - 1 else n + 1
                    x2 = self.face_landmarks.part(lipIndexlist[next_point]).x
                    y2 = self.face_landmarks.part(lipIndexlist[next_point]).y
                    cv.line(frame, (x, y), (x2, y2), (0, 255, 0), 1)
        return lmList

    def prettify_face(self, frame, eye=True, lips=True, eyebrow=True, draw=True):
        if eye:
            leftEye = self.get_lm_list(frame, 36, 42)
            rightEye = self.get_lm_list(frame, 42, 48)
            if draw:
                if leftEye:
                    frame = cv.fillConvexPoly(frame, np.array(leftEye, dtype=np.int32), (0, 0, 0))
                if rightEye:
                    frame = cv.fillConvexPoly(frame, np.array(rightEye, dtype=np.int32), (0, 0, 0))
        if lips:
            for lipIndexlist in ([51, 52, 53, 54, 64, 63, 62], [48, 49, 50, 51, 62, 61, 60], [57, 58, 59, 48, 67, 66], [54, 55, 56, 57, 66, 65, 64]):
                lip = self.get_lip_list(frame, lipIndexlist, draw=True)
                if draw and lip:
                    frame = cv.fillConvexPoly(frame, np.array(lip, dtype=np.int32), (249, 0, 226))
        if eyebrow:
            lefteyebrow = self.get_lm_list(frame, 17, 22)
            righteyebrow = self.get_lm_list(frame, 22, 27)
            if draw:
                if lefteyebrow:
                    frame = cv.fillConvexPoly(frame, np.array(lefteyebrow, dtype=np.int32), (255, 255, 255))
                if righteyebrow:
                    frame = cv.fillConvexPoly(frame, np.array(righteyebrow, dtype=np.int32), (255, 255, 255))
        return frame

    def destroy_node(self):
        try:
            cv.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FaceLandmarks()
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
