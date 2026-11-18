#!/usr/bin/env python3
# encoding: utf-8

import os
os.environ.setdefault("MEDIAPIPE_DISABLE_TF", "1")
os.environ.setdefault("MEDIAPIPE_DISABLE_GPU", "1")

import math
import cv2 as cv
import mediapipe as mp
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image


class VirtualPaint(Node):
    def __init__(self):
        super().__init__("virtual_paint_node")
        self.declare_parameter("rgb_topic", "/camera/image_raw")
        self.declare_parameter("max_hands", 2)
        self.declare_parameter("detection_con", 0.85)
        self.declare_parameter("tracking_con", 0.5)
        self.declare_parameter("brush_thickness", 5)
        self.declare_parameter("eraser_thickness", 100)
        self.declare_parameter("top_height", 50)
        self.declare_parameter("window_name", "virtual_paint")

        self.rgb_topic = self.get_parameter("rgb_topic").value
        max_hands = int(self.get_parameter("max_hands").value)
        detection_con = float(self.get_parameter("detection_con").value)
        tracking_con = float(self.get_parameter("tracking_con").value)
        self.brushThickness = int(self.get_parameter("brush_thickness").value)
        self.eraserThickness = int(self.get_parameter("eraser_thickness").value)
        self.top_height = int(self.get_parameter("top_height").value)
        self.window_name = str(self.get_parameter("window_name").value)

        self.xp = self.yp = self.boxx = 0
        self.tipIds = [4, 8, 12, 16, 20]
        self.imgCanvas = None
        self.Color = "Red"
        self.ColorList = {
            "Red": (0, 0, 255),
            "Green": (0, 255, 0),
            "Blue": (255, 0, 0),
            "Yellow": (0, 255, 255),
            "Black": (0, 0, 0),
        }

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.mpHand = mp.solutions.hands
        self.mpDraw = mp.solutions.drawing_utils
        self.hands = self.mpHand.Hands(
            static_image_mode=False,
            max_num_hands=max_hands,
            min_detection_confidence=detection_con,
            min_tracking_confidence=tracking_con,
        )
        self.lmDrawSpec = self.mpDraw.DrawingSpec(color=(0, 0, 255), thickness=-1, circle_radius=15)
        self.drawSpec = self.mpDraw.DrawingSpec(color=(0, 255, 0), thickness=10, circle_radius=10)
        self.rgb_bridge = CvBridge()
        self.sub_rgb = self.create_subscription(Image, self.rgb_topic, self.get_rgb_image_callback, self.qos_profile)
        self.get_logger().info(f"VirtualPaint node started. Subscribe image topic: {self.rgb_topic}")

    def get_rgb_image_callback(self, msg: Image):
        try:
            rgb_image = self.rgb_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert ROS Image to OpenCV image: {e}")
            return
        h, w, _ = rgb_image.shape
        if self.imgCanvas is None or self.imgCanvas.shape[:2] != (h, w):
            self.imgCanvas = np.zeros((h, w, 3), np.uint8)
        frame, lmList = self.find_hands(rgb_image, draw=False)
        key = cv.waitKey(1) & 0xFF
        if key == ord("q"):
            self.get_logger().info("Pressed q, shutting down VirtualPaint node.")
            os._exit(0)  # pylint: disable=protected-access
            return
        if key == ord("c"):
            self.imgCanvas[:] = 0
            self.get_logger().info("Canvas cleared.")

        if len(lmList) >= 21:
            x1, y1 = lmList[8][1:]
            x2, y2 = lmList[12][1:]
            fingers = self.fingers_up()
            if fingers[1] and fingers[2]:
                if y1 < self.top_height:
                    if 0 < x1 < int(w / 5) - 1:
                        self.boxx = 0
                        self.Color = "Red"
                    elif int(w / 5) < x1 < int(w * 2 / 5) - 1:
                        self.boxx = int(w / 5)
                        self.Color = "Green"
                    elif int(w * 2 / 5) < x1 < int(w * 3 / 5) - 1:
                        self.boxx = int(w * 2 / 5)
                        self.Color = "Blue"
                    elif int(w * 3 / 5) < x1 < int(w * 4 / 5) - 1:
                        self.boxx = int(w * 3 / 5)
                        self.Color = "Yellow"
                    elif int(w * 4 / 5) < x1 < w - 1:
                        self.boxx = int(w * 4 / 5)
                        self.Color = "Black"
                cv.rectangle(frame, (x1, y1 - 25), (x2, y2 + 25), self.ColorList[self.Color], cv.FILLED)
                self.draw_palette(frame, w)
            if fingers[1] and not fingers[2] and math.hypot(x2 - x1, y2 - y1) > 50:
                if self.xp == self.yp == 0:
                    self.xp, self.yp = x1, y1
                thickness = self.eraserThickness if self.Color == "Black" else self.brushThickness
                cv.line(frame, (self.xp, self.yp), (x1, y1), self.ColorList[self.Color], thickness)
                cv.line(self.imgCanvas, (self.xp, self.yp), (x1, y1), self.ColorList[self.Color], thickness)
                cv.circle(frame, (x1, y1), 15, self.ColorList[self.Color], cv.FILLED)
                self.xp, self.yp = x1, y1
            else:
                self.xp = self.yp = 0
        imgGray = cv.cvtColor(self.imgCanvas, cv.COLOR_BGR2GRAY)
        _, imgInv = cv.threshold(imgGray, 50, 255, cv.THRESH_BINARY_INV)
        imgInv = cv.cvtColor(imgInv, cv.COLOR_GRAY2BGR)
        frame = cv.bitwise_and(frame, imgInv)
        frame = cv.bitwise_or(frame, self.imgCanvas)
        cv.rectangle(frame, (20, h - 100), (50, h - 70), self.ColorList[self.Color], cv.FILLED)
        cv.putText(frame, self.Color, (70, h - 75), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        cv.imshow(self.window_name, frame)

    def draw_palette(self, frame, w):
        cv.rectangle(frame, (self.boxx, 0), (self.boxx + int(w / 5), self.top_height), self.ColorList[self.Color], cv.FILLED)
        cv.rectangle(frame, (0, 0), (int(w / 5) - 1, self.top_height), self.ColorList["Red"], 3)
        cv.rectangle(frame, (int(w / 5) + 2, 0), (int(w * 2 / 5) - 1, self.top_height), self.ColorList["Green"], 3)
        cv.rectangle(frame, (int(w * 2 / 5) + 2, 0), (int(w * 3 / 5) - 1, self.top_height), self.ColorList["Blue"], 3)
        cv.rectangle(frame, (int(w * 3 / 5) + 2, 0), (int(w * 4 / 5) - 1, self.top_height), self.ColorList["Yellow"], 3)
        cv.rectangle(frame, (int(w * 4 / 5) + 2, 0), (w - 1, self.top_height), self.ColorList["Black"], 3)

    def find_hands(self, frame, draw=True):
        self.lmList = []
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.hands.process(img_RGB)
        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(frame, handLms, self.mpHand.HAND_CONNECTIONS, self.lmDrawSpec, self.drawSpec)
                else:
                    self.mpDraw.draw_landmarks(frame, handLms, self.mpHand.HAND_CONNECTIONS)
            for idx, lm in enumerate(self.results.multi_hand_landmarks[0].landmark):
                h, w, _ = frame.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                self.lmList.append([idx, cx, cy])
        return frame, self.lmList

    def fingers_up(self):
        fingers = []
        if len(self.lmList) < 21:
            return [0, 0, 0, 0, 0]
        if (self.calc_angle(self.tipIds[0], self.tipIds[0] - 1, self.tipIds[0] - 2) > 150.0) and \
           (self.calc_angle(self.tipIds[0] - 1, self.tipIds[0] - 2, self.tipIds[0] - 3) > 150.0):
            fingers.append(1)
        else:
            fingers.append(0)
        for idx in range(1, 5):
            if self.lmList[self.tipIds[idx]][2] < self.lmList[self.tipIds[idx] - 2][2]:
                fingers.append(1)
            else:
                fingers.append(0)
        return fingers

    def get_dist(self, point1, point2):
        x1, y1 = point1
        x2, y2 = point2
        return abs(math.sqrt(math.pow(abs(y1 - y2), 2) + math.pow(abs(x1 - x2), 2)))

    def calc_angle(self, pt1, pt2, pt3):
        point1 = self.lmList[pt1][1], self.lmList[pt1][2]
        point2 = self.lmList[pt2][1], self.lmList[pt2][2]
        point3 = self.lmList[pt3][1], self.lmList[pt3][2]
        a = self.get_dist(point1, point2)
        b = self.get_dist(point2, point3)
        c = self.get_dist(point1, point3)
        try:
            denom = 2 * a * b
            if denom == 0:
                return 0.0
            value = (math.pow(a, 2) + math.pow(b, 2) - math.pow(c, 2)) / denom
            value = max(-1.0, min(1.0, value))
            radian = math.acos(value)
            angle = radian / math.pi * 180
        except Exception:
            angle = 0.0
        return abs(angle)

    def destroy_node(self):
        try:
            self.hands.close()
        except Exception:
            pass
        try:
            cv.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VirtualPaint()
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
