#!/usr/bin/env python3
# encoding: utf-8

import math
import os
import time

import cv2 as cv
import mediapipe as mp
import numpy as np
import rclpy
from cv_bridge import CvBridge
from arm_msgs.msg import ArmJoints, ArmJoint
from rclpy.node import Node
from sensor_msgs.msg import Image
from interfaces.srv import ArmPosition
class handDetector(Node):
    MIN_PINCH_ANGLE = 2.0
    MAX_TRACK_ANGLE = 70.0
    MIN_GRIP_SERVO = 0
    MAX_GRIP_SERVO = 180
    CLOSE_HOLD_PERCENT = 5.0
    CLOSE_RELEASE_PERCENT = 8.0
    SERVO_SMOOTHING = 0.35
    MIN_PUBLISH_DELTA = 2

    def __init__(self, name):
        super().__init__(name)
        self.declare_parameter("observation_pose", [90, 180, 0, 0, 90, 90])
        self.observation_pose = self.get_parameter("observation_pose").get_parameter_value().integer_array_value
        self.set_arm_client=self.create_client(ArmPosition, 'set_arm_joints')
        self.lmList = []
        self.results = None
        self.pTime = 0.0
        self.volBar = 400.0
        self.pinch_percent = 0.0
        self.grip_servo = float(self.MAX_GRIP_SERVO)
        self.last_angle = 0.0
        self.last_publish_servo = None
        self.close_hold = False
        self.mpHand = mp.solutions.hands
        self.mpDraw = mp.solutions.drawing_utils
        self.hands = self.mpHand.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )

        self.rgb_bridge = CvBridge()
        self.TargetAngle_pub = self.create_publisher(ArmJoints, "arm6_joints", 10)
        self.pub_SingleTargetAngle = self.create_publisher(ArmJoint, "arm_joint", 10)
        self.sub_rgb = self.create_subscription(Image, "/ascamera_hp60c/camera_publisher/rgb0/image", self.get_RGBImageCallBack, 1)
        if not self.set_arm_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn("Failed to connect to the arm server,skipping init arm pose")
        else:
            req = ArmPosition.Request()
            req.request_joints = list(self.observation_pose)
            self.get_logger().info(f"Init arm pose: {list(req.request_joints)}")
            self._init_arm_future = self.set_arm_client.call_async(req)
            self._init_arm_future.add_done_callback(self._init_arm_done_callback)

    def _init_arm_done_callback(self, future):
        resp = future.result()
        if not resp.success:
            self.get_logger().warn(f"Init arm pose failed: {resp.message}")
            
            
    def pubSix_Arm(self, joints, id=6, angle=180.0, runtime=2000):
        arm_joint = ArmJoints()
        arm_joint.joint1 = joints[0]
        arm_joint.joint2 = joints[1]
        arm_joint.joint3 = joints[2]
        arm_joint.joint4 = joints[3]
        arm_joint.joint5 = joints[4]
        arm_joint.joint6 = joints[5]
        arm_joint.time = runtime
        self.TargetAngle_pub.publish(arm_joint)

    def pubSingleArm(self, joint_id, joint_angle, run_time=500):
        arm_joint = ArmJoint()
        arm_joint.joint = joint_angle
        arm_joint.id = joint_id
        arm_joint.time = run_time
        self.pub_SingleTargetAngle.publish(arm_joint)

    def get_RGBImageCallBack(self, msg):
        frame = self.rgb_bridge.imgmsg_to_cv2(msg, "bgr8")
        img = self.findHands(frame)
        lmList = self.findPosition(frame, draw=False)
        if lmList:
            raw_angle = self.calc_angle(4, 0, 8)
            x1, y1 = lmList[4][1], lmList[4][2]
            x2, y2 = lmList[8][1], lmList[8][2]
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            self.update_grip_state(raw_angle)

            cv.circle(img, (x1, y1), 15, (255, 0, 255), cv.FILLED)
            cv.circle(img, (x2, y2), 15, (255, 0, 255), cv.FILLED)
            cv.line(img, (x1, y1), (x2, y2), (255, 0, 255), 3)
            cv.circle(img, (cx, cy), 15, (255, 0, 255), cv.FILLED)
            if self.pinch_percent <= 10:
                cv.circle(img, (cx, cy), 15, (0, 255, 0), cv.FILLED)

        fps = self.get_fps()
        cv.rectangle(img, (50, 150), (85, 400), (255, 0, 0), 3)
        cv.rectangle(img, (50, int(self.volBar)), (85, 400), (0, 255, 0), cv.FILLED)
        cv.putText(img, f"{int(self.pinch_percent)}%", (40, 450), cv.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 3)
        cv.putText(frame, f"FPS : {int(fps)}", (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        dst = self.frame_combine(frame, img)
        cv.imshow("FingerCtrl", dst)

        if cv.waitKey(1) & 0xFF == ord("q"):
            cv.destroyAllWindows()
            rclpy.shutdown()

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
            radian = math.acos((math.pow(a, 2) + math.pow(b, 2) - math.pow(c, 2)) / (2 * a * b))
            angle = radian / math.pi * 180
        except Exception:
            angle = 0
        return abs(angle)

    def findHands(self, frame, draw=True):
        img = np.zeros(frame.shape, np.uint8)
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.hands.process(img_RGB)
        if draw and self.results.multi_hand_landmarks:
            for hand_lms in self.results.multi_hand_landmarks:
                self.mpDraw.draw_landmarks(img, hand_lms, self.mpHand.HAND_CONNECTIONS)
        return img

    def findPosition(self, frame, draw=True):
        self.lmList = []
        if self.results.multi_hand_landmarks:
            for idx, lm in enumerate(self.results.multi_hand_landmarks[0].landmark):
                h, w, _ = frame.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                self.lmList.append([idx, cx, cy])
                if draw:
                    cv.circle(frame, (cx, cy), 6, (0, 0, 255), cv.FILLED)
        return self.lmList

    def update_grip_state(self, raw_angle):
        prev_close_hold = self.close_hold
        self.last_angle = raw_angle

        angle_for_percent = float(np.clip(raw_angle, 0.0, self.MAX_TRACK_ANGLE))
        self.pinch_percent = float(np.interp(angle_for_percent, [0.0, self.MAX_TRACK_ANGLE], [0.0, 100.0]))
        self.volBar = float(np.interp(angle_for_percent, [0.0, self.MAX_TRACK_ANGLE], [400.0, 150.0]))

        pinch_angle = max(raw_angle, self.MIN_PINCH_ANGLE)
        target_servo = float(np.clip(360.0 / pinch_angle, self.MIN_GRIP_SERVO, self.MAX_GRIP_SERVO))

        if self.close_hold:
            if self.pinch_percent > self.CLOSE_RELEASE_PERCENT:
                self.close_hold = False
            else:
                target_servo = float(self.MAX_GRIP_SERVO)
        elif self.pinch_percent <= self.CLOSE_HOLD_PERCENT:
            self.close_hold = True
            target_servo = float(self.MAX_GRIP_SERVO)

        if self.close_hold:
            self.grip_servo = float(self.MAX_GRIP_SERVO)
        else:
            self.grip_servo = (
                self.SERVO_SMOOTHING * target_servo
                + (1.0 - self.SERVO_SMOOTHING) * self.grip_servo
            )

        publish_servo = int(round(self.grip_servo))
        state_changed = prev_close_hold != self.close_hold
        if (
            self.last_publish_servo is None
            or state_changed
            or abs(publish_servo - self.last_publish_servo) >= self.MIN_PUBLISH_DELTA
        ):
            self.pubSingleArm(6, publish_servo)
            self.last_publish_servo = publish_servo

    def frame_combine(self, frame, src):
        if len(frame.shape) == 3:
            frameH, frameW = frame.shape[:2]
            srcH, srcW = src.shape[:2]
            dst = np.zeros((max(frameH, srcH), frameW + srcW, 3), np.uint8)
            dst[:, :frameW] = frame[:, :]
            dst[:, frameW:] = src[:, :]
        else:
            src = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
            frameH, frameW = frame.shape[:2]
            _, imgW = src.shape[:2]
            dst = np.zeros((frameH, frameW + imgW), np.uint8)
            dst[:, :frameW] = frame[:, :]
            dst[:, frameW:] = src[:, :]
        return dst

    def get_fps(self):
        cTime = time.time()
        if self.pTime == 0:
            self.pTime = cTime
            return 0.0
        fps = 1.0 / max(cTime - self.pTime, 1e-6)
        self.pTime = cTime
        return fps


def main():
    rclpy.init()
    hand_detector = handDetector("hand_detector")
    try:
        rclpy.spin(hand_detector)
    finally:
        hand_detector.destroy_node()
        cv.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
