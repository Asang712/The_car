#!/usr/bin/env python3
# encoding: utf-8
import os
os.environ["MEDIAPIPE_DISABLE_TF"] = "1"
os.environ["MEDIAPIPE_DISABLE_GPU"] = "1"
import rclpy
from rclpy.node import Node
import mediapipe as mp
import cv2 as cv
import numpy as np
import time
import os
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from interfaces.srv import ArmPosition
class HandDetector(Node):
    def __init__(self,name, mode=False, maxHands=2, detectorCon=0.5, trackCon=0.5):
        super().__init__(name)
        self.declare_parameter("observation_pose", [90, 150, 12, 20, 90, 30])
        self.observation_pose = self.get_parameter("observation_pose").get_parameter_value().integer_array_value
        self.set_arm_client=self.create_client(ArmPosition, 'set_arm_joints') 
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.pTime=0
        self.mpHand = mp.solutions.hands
        self.mpDraw = mp.solutions.drawing_utils
        self.hands = self.mpHand.Hands(
            static_image_mode=mode,
            max_num_hands=maxHands,
            min_detection_confidence=detectorCon,
            min_tracking_confidence=trackCon)
        self.lmDrawSpec = mp.solutions.drawing_utils.DrawingSpec(color=(0, 0, 255), thickness=-1, circle_radius=6)
        self.drawSpec = mp.solutions.drawing_utils.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2)
        #create a publisher
        self.rgb_bridge = CvBridge()
        self.sub_rgb = self.create_subscription(Image,'/ascamera_hp60c/camera_publisher/rgb0/image',self.get_RGBImageCallBack,self.qos_profile)
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


    def get_RGBImageCallBack(self,msg):

        rgb_image = self.rgb_bridge.imgmsg_to_cv2(msg, "bgr8")
        frame, img = self.pubHandsPoint(rgb_image, draw=False)
        dist = self.frame_combine(frame, img)
        cTime = time.time()
        if self.pTime == 0:
            fps = 0
        else:
            fps = 1 / (cTime - self.pTime)
        self.pTime = cTime
        text = "FPS : " + str(int(fps))
        cv.putText(dist, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        key = cv2.waitKey(1)
        cv.imshow('dist', dist)

    def pubHandsPoint(self, frame, draw=True):
        img = np.zeros(frame.shape, np.uint8)
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.hands.process(img_RGB)
        if self.results.multi_hand_landmarks:
            for i in range(len(self.results.multi_hand_landmarks)):
                if draw: self.mpDraw.draw_landmarks(frame, self.results.multi_hand_landmarks[i], self.mpHand.HAND_CONNECTIONS, self.lmDrawSpec, self.drawSpec)
                self.mpDraw.draw_landmarks(img, self.results.multi_hand_landmarks[i], self.mpHand.HAND_CONNECTIONS, self.lmDrawSpec, self.drawSpec)
        return frame, img
    
    def frame_combine(self,frame, src):
        if len(frame.shape) == 3:
            frameH, frameW = frame.shape[:2]
            srcH, srcW = src.shape[:2]
            dst = np.zeros((max(frameH, srcH), frameW + srcW, 3), np.uint8)
            dst[:, :frameW] = frame[:, :]
            dst[:, frameW:] = src[:, :]
        else:
            src = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
            frameH, frameW = frame.shape[:2]
            imgH, imgW = src.shape[:2]
            dst = np.zeros((frameH, frameW + imgW), np.uint8)
            dst[:, :frameW] = frame[:, :]
            dst[:, frameW:] = src[:, :]
        return dst

def main():
    print("start it")
    rclpy.init()
    hand_detector = HandDetector('hand_detector')
    rclpy.spin(hand_detector)
