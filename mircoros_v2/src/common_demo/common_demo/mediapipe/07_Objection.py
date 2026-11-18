#!/usr/bin/env python3
# encoding: utf-8

import cv2 as cv
import time
import rclpy
from rclpy.node import Node
import mediapipe as mp
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
from interfaces.srv import ArmPosition
class Objectron(Node):
    def __init__(self,name):
        super().__init__(name)
        self.declare_parameter("observation_pose", [90, 150, 12, 20, 90, 30])
        self.observation_pose = self.get_parameter("observation_pose").get_parameter_value().integer_array_value
        self.set_arm_client=self.create_client(ArmPosition, 'set_arm_joints') 
        self.staticMode=False
        self.maxObjects=5
        self.minDetectionCon=0.5
        self.minTrackingCon=0.99
        self.index=0
        self.modelNames = ['Shoe', 'Chair', 'Cup', 'Camera']
        self.mpObjectron = mp.solutions.objectron
        self.mpDraw = mp.solutions.drawing_utils
        self.mpobjectron = self.mpObjectron.Objectron(
            self.staticMode, self.maxObjects, self.minDetectionCon, self.minTrackingCon, self.modelNames[self.index])
        self.rgb_bridge = CvBridge()
        self.sub_rgb = self.create_subscription(Image,'/ascamera_hp60c/camera_publisher/rgb0/image',self.get_RGBImageCallBack,5)
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
        if msg.data is None or len(msg.data) == 0:
            self.get_logger().warn("Received empty image message")
            return
        rgb_image = self.rgb_bridge.imgmsg_to_cv2(msg, "bgr8")
        action = cv2.waitKey(1)
        if action == ord('f') or action == ord('F') : self.configUP()
        frame = self.findObjectron(rgb_image)
        cv.imshow('frame', frame)

    def findObjectron(self, frame):
        cv.putText(frame, self.modelNames[self.index], (int(frame.shape[1] / 2) - 30, 30),
                   cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 3)
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        results = self.mpobjectron.process(img_RGB)
        if results.detected_objects:
            for id, detection in enumerate(results.detected_objects):
                self.mpDraw.draw_landmarks(frame, detection.landmarks_2d, self.mpObjectron.BOX_CONNECTIONS)
                self.mpDraw.draw_axis(frame, detection.rotation, detection.translation)
        return frame

    def configUP(self):
        self.index += 1
        if self.index>=4:self.index=0
        self.mpobjectron = self.mpObjectron.Objectron(
            self.staticMode, self.maxObjects, self.minDetectionCon, self.minTrackingCon, self.modelNames[self.index])

def main():
    print("start it")
    rclpy.init()
    objectron = Objectron("objectron_node")
    rclpy.spin(objectron)