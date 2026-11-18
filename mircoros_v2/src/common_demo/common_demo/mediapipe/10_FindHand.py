#!/usr/bin/env python3
# encoding: utf-8

#import ros lib
import rclpy
from rclpy.node import Node
from .media_library import *
import cv2 as cv
from interfaces.srv import ArmPosition
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

class FindHand(Node):
    def __init__(self,name, mode=False, maxHands=2, detectorCon=0.5, trackCon=0.5):
        super().__init__(name)
        self.declare_parameter("observation_pose", [90, 150, 12, 20, 90, 30])
        self.observation_pose = self.get_parameter("observation_pose").get_parameter_value().integer_array_value
        self.set_arm_client=self.create_client(ArmPosition, 'set_arm_joints') 
        self.hand_detector = HandDetector()
        #create a publisher
        self.rgb_bridge = CvBridge()
        self.sub_rgb = self.create_subscription(Image,'/ascamera_hp60c/camera_publisher/rgb0/image',self.get_RGBImageCallBack,1)
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
        frame = self.process(rgb_image)
        key = cv2.waitKey(1)
        cv.imshow('dist', frame)
        
    def process(self, frame):
        frame, lmList, bbox = self.hand_detector.findHands(frame)
        self.hand_detector.draw = True
        if len(lmList) != 0:
            hand = self.hand_detector.fingersUp(lmList)
        indexX = (bbox[0] + bbox[2]) / 2
        indexY = (bbox[1] + bbox[3]) / 2
        self.get_logger().info('indexX: %.1f, indexY: %.1f' % (indexX, indexY))
        # print("index X: %.1f, Y: %.1f" % (indexX, indexY))
        return frame


def main():
    print("start it")
    rclpy.init()
    hand_ = FindHand('hand_node')
    rclpy.spin(hand_)


