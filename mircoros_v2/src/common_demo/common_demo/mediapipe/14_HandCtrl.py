#!/usr/bin/env python3
# encoding: utf-8
import threading
import cv2 as cv
import numpy as np
from time import sleep, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import os
from cv_bridge import CvBridge
from .media_common import handDetector
class HandCtrl(Node):
    def __init__(self,name):
        super().__init__(name)
        self.declare_parameter('depth_topic','/camera/depth/image_raw')
        self.declare_parameter('rgb_topic','/ascamera_hp60c/camera_publisher/rgb0/image')
        rgb_topic=self.get_parameter("rgb_topic").get_parameter_value().string_value
        
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(
            Image,
            rgb_topic, 
            self.image_callback,
            10)
        
        self.pub_cmdVel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.car_status = True
        self.stop_status = 0
        self.locking = False
        self.hand_detector = handDetector()
        self.pTime = self.index = 0
        self.event = threading.Event()
        self.event.set()

    def process(self, frame):
        frame = cv.flip(frame, 1)

        frame, _, _ = self.hand_detector.findHands(frame)
        if len(self.hand_detector.lmList) != 0:
            threading.Thread(target=self.hand_threading, args=(self.hand_detector.lmList,)).start()
        else:
            self.pub_cmdVel.publish(Twist())
            
        self.cTime = time()  
        fps = 1 / (self.cTime - self.pTime)
        self.pTime = self.cTime 
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        
        return frame

    def hand_threading(self, lmList):
        if self.event.is_set():
            self.event.clear()
            self.stop_status = 0
            self.index = 0
            fingers = self.hand_detector.fingersUp()
            print("fingers: ", fingers)
            
            twist = Twist()
            
            if sum(fingers) == 5: 
                twist.linear.x = 0.1
                twist.angular.z = 0.0
                self.pub_cmdVel.publish(twist)
                sleep(0.5)
                
            elif sum(fingers) == 3: 
                twist.linear.x = -0.1
                twist.angular.z = 0.0
                self.pub_cmdVel.publish(twist)
                sleep(0.5)
                
            elif sum(fingers) == 1 and fingers[1] == 1: 
                twist.linear.x = 0.1
                twist.angular.z = -0.7
                self.pub_cmdVel.publish(twist)
                sleep(0.5)
                
            elif sum(fingers) == 2 and fingers[1] == 1 and fingers[2] == 1:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.pub_cmdVel.publish(twist)
                sleep(0.5)
                
            elif sum(fingers) == 4 and fingers[0] == 0:
                twist.linear.x = 0.1
                twist.angular.z = 0.7
                self.pub_cmdVel.publish(twist)
                sleep(0.5)
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.pub_cmdVel.publish(twist)
            self.event.set()

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame = self.process(frame)
        cv.imshow('HandDetector', frame)
        
        if cv.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Exiting program")
            cv.destroyAllWindows()
            self.destroy_node()
            rclpy.shutdown()
            exit(0)

    def cancel(self):
        cmd1 = "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
        cmd2 = '''"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'''
        cmd = cmd1 + cmd2
        os.system(cmd)
        print("Shutting down this node.")
        cv.destroyAllWindows()


def main():
    rclpy.init()
    hand_ctrl = HandCtrl('handctrl')
    try:
        rclpy.spin(hand_ctrl)
    except KeyboardInterrupt:
        pass
    finally:
        hand_ctrl.cancel()
        hand_ctrl.destroy_node()
