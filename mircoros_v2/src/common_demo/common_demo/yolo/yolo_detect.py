#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
from colorama import Fore
from interfaces.srv import ArmPosition
from ultralytics import YOLO

class YOLO_DETECT(Node):
    def __init__(self,name):
        super().__init__(name)
        self.declare_parameter('init_pose', [90, 180, 0, 0, 90, 30])
        self.init_pose = self.get_parameter('init_pose').get_parameter_value().integer_array_value
        self.declare_parameter('model_path','')
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.declare_parameter('max_det', 5)
        self.max_det = self.get_parameter('max_det').get_parameter_value().integer_value
        self.set_arm_client = self.create_client(ArmPosition, 'set_arm_joints')
        self.rgb_bridge = CvBridge()
        self.msg2img_bridge = CvBridge()
        self.count=0
        self.sub_rgb = self.create_subscription(Image,'camera_rgb_topic',self.get_RGBImageCallBack,1)   
        self.frame_count=0
        self.prev_time = time.time()
        self.frame_count = 0
        self.fps = 0.0

        self.yolo_model = YOLO(self.model_path, task='detect')
        self.get_logger().info(Fore.GREEN + 'YOLOV26-Detect Model Loaded.' + Fore.RESET)
        self._init_arm_pose()

    def _init_arm_pose(self):
        if not self.set_arm_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn("Failed to connect to set_arm_joints server, skipping init arm pose")
            return
        req = ArmPosition.Request()
        req.request_joints = list(self.init_pose)
        future = self.set_arm_client.call_async(req)
        future.add_done_callback(self._init_arm_done_callback)


    def _init_arm_done_callback(self, future):
        result = future.result()
        if not result.success:
            self.get_logger().warn(f"Init arm pose failed: {list(result.result_joints)}, msg: {result.message}")


    def get_RGBImageCallBack(self,rgb_msg):
        rgb_image = self.rgb_bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        infer_start = time.time()
        results = self.yolo_model.predict(rgb_image, stream=True, max_det=self.max_det, verbose=False)
        for res in results:
            if res.boxes is not None:
                annotated_frame = res.plot()
        infer_end = time.time()
        self.frame_count += 1
        curr_time = time.time()
        time_diff = curr_time - self.prev_time
        if time_diff >= 1.0:
            self.fps = self.frame_count / time_diff
            self.frame_count = 0
            self.prev_time = curr_time
        cv2.putText(annotated_frame,f"FPS: {self.fps:.2f}",(10, 25),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0, 255, 0),1,)
        infer_time = (infer_end - infer_start) * 1000
        cv2.putText(annotated_frame, f"Infer: {infer_time:.1f} ms", (10, 45),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)

        cv2.imshow("YOLOV26-Detect Panel", annotated_frame)
        cv2.waitKey(1)


def main():
    rclpy.init()   
    yolov11_detect_node = YOLO_DETECT('yolov11_detect_node')
    rclpy.spin(yolov11_detect_node)	
    yolov11_detect_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
	main()        
