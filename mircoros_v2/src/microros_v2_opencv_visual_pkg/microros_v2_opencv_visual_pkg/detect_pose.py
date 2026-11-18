#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
class OpenPoseDetectionNode(Node):
    def __init__(self):
        super().__init__('openpose_detection_node')
        self.bridge = CvBridge()
        topic_name = '/camera/image_raw'
        
        self.subscription = self.create_subscription(
            Image,
            topic_name, 
            self.image_callback,
            QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE))

        # Load the COCO class names and DNN model for object detection
        config_dir = os.path.join(get_package_share_directory('microros_v2_opencv_visual_pkg'), 'config')
        models_dir = os.path.join(os.path.expanduser('~'), 'MODELS')
        with open(os.path.join(config_dir, 'object_detection_coco.txt'), 'r') as f: 
            self.class_names = f.read().split('\n')
        self.COLORS = np.random.uniform(0, 255, size=(len(self.class_names), 3))
        self.model = cv.dnn.readNet(model=os.path.join(models_dir, 'frozen_inference_graph.pb'), config=os.path.join(config_dir, 'ssd_mobilenet_v2_coco.txt'), framework='TensorFlow')

        # Set up OpenPose model
        self.BODY_PARTS = {
            "Nose": 0, "Neck": 1, "RShoulder": 2, "RElbow": 3, "RWrist": 4,
            "LShoulder": 5, "LElbow": 6, "LWrist": 7, "RHip": 8, "RKnee": 9,
            "RAnkle": 10, "LHip": 11, "LKnee": 12, "LAnkle": 13, "REye": 14,
            "LEye": 15, "REar": 16, "LEar": 17, "Background": 18
        }
        self.POSE_PAIRS = [
            ["Neck", "RShoulder"], ["Neck", "LShoulder"], ["RShoulder", "RElbow"],
            ["RElbow", "RWrist"], ["LShoulder", "LElbow"], ["LElbow", "LWrist"],
            ["Neck", "RHip"], ["RHip", "RKnee"], ["RKnee", "RAnkle"], ["Neck", "LHip"],
            ["LHip", "LKnee"], ["LKnee", "LAnkle"], ["Neck", "Nose"], ["Nose", "REye"],
            ["REye", "REar"], ["Nose", "LEye"], ["LEye", "LEar"]
        ]
        self.net = cv.dnn.readNetFromTensorflow(os.path.join(models_dir, 'graph_opt.pb'))

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image decode failed: {e}')
            return

        frame = self.openpose(frame)
        # frame = self.target_detection(frame)

        # Show the processed frame directly in a window
        cv.imshow('OpenPose Detection', frame)

        # Wait for 'q' key press to stop the loop
        if cv.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Exiting the application...")
            rclpy.shutdown()

    def target_detection(self, image):
        image_height, image_width, _ = image.shape
        # Create blob from image
        blob = cv.dnn.blobFromImage(image=image, size=(300, 300), mean=(104, 117, 123), swapRB=True)
        self.model.setInput(blob)
        output = self.model.forward()

        for detection in output[0, 0, :, :]:
            confidence = detection[2]
            if confidence > 0.4:
                class_id = detection[1]
                class_name = self.class_names[int(class_id) - 1]
                color = self.COLORS[int(class_id)]
                box_x = detection[3] * image_width
                box_y = detection[4] * image_height
                box_width = detection[5] * image_width
                box_height = detection[6] * image_height
                cv.rectangle(image, (int(box_x), int(box_y)), (int(box_width), int(box_height)), color, thickness=2)
                cv.putText(image, class_name, (int(box_x), int(box_y - 5)), cv.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        return image

    def openpose(self, frame):
        frameHeight, frameWidth = frame.shape[:2]
        self.net.setInput(cv.dnn.blobFromImage(frame, 1.0, (368, 368), (127.5, 127.5, 127.5), swapRB=True, crop=False))
        out = self.net.forward()
        out = out[:, :19, :, :]  # MobileNet output [1, 57, -1, -1], we only need the first 19 elements
        points = []
        for i in range(len(self.BODY_PARTS)):
            heatMap = out[0, i, :, :]
            _, conf, _, point = cv.minMaxLoc(heatMap)
            x = (frameWidth * point[0]) / out.shape[3]
            y = (frameHeight * point[1]) / out.shape[2]
            points.append((int(x), int(y)) if conf > 0.2 else None)
        for pair in self.POSE_PAIRS:
            partFrom = pair[0]
            partTo = pair[1]
            idFrom = self.BODY_PARTS[partFrom]
            idTo = self.BODY_PARTS[partTo]
            if points[idFrom] and points[idTo]:
                cv.line(frame, points[idFrom], points[idTo], (0, 255, 0), 3)
                cv.ellipse(frame, points[idFrom], (3, 3), 0, 0, 360, (0, 0, 255), cv.FILLED)
                cv.ellipse(frame, points[idTo], (3, 3), 0, 0, 360, (0, 0, 255), cv.FILLED)
        return frame

def main(args=None):
    rclpy.init(args=args)
    node = OpenPoseDetectionNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        cv.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
