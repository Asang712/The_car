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


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        self.bridge = CvBridge()

        topic_name = '/camera/image_raw'

        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.image_callback,
            QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE)
        )

        self.get_logger().info(
            f'Object detection node started, subscribing to {topic_name}'
        )

        config_path = os.path.join(
            get_package_share_directory('microros_v2_opencv_visual_pkg'),
            'config'
        )

        models_dir = os.path.join(os.path.expanduser('~'), 'MODELS')

        with open(f'{config_path}/object_detection_coco.txt', 'r') as f:
            self.class_names = f.read().split('\n')

        self.COLORS = np.random.uniform(
            0,
            255,
            size=(len(self.class_names), 3)
        )

        self.model = cv.dnn.readNet(
            model=f'{models_dir}/frozen_inference_graph.pb',
            config=f'{config_path}/ssd_mobilenet_v2_coco.txt',
            framework='TensorFlow'
        )

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception as e:
            self.get_logger().error(f'Image decode failed: {e}')
            return

        frame = self.target_detection(frame)

        cv.imshow('Object Detection', frame)

        if cv.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info('Exiting the application...')
            rclpy.shutdown()

    def target_detection(self, image):
        image_height, image_width, _ = image.shape

        blob = cv.dnn.blobFromImage(
            image=image,
            size=(300, 300),
            mean=(104, 117, 123),
            swapRB=True
        )

        self.model.setInput(blob)
        output = self.model.forward()

        for detection in output[0, 0, :, :]:
            confidence = detection[2]

            if confidence > 0.4:
                class_id = int(detection[1])
                class_index = class_id - 1

                if class_index < 0 or class_index >= len(self.class_names):
                    continue

                class_name = self.class_names[class_index]
                color = self.COLORS[class_index]

                x1 = int(detection[3] * image_width)
                y1 = int(detection[4] * image_height)
                x2 = int(detection[5] * image_width)
                y2 = int(detection[6] * image_height)

                cv.rectangle(
                    image,
                    (x1, y1),
                    (x2, y2),
                    color,
                    thickness=2
                )

                cv.putText(
                    image,
                    f'{class_name}: {confidence:.2f}',
                    (x1, y1 - 5),
                    cv.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    color,
                    2
                )

        return image


def main(args=None):
    rclpy.init(args=args)

    node = ObjectDetectionNode()

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