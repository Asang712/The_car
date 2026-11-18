#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# common lib
import os
import yaml
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge

# ros2 lib
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from std_msgs.msg import String
from sensor_msgs.msg import Image


class SimpleARNode(Node):
    def __init__(self):
        super().__init__('simple_ar_node')

        self.bridge = CvBridge()

        self.window_name = 'Simple AR'
        self.display_width = 960
        self.display_height = 720

        cv.namedWindow(self.window_name, cv.WINDOW_NORMAL)
        cv.resizeWindow(self.window_name, self.display_width, self.display_height)

        self.mirror = False

        self.text_scale = 0.5
        self.text_thickness = 1
        self.text_color = (0, 0, 255)

        self.pub_img = self.create_publisher(
            Image,
            '/simpleAR/camera',
            1
        )

        self.sub_graphics = self.create_subscription(
            String,
            '/Graphics_topic',
            self.choose_graphics,
            1
        )

        topic_name = '/camera/image_raw'

        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.image_callback,
            QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE)
        )

        self.get_logger().info(
            f'Simple AR node started, subscribing to {topic_name}'
        )

        self.index = 0

        self.graphics = [
            "Triangle",
            "Rectangle",
            "Parallelogram",
            "WindMill",
            "TableTennisTable",
            "Ball",
            "Arrow",
            "Knife",
            "Desk",
            "Bench",
            "Stickman",
            "ParallelBars"
        ]

        self.Graphics = self.graphics[self.index]
        self.patternSize = (6, 9)

        self.axis = np.float32([
            [0, 0, -1], [0, 8, -1], [5, 8, -1], [5, 0, -1],
            [1, 2, -1], [1, 6, -1], [4, 2, -1], [4, 6, -1],
            [1, 0, -4], [1, 8, -4], [4, 0, -4], [4, 8, -4],
            [1, 2, -4], [1, 6, -4], [4, 2, -4], [4, 6, -4],
            [0, 1, -4], [3, 2, -1], [2, 2, -3], [3, 2, -3],
            [1, 2, -3], [2, 2, -4], [2, 2, -5], [0, 4, -4],
            [2, 3, -4], [1, 3, -4], [4, 3, -5], [4, 5, -5],
            [1, 2, -3], [1, 6, -3], [5, 2, -3], [5, 6, -3],
            [3, 4, -5], [0, 6, -4], [5, 6, -4], [2, 8, -4],
            [3, 8, -4], [2, 6, -4], [2, 0, -4], [1, 5, -4],
            [3, 0, -4], [3, 2, -4], [0, 3, -4], [1, 2, -4],
            [4, 2, -4], [5, 3, -4], [2, 7, -4], [3, 7, -4],
            [3, 3, -1], [3, 5, -1], [1, 5, -1], [1, 3, -1],
            [3, 3, -3], [3, 5, -3], [1, 5, -3], [1, 3, -3],
            [1, 3, -6], [1, 5, -6], [3, 3, -4], [3, 5, -4],
            [0, 0, -4], [3, 1, -4], [1, 1, -4], [0, 2, -4],
            [2, 4, -4], [4, 4, -4], [0, 8, -4], [5, 8, -4],
            [5, 0, -4], [0, 4, -5], [5, 4, -4], [5, 4, -5],
            [2, 5, -1], [2, 7, -1], [2, 6, -3], [2, 6, -5],
            [2, 5, -3], [2, 7, -3]
        ])

        self.objectPoints = np.zeros((6 * 9, 3), np.float32)
        self.objectPoints[:, :2] = np.mgrid[0:6, 0:9].T.reshape(-1, 2)

        config_path = os.path.join(
            get_package_share_directory('microros_v2_opencv_visual_pkg'),
            'config'
        )

        yaml_path = f'{config_path}/camera.yaml'

        self.cameraMatrix = None
        self.distCoeffs = None
        self.camera_ready = False

        if os.path.exists(yaml_path):
            try:
                with open(yaml_path, 'r') as f:
                    calib_data = yaml.safe_load(f)

                camera_matrix_data = calib_data['camera_matrix']['data']
                distortion_data = calib_data['distortion_coefficients']['data']

                self.cameraMatrix = np.array(
                    camera_matrix_data,
                    dtype=np.float64
                ).reshape(3, 3)

                self.distCoeffs = np.array(
                    distortion_data,
                    dtype=np.float64
                ).reshape(1, -1)

                self.camera_ready = True
                self.get_logger().info(f'Loaded ROS camera config: {yaml_path}')

            except Exception as e:
                self.cameraMatrix = None
                self.distCoeffs = None
                self.camera_ready = False
                self.get_logger().error(f'Failed to load camera.yaml: {e}')
        else:
            self.get_logger().warn(f'camera.yaml not found: {yaml_path}')

    def choose_graphics(self, msg):
        if not isinstance(msg, String):
            return

        if msg.data in self.graphics:
            self.Graphics = msg.data
            self.get_logger().info(f'Graphics changed to: {self.Graphics}')
        else:
            self.graphics_update()

    def graphics_update(self):
        self.index += 1

        if self.index >= len(self.graphics):
            self.index = 0

        self.Graphics = self.graphics[self.index]
        self.get_logger().info(f'Graphics changed to: {self.Graphics}')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception as e:
            self.get_logger().error(f'Image decode failed: {e}')
            return

        if self.mirror:
            frame = cv.flip(frame, 1)

        action = cv.waitKey(10) & 0xFF

        frame = self.process(frame, action)

        cv.imshow(self.window_name, frame)

        if action == ord('q') or action == 113:
            self.get_logger().info('Exiting the application...')
            rclpy.shutdown()

    def process(self, img, action):
        if action == ord('f') or action == ord('F'):
            self.graphics_update()

        if not self.camera_ready:
            cv.putText(
                img,
                'camera.yaml not loaded',
                (20, 55),
                cv.FONT_HERSHEY_SIMPLEX,
                self.text_scale,
                self.text_color,
                self.text_thickness
            )

            cv.putText(
                img,
                self.Graphics,
                (20, 30),
                cv.FONT_HERSHEY_SIMPLEX,
                self.text_scale,
                self.text_color,
                self.text_thickness
            )

            self.pub_img.publish(
                self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            )

            return img

        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        retval, corners = cv.findChessboardCorners(
            gray,
            self.patternSize,
            None,
            flags=cv.CALIB_CB_ADAPTIVE_THRESH
            + cv.CALIB_CB_NORMALIZE_IMAGE
            + cv.CALIB_CB_FAST_CHECK
        )

        if retval:
            corners = cv.cornerSubPix(
                gray,
                corners,
                (11, 11),
                (-1, -1),
                (
                    cv.TERM_CRITERIA_EPS
                    + cv.TERM_CRITERIA_MAX_ITER,
                    30,
                    0.001
                )
            )

            retval, rvec, tvec, inliers = cv.solvePnPRansac(
                self.objectPoints,
                corners,
                self.cameraMatrix,
                self.distCoeffs
            )

            if retval:
                image_points, jacobian = cv.projectPoints(
                    self.axis,
                    rvec,
                    tvec,
                    self.cameraMatrix,
                    self.distCoeffs
                )

                img = self.draw(img, corners, image_points)

        cv.putText(
            img,
            self.Graphics,
            (20, 30),
            cv.FONT_HERSHEY_SIMPLEX,
            self.text_scale,
            self.text_color,
            self.text_thickness
        )

        self.pub_img.publish(
            self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        )

        return img

    def draw(self, img, corners, image_points):
        img_pts = np.int32(image_points).reshape(-1, 2)

        if self.Graphics == "Triangle":
            cv.drawContours(
                img,
                [np.array([img_pts[14], img_pts[15], img_pts[23]])],
                -1,
                (255, 0, 0),
                -1
            )

        elif self.Graphics == "Rectangle":
            cv.drawContours(
                img,
                [np.array([img_pts[12], img_pts[13], img_pts[15], img_pts[14]])],
                -1,
                (0, 255, 0),
                -1
            )

        elif self.Graphics == "Parallelogram":
            cv.drawContours(
                img,
                [np.array([img_pts[12], img_pts[10], img_pts[15], img_pts[9]])],
                -1,
                (65, 105, 225),
                1
            )

        elif self.Graphics == "WindMill":
            cv.drawContours(
                img,
                [np.array([img_pts[60], img_pts[38], img_pts[61], img_pts[21]])],
                -1,
                (0, 0, 255),
                -1
            )

            cv.drawContours(
                img,
                [np.array([img_pts[10], img_pts[14], img_pts[58], img_pts[21]])],
                -1,
                (0, 0, 255),
                -1
            )

            cv.drawContours(
                img,
                [np.array([img_pts[62], img_pts[63], img_pts[23], img_pts[21]])],
                -1,
                (0, 0, 255),
                -1
            )

            cv.drawContours(
                img,
                [np.array([img_pts[25], img_pts[64], img_pts[65], img_pts[21]])],
                -1,
                (0, 0, 255),
                -1
            )

            cv.line(
                img,
                tuple(img_pts[64]),
                tuple(img_pts[35]),
                (0, 255, 0),
                3
            )

        elif self.Graphics == "TableTennisTable":
            cv.line(img, tuple(img_pts[0]), tuple(img_pts[60]), (255, 0, 0), 3)

            for i in range(1, 4):
                cv.line(
                    img,
                    tuple(img_pts[i]),
                    tuple(img_pts[65 + i]),
                    (255, 0, 0),
                    3
                )

            cv.drawContours(
                img,
                [np.array([img_pts[60], img_pts[66], img_pts[67], img_pts[68]])],
                -1,
                (0, 255, 0),
                -1
            )

            cv.drawContours(
                img,
                [np.array([img_pts[23], img_pts[69], img_pts[71], img_pts[70]])],
                -1,
                (0, 0, 255),
                -1
            )

        elif self.Graphics == "Ball":
            cv.circle(
                img,
                tuple(img_pts[22]),
                30,
                (0, 0, 255),
                -1
            )

        elif self.Graphics == "Arrow":
            cv.drawContours(
                img,
                [np.array([img_pts[13], img_pts[34], img_pts[36]])],
                -1,
                (0, 255, 0),
                -1
            )

            cv.drawContours(
                img,
                [np.array([img_pts[37], img_pts[15], img_pts[10], img_pts[38]])],
                -1,
                (0, 255, 0),
                -1
            )

        elif self.Graphics == "Knife":
            cv.drawContours(
                img,
                [np.array([img_pts[58], img_pts[24], img_pts[35], img_pts[47]])],
                -1,
                (160, 252, 0),
                -1
            )

            cv.drawContours(
                img,
                [np.array([img_pts[40], img_pts[38], img_pts[21], img_pts[41]])],
                -1,
                (30, 144, 255),
                -1
            )

            cv.drawContours(
                img,
                [np.array([img_pts[42:46]])],
                -1,
                (0, 0, 255),
                -1
            )

        elif self.Graphics == "Desk":
            for i in range(4):
                cv.line(
                    img,
                    tuple(img_pts[4 + i]),
                    tuple(img_pts[12 + i]),
                    (163, 148, 128),
                    3
                )

            cv.drawContours(
                img,
                [np.array([img_pts[14], img_pts[12], img_pts[13], img_pts[15]])],
                -1,
                (0, 199, 140),
                -1
            )

        elif self.Graphics == "Bench":
            for i in range(4):
                cv.line(
                    img,
                    tuple(img_pts[48 + i]),
                    tuple(img_pts[52 + i]),
                    (255, 0, 0),
                    3
                )

            cv.drawContours(
                img,
                [img_pts[52:56]],
                -1,
                (0, 0, 255),
                -1
            )

            cv.drawContours(
                img,
                [img_pts[54:58]],
                -1,
                (139, 69, 19),
                -1
            )

        elif self.Graphics == "Stickman":
            cv.line(img, tuple(img_pts[18]), tuple(img_pts[4]), (0, 0, 255), 3)
            cv.line(img, tuple(img_pts[18]), tuple(img_pts[6]), (0, 0, 255), 3)
            cv.line(img, tuple(img_pts[18]), tuple(img_pts[21]), (0, 0, 255), 3)
            cv.line(img, tuple(img_pts[21]), tuple(img_pts[19]), (0, 0, 255), 3)
            cv.line(img, tuple(img_pts[21]), tuple(img_pts[20]), (0, 0, 255), 3)
            cv.line(img, tuple(img_pts[21]), tuple(img_pts[22]), (0, 0, 255), 3)
            cv.circle(img, tuple(img_pts[22]), 15, (0, 0, 255), -1)

            cv.line(img, tuple(img_pts[74]), tuple(img_pts[72]), (0, 255, 0), 3)
            cv.line(img, tuple(img_pts[74]), tuple(img_pts[73]), (0, 255, 0), 3)
            cv.line(img, tuple(img_pts[74]), tuple(img_pts[37]), (0, 255, 0), 3)
            cv.line(img, tuple(img_pts[37]), tuple(img_pts[76]), (0, 255, 0), 3)
            cv.line(img, tuple(img_pts[37]), tuple(img_pts[77]), (0, 255, 0), 3)
            cv.line(img, tuple(img_pts[37]), tuple(img_pts[75]), (0, 255, 0), 3)
            cv.circle(img, tuple(img_pts[75]), 15, (0, 255, 0), -1)

        elif self.Graphics == "ParallelBars":
            for i in range(4):
                cv.line(
                    img,
                    tuple(img_pts[4 + i]),
                    tuple(img_pts[12 + i]),
                    (255, 0, 0),
                    3
                )

            cv.line(img, tuple(img_pts[8]), tuple(img_pts[9]), (0, 0, 255), 3)
            cv.line(img, tuple(img_pts[10]), tuple(img_pts[11]), (0, 0, 255), 3)

        return img


def main(args=None):
    rclpy.init(args=args)

    node = SimpleARNode()

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