#!/usr/bin/env python3
import os
import time
import threading
import yaml
import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
class IPCameraPublisher(Node):
    def __init__(self):
        super().__init__('esp32_ip_camera_publisher')
        self.declare_parameter('camera_url', 'http://192.168.12.32:81/stream')
        self.camera_url = self.get_parameter('camera_url').value
        self.declare_parameter('startup_delay', 3.0)
        self.declare_parameter('reconnect_min_delay', 1.0)
        self.declare_parameter('reconnect_max_delay', 30.0)
        self.declare_parameter('ost_file', '')
        self.ost_file = self.get_parameter('ost_file').get_parameter_value().string_value
        if not self.ost_file:
            # 直跑 camera_driver.py 未传 ost_file 时，回退到包内标定文件，
            # 避免 CameraInfo 因空路径而停止发布。
            try:
                self.ost_file = os.path.join(
                    get_package_share_directory('microros_v2_bringup'),
                    'config', 'ost.yaml'
                )
                self.get_logger().info(f'ost_file not set, fallback to package default: {self.ost_file}')
            except Exception as e:
                self.get_logger().warn(f'Cannot resolve default ost_file: {e}')

        self.startup_delay = self._get_float_parameter('startup_delay', 3.0)
        self.reconnect_min_delay = self._get_float_parameter('reconnect_min_delay', 1.0)
        self.reconnect_max_delay = self._get_float_parameter('reconnect_max_delay', 30.0)
        self._next_reconnect_time = time.time() + self.startup_delay
        self._current_reconnect_delay = self.reconnect_min_delay
        self.bridge = CvBridge()
        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', image_qos)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 1)

        self._camera_info_msg = self._load_ost_file(self.ost_file)
        if self._camera_info_msg is not None:
            self._info_timer = self.create_timer(0.5, self._publishCameraInfo)
            self.get_logger().info(f'CameraInfo publisher started, ost_file: {self.ost_file}')
        else:
            self._info_timer = None
            self.get_logger().warn('No valid ost_file, CameraInfo will not be published')

        self.declare_parameter('img_save_path', '/tmp/images')
        self.save_path = self.get_parameter('img_save_path').get_parameter_value().string_value

        self.cap = None
        self.latest_frame = None
        self._running = True
        self.save_service = self.create_service(Trigger, '/save_image', self._saveImageCallback)

        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()
        self.get_logger().info(f'camera node started, camera_url: {self.camera_url}')

    def _load_ost_file(self, ost_path):
        """读取ost.yaml标定文件，构建CameraInfo消息"""
        if not ost_path or not os.path.isfile(ost_path):
            self.get_logger().warn(f'ost_file not found: "{ost_path}"')
            return None
        try:
            with open(ost_path, 'r') as f:
                calib = yaml.safe_load(f)
            msg = CameraInfo()
            msg.header.frame_id = 'camera'
            msg.width = int(calib['image_width'])
            msg.height = int(calib['image_height'])
            msg.distortion_model = calib.get('distortion_model', 'plumb_bob')

            # 内参矩阵 K (3x3)
            msg.k = [float(v) for v in calib['camera_matrix']['data']]
            # 畸变系数 D
            msg.d = [float(v) for v in calib['distortion_coefficients']['data']]
            # 矫正矩阵 R (3x3)
            msg.r = [float(v) for v in calib['rectification_matrix']['data']]
            # 投影矩阵 P (3x4)
            msg.p = [float(v) for v in calib['projection_matrix']['data']]

            self.get_logger().info(
                f'Loaded calibration: {msg.width}x{msg.height}, '
                f'fx={msg.k[0]:.2f}, fy={msg.k[4]:.2f}'
            )
            return msg
        except Exception as e:
            self.get_logger().error(f'Failed to parse ost_file: {e}')
            return None

    def _publishCameraInfo(self):
        """publish CameraInfo message"""
        if self._camera_info_msg is None:
            return
        self._camera_info_msg.header.stamp = self.get_clock().now().to_msg()
        self.camera_info_pub.publish(self._camera_info_msg)

    def _get_float_parameter(self, name, default_value):
        value = self.get_parameter(name).value
        try:
            return float(value)
        except (TypeError, ValueError):
            self.get_logger().warn(f'Invalid float parameter {name}={value}, fallback to {default_value}')
            return default_value

    def open_camera(self):
        if self.cap is not None:
            self.cap.release()
        self.cap = cv2.VideoCapture(self.camera_url, cv2.CAP_FFMPEG)
        # 尽量降低缓存延迟
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open IP camera stream')
        else:
            self.get_logger().info('IP camera stream opened successfully')
            self._current_reconnect_delay = self.reconnect_min_delay

    def _reader_loop(self):
        time.sleep(self.startup_delay)
        publish_interval = 1.0 / 30.0
        last_publish_time = 0.0

        while self._running:
            if self.cap is None or not self.cap.isOpened():
                self._try_reconnect()
                time.sleep(0.01)
                continue

            ret, frame = self.cap.read()
            if not ret or frame is None:
                self.get_logger().warn('Failed to read frame, reconnecting...')
                self.open_camera()
                time.sleep(0.01)
                continue

            now = time.time()
            if now - last_publish_time >= publish_interval:
                self.publish_raw_image(frame)
                last_publish_time = now

    def _try_reconnect(self):
        """指数退避重连"""
        now = time.time()
        if now < self._next_reconnect_time:
            return
        self.get_logger().warn(
            f'Camera not opened, trying reconnect... '
            f'(next retry in {self._current_reconnect_delay:.1f}s)'
        )
        self.open_camera()
        self._current_reconnect_delay = min(
            self._current_reconnect_delay * 2.0,
            self.reconnect_max_delay
        )
        self._next_reconnect_time = now + self._current_reconnect_delay

    def publish_raw_image(self, frame):
        if frame.shape[1] != 640 or frame.shape[0] != 480:
            frame = cv2.resize(frame, (640, 480))
        self.latest_frame = frame.copy()
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        self.image_pub.publish(msg)

    def _saveImageCallback(self, request: Trigger.Request, response: Trigger.Response):
        if self.latest_frame is None:
            error_msg = 'No images are currently available. Please check if your camera is working properly.'
            response.success = False
            response.message = error_msg
            self.get_logger().warn(error_msg)
            return response
        try:
            cv2.imwrite(self.save_path, self.latest_frame)
            response.success = True
            response.message = self.save_path
            self.get_logger().info(f'Image saved: {self.save_path}')
        except Exception as e:
            response.success = False
            response.message = f'Image saving failed:{str(e)}'
            self.get_logger().error(f'Image saving failed:{str(e)}')
        return response

    def destroy_node(self):
        self._running = False
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=5.0)
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IPCameraPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
