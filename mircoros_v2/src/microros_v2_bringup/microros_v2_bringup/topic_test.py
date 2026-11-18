#!/usr/bin/env python3
# encoding: utf-8

import time
from collections import deque

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, LaserScan


class TopicStat:
    """Statistics for a single topic"""

    def __init__(self, name):
        self.name = name
        self.total_count = 0        # total received messages
        self.window_count = 0       # messages received in current report window
        self.last_recv_time = None  # timestamp of last received message

    def update(self):
        self.total_count += 1
        self.window_count += 1
        self.last_recv_time = time.time()

    def report(self, window_sec):
        hz = self.window_count / window_sec if window_sec > 0 else 0.0
        if self.last_recv_time is None:
            return "[%s] NO DATA!!! No message has ever been received" % self.name
        since_last = time.time() - self.last_recv_time
        return "[%s] window: %d msgs (%.2f Hz), total: %d msgs, last received: %.1f s ago" % (
            self.name, self.window_count, hz, self.total_count, since_last)

    def reset_window(self):
        self.window_count = 0


class TopicTestNode(Node):
    def __init__(self):
        super().__init__('topic_test_node')
        self.declare_parameter('report_interval', 10.0)
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('image_topic', '/camera/image_raw')

        self.report_interval = self.get_parameter('report_interval').value
        imu_topic = self.get_parameter('imu_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        image_topic = self.get_parameter('image_topic').value

        self.imu_stat = TopicStat(imu_topic)
        self.scan_stat = TopicStat(scan_topic)
        self.image_stat = TopicStat(image_topic)

        # image display
        self.bridge = CvBridge()
        self._window_name = 'topic_test image'
        self._window_available = True
        self._frame_times = deque(maxlen=30)  # timestamps of recent frames for realtime FPS

        # use sensor_data QoS (BestEffort) for sensor topics, compatible with Reliable publishers
        self.imu_sub = self.create_subscription(
            Imu, imu_topic, self.imu_callback, qos_profile_sensor_data)
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, qos_profile_sensor_data)
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data)

        self.report_timer = self.create_timer(self.report_interval, self.report_callback)
        self.get_logger().info(
            "Topic test node started, subscribing: %s, %s, %s, printing statistics every %.0f seconds"
            % (imu_topic, scan_topic, image_topic, self.report_interval))

    def imu_callback(self, msg):
        self.imu_stat.update()

    def scan_callback(self, msg):
        self.scan_stat.update()

    def image_callback(self, msg):
        self.image_stat.update()
        self._frame_times.append(time.time())
        if not self._window_available:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn("Failed to convert image: %s" % e, throttle_duration_sec=10.0)
            return
        # realtime FPS from recent frame intervals
        if len(self._frame_times) >= 2:
            elapsed = self._frame_times[-1] - self._frame_times[0]
            fps = (len(self._frame_times) - 1) / elapsed if elapsed > 0 else 0.0
        else:
            fps = 0.0
        cv2.putText(frame, "FPS: %.1f" % fps, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        try:
            cv2.imshow(self._window_name, frame)
            cv2.waitKey(1)
        except cv2.error as e:
            # headless environment (no display), disable the window
            self._window_available = False
            self.get_logger().warn("cv2.imshow unavailable (%s); image window disabled." % e)

    def report_callback(self):
        self.get_logger().info("========== Topic Data Statistics ==========")
        for stat in (self.imu_stat, self.scan_stat, self.image_stat):
            line = stat.report(self.report_interval)
            if stat.window_count == 0:
                self.get_logger().warn(line)
            else:
                self.get_logger().info(line)
            stat.reset_window()
        self.get_logger().info("============================================")


def main(args=None):
    rclpy.init(args=args)
    node = TopicTestNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
