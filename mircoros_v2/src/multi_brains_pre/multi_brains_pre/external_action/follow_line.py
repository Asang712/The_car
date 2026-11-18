#!/usr/bin/env python3
# encoding: utf-8
# ros lib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# common lib
import os
import sys
import time
import cv2 as cv
from colorama import Fore
from ament_index_python.packages import get_package_share_directory
from .follow_common import color_follow, simplePID, ManyImgs
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
# Mapping from color name to its line index (1-based) in the HSV profile file
COLOR_INDEX = {'red': 1, 'green': 2, 'blue': 3, 'yellow': 4}


class LineDetect(Node):

	def __init__(self, name):
		super().__init__(name)
		# publisher / subscriber
		self.pub_cmdVel = self.create_publisher(Twist, '/cmd_vel', 10)

		camera_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

		self.rgb_image_sub = self.create_subscription(Image, '/camera/image_raw', self.rgb_img_callback, camera_qos)
		self.rgb_bridge = CvBridge()

		# target color is passed as a string parameter: red / green / blue / yellow
		self.declare_parameter('color', 'red')
		self.color_name = self.get_parameter('color').get_parameter_value().string_value.strip().lower()

		# load the HSV range of the target color from the HSV profile file
		self.hsv_text = os.path.join(get_package_share_directory('microros_v2_bringup'), 'config', 'HSV.text')
		self.hsv_range = self.load_hsv_range(self.color_name)
		if self.hsv_range is None:
			self.get_logger().error(Fore.RED + f"Cannot follow line: no HSV range for color '{self.color_name}'" + Fore.RESET)
			raise SystemExit(1)

		self.color = color_follow()
		self.scale = 1000
		self.linear = 0.15
		self.FollowLinePID = (30, 0, 10)
		self.max_angular = 0.5  # cap the turning angular velocity (rad/s)
		self.PID_init()

		# state used to detect when the line disappears (line following finished)
		self.line_detected_once = False
		self.last_seen_time = None
		self.lost_timeout = 2.0  # seconds without the line before finishing

		# exit with a failure code if no line is found within this many seconds after startup
		self.start_time = time.time()
		self.startup_timeout = 15.0
		self.startup_timer = self.create_timer(1.0, self.check_startup_timeout)

		self.get_logger().info(Fore.GREEN + f"Following '{self.color_name}' line, HSV range: {self.hsv_range}" + Fore.RESET)

	def load_hsv_range(self, color_name):
		'''Load the (lower, upper) HSV range of color_name from the HSV profile file.'''
		index = COLOR_INDEX.get(color_name)
		if index is None:
			self.get_logger().error(f"Invalid color '{color_name}'. Valid options: red, green, blue, yellow")
			return None
		try:
			with open(self.hsv_text, 'r') as f:
				lines = [line.strip() for line in f if line.strip()]
		except Exception as e:
			self.get_logger().error(f"Failed to read HSV file: {e}")
			return None
		if index > len(lines):
			self.get_logger().error(f"HSV profile for '{color_name}' (line {index}) not found")
			return None
		values = list(map(int, lines[index - 1].split(',')))
		if len(values) != 6:
			self.get_logger().error("Invalid HSV format in file")
			return None
		return ((values[0], values[1], values[2]), (values[3], values[4], values[5]))

	def PID_init(self):
		self.PID_controller = simplePID(
			[0, 0],
			[self.FollowLinePID[0] / 1.0 / self.scale, 0],
			[self.FollowLinePID[1] / 1.0 / self.scale, 0],
			[self.FollowLinePID[2] / 1.0 / self.scale, 0])

	def check_startup_timeout(self):
		'''Exit with a failure code if no line is found within the startup timeout.'''
		if not self.line_detected_once and (time.time() - self.start_time) > self.startup_timeout:
			self.get_logger().error(Fore.RED + f"No '{self.color_name}' line found within {self.startup_timeout:.0f}s, exiting" + Fore.RESET)
			self.finish(code=2)

	def execute(self, point_x):
		'''Drive the chassis so the detected line stays centered in the view.'''
		twist = Twist()
		[z_Pid, _] = self.PID_controller.update([(point_x - 320) * 1.0 / 16, 0])
		# limit the turning angular velocity to keep the motion smooth
		z_Pid = max(-self.max_angular, min(self.max_angular, z_Pid))
		twist.angular.z = z_Pid
		twist.linear.x = self.linear
		if abs(point_x - 320) < 40:
			twist.angular.z = 0.0
		self.pub_cmdVel.publish(twist)

	def process(self, rgb_img):
		rgb_img = cv.resize(rgb_img, (640, 480))
		rgb_img, binary, circle = self.color.line_follow(rgb_img, self.hsv_range)
		# a valid line is found when the enclosing circle radius is large enough
		if len(circle) != 0 and circle[2] >= 4:
			if not self.line_detected_once:
				self.line_detected_once = True
				self.startup_timer.cancel()
			self.last_seen_time = time.time()
			self.execute(circle[0])
		else:
			# stop the chassis while the line is not visible
			self.pub_cmdVel.publish(Twist())
			# finish once the line disappears after having been followed
			if self.line_detected_once and self.last_seen_time is not None \
					and (time.time() - self.last_seen_time) > self.lost_timeout:
				self.get_logger().info(Fore.GREEN + "Line disappeared, follow line finished" + Fore.RESET)
				self.finish()
		return rgb_img, binary

	def rgb_img_callback(self, msg):
		rgb_image = self.rgb_bridge.imgmsg_to_cv2(msg, 'bgr8')
		result_image, binary = self.process(rgb_image)
		if len(binary) != 0:
			cv.imshow('frame', ManyImgs(1, ([result_image, binary])))
		else:
			cv.imshow('frame', result_image)
		cv.waitKey(1)

	def finish(self, code=0):
		'''Stop the chassis and exit the process with the given exit code.'''
		self.pub_cmdVel.publish(Twist())
		cv.destroyAllWindows()
		raise SystemExit(code)


def main():
	rclpy.init()
	linedetect = LineDetect("follow_line")
	exit_code = 0
	try:
		rclpy.spin(linedetect)
	except SystemExit as e:
		exit_code = e.code if isinstance(e.code, int) else 0
	except KeyboardInterrupt:
		pass
	finally:
		linedetect.pub_cmdVel.publish(Twist())
		linedetect.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()
	sys.exit(exit_code)


if __name__ == '__main__':
	main()
