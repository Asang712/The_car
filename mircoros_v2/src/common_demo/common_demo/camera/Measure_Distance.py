from cv_bridge import CvBridge
import cv2
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image
from interfaces.srv import ArmPosition
import numpy as np

encoding = ['16UC1', '32FC1']


class ImagetNode(Node):
	def __init__(self, name):
		super().__init__(name)

		self.declare_parameter('init_pose', [90, 170, 0, -10, 90, 30])
		self.init_pose = self.get_parameter('init_pose').get_parameter_value().integer_array_value
		self.sub_depth = self.create_subscription(
			Image, '/ascamera_hp60c/camera_publisher/depth0/image_raw',
			self.get_DepthImgCallBack, 5
		)
		self.depth_bridge = CvBridge()

		self.set_arm_client = self.create_client(ArmPosition, 'set_arm_joints')
		self._init_arm_pose()

		self.window_name = "result_image"
		self.y = 240
		self.x = 320
		cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
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
			self.get_logger().warn(
				f"Init arm pose failed: {list(result.result_joints)}, msg: {result.message}")

	def click_callback(self, event, x, y, flags, params):
		if event == 1:
			self.x = x
			self.y = y

	def get_DepthImgCallBack(self, depth_frame):
		depth_image = self.depth_bridge.imgmsg_to_cv2(depth_frame, desired_encoding='passthrough')
		depth_to_color_image = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.45), cv2.COLORMAP_JET)
		depth_image_info = depth_image.astype(np.float32)

		# Raw depth at clicked pixel (mm -> m)
		raw_depth_mm = depth_image_info[self.y, self.x]
		raw_depth_m = round(raw_depth_mm / 1000.0, 4)

		cv2.putText(depth_to_color_image, f"depth: {raw_depth_m:.4f} m",
					(10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
		cv2.setMouseCallback(self.window_name, self.click_callback)
		cv2.circle(depth_to_color_image, (self.x, self.y), 1, (0, 0, 0), 10)
		cv2.imshow(self.window_name, depth_to_color_image)
		key = cv2.waitKey(1)

	def __del__(self):
		cv2.destroyAllWindows()

def main():
	rclpy.init()
	depth_color_image = ImagetNode('Measure_Distance_node')
	try:
		rclpy.spin(depth_color_image)
	except KeyboardInterrupt:
		print("Shutting down")
	finally:
		depth_color_image.destroy_node()
		rclpy.shutdown()