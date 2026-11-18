from cv_bridge import CvBridge
import cv2
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image
from interfaces.srv import ArmPosition

encoding = ['16UC1', '32FC1']


class ImagetNode(Node):
	def __init__(self, name):
		super().__init__(name)
		self.declare_parameter('init_pose', [90, 180, 0, 0, 90,30])
		self.init_pose = self.get_parameter('init_pose').get_parameter_value().integer_array_value

		self.sub_depth = self.create_subscription(Image,'/ascamera_hp60c/camera_publisher/depth0/image_raw',self.get_DepthImgCallBack,5)
		self.depth_bridge = CvBridge()

		self.set_arm_client = self.create_client(ArmPosition, 'set_arm_joints')
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
			self.get_logger().warn(
				f"Init arm pose failed: {list(result.result_joints)}, msg: {result.message}")

	def get_DepthImgCallBack(self,depth_frame):
		depth_image = self.depth_bridge.imgmsg_to_cv2(depth_frame, desired_encoding='passthrough')
		depth_to_color_image = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.35), cv2.COLORMAP_JET)
		key = cv2.waitKey(1)
		cv2.imshow("result_image", depth_to_color_image)
	def __del__(self):
		cv2.destroyAllWindows()

def main():
	rclpy.init()
	depth_color_image = ImagetNode('Depth_Color_Image_node')
	try:
		rclpy.spin(depth_color_image)
	except KeyboardInterrupt:
		print("Shutting down")
	finally:
		depth_color_image.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()