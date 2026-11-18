import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
from rclpy.node import Node
import rclpy
from interfaces.srv import ArmPosition
from common_demo.mediapipe.media_library import *
from sensor_msgs.msg import Image

class MediapipeDetectNode(Node):
	def __init__(self, name):
		super().__init__(name)
		self.rgb_bridge = CvBridge()
		self.depth_bridge = CvBridge()
		self.hand_detector = HandDetector()
		self.pr_time = time.time()
		self.pTime = self.cTime = 0
		self.declare_parameter('init_pose', [90, 180, 0, 0, 90, 30])
		self.init_pose = self.get_parameter('init_pose').get_parameter_value().integer_array_value
		self.set_arm_client = self.create_client(ArmPosition, 'set_arm_joints')
		self._init_arm_pose()
		self.sub_rgb = self.create_subscription(Image, '/ascamera_hp60c/camera_publisher/rgb0/image', self.callback, 5)
		self.start_time = 0.0

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

	def callback(self, color_msg):
		frame = self.rgb_bridge.imgmsg_to_cv2(color_msg, "bgr8")
		frame, lmList, bbox = self.hand_detector.findHands(frame)
		gesture = ""
		if len(lmList) != 0:
			gesture = self.hand_detector.get_gesture(lmList)
		self.cTime = time.time()
		fps = 1 / (self.cTime - self.pTime)
		self.pTime = self.cTime
		fps_text = "FPS : " + str(int(fps))
		cv.putText(frame, fps_text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
		gesture_text = "Gesture : " + gesture
		cv.putText(frame, gesture_text, (20, 65), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
		cv.imshow('frame', frame)
		cv.waitKey(1)

def main():
	rclpy.init()
	mediapipe_detect = MediapipeDetectNode('MediapipeDetect_node')
	rclpy.spin(mediapipe_detect)
	cv.destroyAllWindows()
