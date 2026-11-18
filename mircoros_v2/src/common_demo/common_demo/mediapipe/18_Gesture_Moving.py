import os
import threading
import time
import numpy as np
import time
import mediapipe as mp
import cv2 as cv
import rclpy
from cv_bridge import CvBridge
from arm_msgs.msg import ArmJoints, ArmJoint
import sys
from rclpy.node import Node
from sensor_msgs.msg import Image
from interfaces.srv import ArmPosition
from common_demo.mediapipe.media_library import *
class HandDetector:
    def __init__(self, mode=False, maxHands=1, detectorCon=0.5, trackCon=0.5):
        self.mpHand = mp.solutions.hands
        self.draw = True
        self.mpDraw = mp.solutions.drawing_utils
        self.hands = self.mpHand.Hands(
            static_image_mode=mode,
            max_num_hands=maxHands,
            min_detection_confidence=detectorCon,
            min_tracking_confidence=trackCon
        )

    def findHands(self, frame, draw=True):
        lmList = []
        self.cxList = []
        self.cyList = []
        bbox = 0, 0, 0, 0
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.hands.process(img_RGB)
        if self.results.multi_hand_landmarks:
            for i in range(len(self.results.multi_hand_landmarks)):
                if not draw: self.mpDraw.draw_landmarks(frame, self.results.multi_hand_landmarks[i], self.mpHand.HAND_CONNECTIONS)
                for id, lm in enumerate(self.results.multi_hand_landmarks[i].landmark):
                    h, w, c = frame.shape
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    lmList.append([id, cx, cy])
                    self.cxList.append(cx)
                    self.cyList.append(cy)
        if len(self.cxList) != 0 and len(self.cyList) != 0:
            xmin, xmax = min(self.cxList), max(self.cxList)
            ymin, ymax = min(self.cyList), max(self.cyList)
            bbox = xmin, ymin, xmax, ymax
            if draw: cv.rectangle(frame, (xmin - 20, ymin - 20), (xmax + 20, ymax + 20), (0, 255, 0), 2)
        return frame, lmList, bbox

    def fingersUp(self, lmList):
        fingers = []
        point1 = lmList[4][1:3]
        point2 = lmList[3][1:3]
        point3 = lmList[2][1:3]
        point4 = lmList[1][1:3]
        if (abs(calc_angle(point1, point2, point3)) > 150.0) and (abs(calc_angle(point2, point3, point4)) > 150.0):
            fingers.append(1)
        else:
            fingers.append(0)
        tipIds = [4, 8, 12, 16, 20]
        for id in range(1, 5):
            if lmList[tipIds[id]][2] < lmList[tipIds[id] - 2][2]:
                fingers.append(1)
            else:
                fingers.append(0)
        return fingers

    def ThumbTOforefinger(self, lmList):
        point1 = lmList[4][1:3]
        point2 = lmList[0][1:3]
        point3 = lmList[8][1:3]
        return abs(calc_angle(point1, point2, point3))

    def get_gesture(self, lmList):
        gesture = ""
        fingers = self.fingersUp(lmList)
        if fingers[2] == fingers[3] == fingers[4] == 1:
            if self.ThumbTOforefinger(lmList) < 10:
                gesture = "OK"
        if fingers[1] == fingers[2] == 1 and sum(fingers) == 2:
            gesture = "Yes"
        try:
            if self.cyList[4] == max(self.cyList):
                gesture = "Thumb_down"
        except Exception as e:
            print("e: ", e)
        return gesture
class MediapipeDetectNode(Node):
    FPS_SMOOTHING = 0.85
    ARM_CMD_RUNTIME_MS = 1500
    ACTION_SETTLE_TIME = 1.6

    def __init__(self, name):
        super().__init__(name)
        self.declare_parameter("observation_pose", [90, 180, 0, 0, 90, 90])
        self.init_joints = self.get_parameter("observation_pose").get_parameter_value().integer_array_value
        self.set_arm_client = self.create_client(ArmPosition, 'set_arm_joints')
        self.rgb_bridge = CvBridge()
        self.hand_detector = HandDetector()
        self.pTime = 0.0
        self.fps = 0.0
        self.action_event = threading.Event()
        self.action_event.set()
        self.current_gesture = "No hand"
        self.current_action = None
        self.current_fingers = []

        self.pub_SingleTargetAngle = self.create_publisher(ArmJoint, "arm_joint", 10)
        self.TargetAngle_pub = self.create_publisher(ArmJoints, "arm6_joints", 10)
        self.sub_rgb = self.create_subscription(Image, "/ascamera_hp60c/camera_publisher/rgb0/image", self.get_RGBImageCallBack, 1)
        if not self.set_arm_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn("Failed to connect to the arm server,skipping init arm pose")
        else:
            req = ArmPosition.Request()
            req.request_joints = list(self.init_joints)
            self.get_logger().info(f"Init arm pose: {list(req.request_joints)}")
            self._init_arm_future = self.set_arm_client.call_async(req)
            self._init_arm_future.add_done_callback(self._init_arm_done_callback)

    def _init_arm_done_callback(self, future):
        resp = future.result()
        if not resp.success:
            self.get_logger().warn(f"Init arm pose failed: {resp.message}")
    def pubSix_Arm(self, joints, id=6, angle=180.0, runtime=1500):
        arm_joint = ArmJoints()
        arm_joint.joint1 = joints[0]
        arm_joint.joint2 = joints[1]
        arm_joint.joint3 = joints[2]
        arm_joint.joint4 = joints[3]
        arm_joint.joint5 = joints[4]
        arm_joint.joint6 = joints[5]
        arm_joint.time = runtime
        self.TargetAngle_pub.publish(arm_joint)

    def pubSingleArm(self, joint_id, joint_angle, run_time):
        arm_joint = ArmJoint()
        arm_joint.joint = joint_angle
        arm_joint.id = joint_id
        arm_joint.time = run_time
        self.pub_SingleTargetAngle.publish(arm_joint)

    def get_RGBImageCallBack(self, color_msg):
        frame = self.rgb_bridge.imgmsg_to_cv2(color_msg, "bgr8")
        self.process(frame)

    def process(self, frame):
        frame, lmList, _ = self.hand_detector.findHands(frame, draw=False)
        self.current_gesture, self.current_action, self.current_fingers = self.detect_current_action(lmList)

        fps = self.get_fps()
        self.draw_status(frame, fps)
        cv.imshow("frame", frame)

        key = cv.waitKey(1) & 0xFF
        if key == ord(" "):
            self.trigger_current_action()
        elif key == ord("q"):
            cv.destroyAllWindows()
            rclpy.shutdown()

    def detect_current_action(self, lmList):
        if not lmList:
            return "No hand", None, []

        gesture = self.hand_detector.get_gesture(lmList)
        fingers = self.hand_detector.fingersUp(lmList)

        if gesture == "Yes":
            return "Yes", "dance", fingers
        if gesture == "OK":
            return "OK", "swing", fingers
        if gesture == "Thumb_down":
            return "Thumb_down", "thumb_down", fingers
        if sum(fingers) == 1:
            return "One", "nod", fingers
        if fingers[1] == fingers[4] == 1 and sum(fingers) == 2:
            return "Rock", "shake", fingers
        if sum(fingers) == 5:
            return "Five", "applaud", fingers
        if gesture:
            return gesture, None, fingers
        return "Unknown", None, fingers

    def draw_status(self, frame, fps):
        h, _ = frame.shape[:2]
        panel_top = max(h - 70, 0)
        cv.rectangle(frame, (0, panel_top), (360, h), (0, 255, 0), cv.FILLED)
        cv.putText(
            frame,
            f"Gesture: {self.current_gesture}",
            (10, panel_top + 28),
            cv.FONT_HERSHEY_PLAIN,
            2,
            (255, 0, 0),
            2,
        )

        action_text = "SPACE: run action" if self.current_action else "SPACE: no mapped action"
        if not self.action_event.is_set():
            action_text = "Action running..."
        cv.putText(
            frame,
            action_text,
            (10, panel_top + 56),
            cv.FONT_HERSHEY_PLAIN,
            1.5,
            (255, 255, 255),
            2,
        )
        cv.putText(frame, f"FPS : {int(fps)}", (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)

    def trigger_current_action(self):
        if self.current_action is None or not self.action_event.is_set():
            return
        threading.Thread(target=self.Arm_Moving_threading, args=(self.current_action,), daemon=True).start()

    def Arm_Moving_threading(self, action_name):
        if not self.action_event.is_set():
            return

        self.action_event.clear()
        try:
            if action_name == "dance":
                self.dance()
            elif action_name == "swing":
                for _ in range(3):
                    time.sleep(0.1)
                    self.pubSix_Arm([80, 135, 0, 15, 90, 180])
                    time.sleep(0.5)
                    self.pubSix_Arm([110, 135, 0, 15, 90, 90])
                    time.sleep(0.5)
            elif action_name == "thumb_down":
                self.pubSix_Arm([90, 0, 180, 0, 90, 180])
                time.sleep(0.5)
                self.pubSix_Arm([90, 0, 180, 0, 90, 90])
                time.sleep(1.5)
            elif action_name == "nod":
                self.arm_nod()
            elif action_name == "shake":
                self.shake()
            elif action_name == "applaud":
                self.arm_applaud()
        finally:
            time.sleep(self.ACTION_SETTLE_TIME)
            self.init_pose()
            self.action_event.set()

    def dance(self):
        time_sleep = 0.5
        self.pubSix_Arm([90, 90, 90, 90, 90, 90])
        time.sleep(time_sleep)
        self.pubSix_Arm([90, 60, 120, 60, 90, 90])
        time.sleep(time_sleep)
        self.pubSix_Arm([90, 45, 135, 45, 90, 90])
        time.sleep(time_sleep)
        self.pubSix_Arm([90, 60, 120, 60, 90, 90])
        time.sleep(time_sleep)
        self.pubSix_Arm([90, 90, 90, 90, 90, 90])
        time.sleep(time_sleep)
        self.pubSix_Arm([90, 100, 80, 80, 90, 90])
        time.sleep(time_sleep)
        self.pubSix_Arm([90, 120, 60, 60, 90, 90])
        time.sleep(time_sleep)
        self.pubSix_Arm([90, 135, 45, 45, 90, 90])
        time.sleep(time_sleep)
        self.pubSix_Arm([90, 90, 90, 90, 90, 90])
        time.sleep(time_sleep)
        self.pubSix_Arm([90, 90, 90, 20, 90, 150])
        time.sleep(time_sleep)
        self.pubSix_Arm([90, 90, 90, 90, 90, 90])
        time.sleep(time_sleep)
        self.pubSix_Arm([90, 90, 90, 20, 90, 150])
        time.sleep(time_sleep)
        self.pubSix_Arm([0, 90, 90, 90, 0, 90])
        time.sleep(time_sleep)
        self.pubSix_Arm([0, 90, 180, 0, 0, 90])
        time.sleep(time_sleep)
        self.pubSingleArm(6, 180, 1500)
        time.sleep(time_sleep)
        self.pubSingleArm(6, 30, 1500)
        time.sleep(time_sleep)
        self.pubSix_Arm([90, 90, 90, 90, 90, 90])
        time.sleep(time_sleep)
        self.pubSix_Arm([90, 135, 0, 45, 90, 90])
        time.sleep(time_sleep)

    def init_pose(self):
        self.pubSix_Arm(self.init_joints, runtime=self.ARM_CMD_RUNTIME_MS)
        time.sleep(self.ACTION_SETTLE_TIME)

    def arm_applaud(self):
        for _ in range(3):
            self.pubSix_Arm([90, 145, 0, 71, 90, 31])
            time.sleep(0.5)
            self.pubSix_Arm([91, 144, 0, 71, 90, 168])
            time.sleep(0.5)

    def shake(self):
        for _ in range(3):
            self.pubSix_Arm([138, 94, 92, 88, 92, 172])
            time.sleep(0.5)
            self.pubSix_Arm([48, 94, 92, 87, 92, 172])
            time.sleep(0.5)

    def arm_nod(self):
        for _ in range(3):
            self.pubSix_Arm([90, 89, 93, 93, 89, 32])
            time.sleep(0.5)
            self.pubSix_Arm([90, 89, 93, 33, 89, 32])
            time.sleep(0.5)

    def get_fps(self):
        cTime = time.time()
        if self.pTime == 0:
            self.pTime = cTime
            self.fps = 0.0
            return self.fps
        instant_fps = 1.0 / max(cTime - self.pTime, 1e-6)
        self.pTime = cTime
        if self.fps == 0.0:
            self.fps = instant_fps
        else:
            self.fps = self.FPS_SMOOTHING * self.fps + (1.0 - self.FPS_SMOOTHING) * instant_fps
        return self.fps


def main():
    rclpy.init()
    mediapipe_detect = MediapipeDetectNode("MediapipeDetect_node")
    try:
        rclpy.spin(mediapipe_detect)
    finally:
        mediapipe_detect.destroy_node()
        cv.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
