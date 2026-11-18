#!/usr/bin/env python3

# encoding: utf-8
import time
import os
import dlib
import cv2 as cv
import numpy as np
import rclpy
from rclpy.node import Node
import mediapipe as mp
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
from interfaces.srv import ArmPosition

class FaceLandmarks(Node):

    def __init__(self, name):
        super().__init__(name)
        self.declare_parameter("observation_pose", [90, 150, 12, 20, 90, 30])
        self.observation_pose = self.get_parameter("observation_pose").get_parameter_value().integer_array_value
        self.set_arm_client=self.create_client(ArmPosition, 'set_arm_joints') 
        self.dat_file =os.path.join(os.path.expanduser('~'), 'MODELS','mediapipe','shape_predictor_68_face_landmarks.dat')
        self.hog_face_detector = dlib.get_frontal_face_detector()

        self.dlib_facelandmark = dlib.shape_predictor(self.dat_file)

        self.rgb_bridge = CvBridge()

        self.sub_rgb = self.create_subscription(
            Image, '/ascamera_hp60c/camera_publisher/rgb0/image', self.get_RGBImageCallBack, 5)
        if not self.set_arm_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn("Failed to connect to the arm server,skipping init arm pose")
        else:
            req = ArmPosition.Request()
            req.request_joints = list(self.observation_pose)
            self.get_logger().info(f"Init arm pose: {list(req.request_joints)}")
            self._init_arm_future = self.set_arm_client.call_async(req)
            self._init_arm_future.add_done_callback(self._init_arm_done_callback)
            
    def _init_arm_done_callback(self, future):
        resp = future.result()
        if not resp.success:
            self.get_logger().warn(f"Init arm pose failed: {resp.message}")
    def get_RGBImageCallBack(self, msg):
        rgb_image = self.rgb_bridge.imgmsg_to_cv2(msg, "bgr8")

        frame = self.get_face(rgb_image, draw=False)

        frame = self.prettify_face(
            frame, eye=True, lips=True, eyebrow=True, draw=True)

        key = cv2.waitKey(1)

        cv.imshow('frame', frame)


    def get_face(self, frame, draw=True):

        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        self.faces = self.hog_face_detector(gray)

        for face in self.faces:

            self.face_landmarks = self.dlib_facelandmark(gray, face)

            if draw:

                for n in range(68):

                    x = self.face_landmarks.part(n).x

                    y = self.face_landmarks.part(n).y

                    cv.circle(frame, (x, y), 2, (0, 255, 255), 2)

                    cv.putText(frame, str(n), (x, y),
                               cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        return frame

    def get_lmList(self, frame, p1, p2, draw=True):

        lmList = []

        if len(self.faces) != 0:

            for n in range(p1, p2):

                x = self.face_landmarks.part(n).x

                y = self.face_landmarks.part(n).y

                lmList.append([x, y])

                if draw:

                    next_point = n + 1

                    if n == p2 - 1:
                        next_point = p1

                    x2 = self.face_landmarks.part(next_point).x

                    y2 = self.face_landmarks.part(next_point).y

                    cv.line(frame, (x, y), (x2, y2), (0, 255, 0), 1)

        return lmList

    def get_lipList(self, frame, lipIndexlist, draw=True):

        lmList = []

        if len(self.faces) != 0:

            for n in range(len(lipIndexlist)):

                x = self.face_landmarks.part(lipIndexlist[n]).x

                y = self.face_landmarks.part(lipIndexlist[n]).y

                lmList.append([x, y])

                if draw:

                    next_point = n + 1

                    if n == len(lipIndexlist) - 1:
                        next_point = 0

                    x2 = self.face_landmarks.part(lipIndexlist[next_point]).x

                    y2 = self.face_landmarks.part(lipIndexlist[next_point]).y

                    cv.line(frame, (x, y), (x2, y2), (0, 255, 0), 1)

        return lmList

    def prettify_face(self, frame, eye=True, lips=True, eyebrow=True, draw=True):

        if eye:

            leftEye = self.get_lmList(frame, 36, 42)

            rightEye = self.get_lmList(frame, 42, 48)

            if draw:

                if len(leftEye) != 0:
                    frame = cv.fillConvexPoly(
                        frame, np.mat(leftEye), (0, 0, 0))

                if len(rightEye) != 0:
                    frame = cv.fillConvexPoly(
                        frame, np.mat(rightEye), (0, 0, 0))

        if lips:

            lipIndexlistA = [51, 52, 53, 54, 64, 63, 62]

            lipIndexlistB = [48, 49, 50, 51, 62, 61, 60]

            lipsUpA = self.get_lipList(frame, lipIndexlistA, draw=True)

            lipsUpB = self.get_lipList(frame, lipIndexlistB, draw=True)

            lipIndexlistA = [57, 58, 59, 48, 67, 66]

            lipIndexlistB = [54, 55, 56, 57, 66, 65, 64]

            lipsDownA = self.get_lipList(frame, lipIndexlistA, draw=True)

            lipsDownB = self.get_lipList(frame, lipIndexlistB, draw=True)

            if draw:

                if len(lipsUpA) != 0:
                    frame = cv.fillConvexPoly(
                        frame, np.mat(lipsUpA), (249, 0, 226))

                if len(lipsUpB) != 0:
                    frame = cv.fillConvexPoly(
                        frame, np.mat(lipsUpB), (249, 0, 226))

                if len(lipsDownA) != 0:
                    frame = cv.fillConvexPoly(
                        frame, np.mat(lipsDownA), (249, 0, 226))

                if len(lipsDownB) != 0:
                    frame = cv.fillConvexPoly(
                        frame, np.mat(lipsDownB), (249, 0, 226))

        if eyebrow:

            lefteyebrow = self.get_lmList(frame, 17, 22)

            righteyebrow = self.get_lmList(frame, 22, 27)

            if draw:

                if len(lefteyebrow) != 0:
                    frame = cv.fillConvexPoly(
                        frame, np.mat(lefteyebrow), (255, 255, 255))

                if len(righteyebrow) != 0:
                    frame = cv.fillConvexPoly(
                        frame, np.mat(righteyebrow), (255, 255, 255))

        return frame


def main():

    print("start it")

    rclpy.init()

    landmarks = FaceLandmarks("face_landmarks_node")

    rclpy.spin(landmarks)

