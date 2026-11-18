#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
hand_detector.py

MediaPipe 手部检测器，从 microros_v2_mediapipe_pkg/media_common.py 提取，
仅保留 gesture tracker 所需的 HandDetector 及其依赖方法。

运行前准备:
    source ~/workspaces/MicroROS-V2-ROS2-SRC/start_yolo_ros2_env.sh
"""

import os
os.environ.setdefault("MEDIAPIPE_DISABLE_TF", "1")
os.environ.setdefault("MEDIAPIPE_DISABLE_GPU", "1")

import math
import threading
import time

import cv2 as cv
import mediapipe as mp
import numpy as np


class HandDetector:
    """MediaPipe 手部检测器 / MediaPipe hand detector."""

    def __init__(self, mode=False, maxHands=1, detectorCon=0.5, trackCon=0.5):
        self.tipIds = [4, 8, 12, 16, 20]
        self.mpHand = mp.solutions.hands
        self.mpDraw = mp.solutions.drawing_utils
        self.hands = self.mpHand.Hands(
            static_image_mode=mode,
            max_num_hands=maxHands,
            min_detection_confidence=detectorCon,
            min_tracking_confidence=trackCon,
        )
        self.lmList = []
        self.cxList = []
        self.cyList = []
        self.results = None
        self.lmDrawSpec = self.mpDraw.DrawingSpec(color=(0, 0, 255), thickness=-1, circle_radius=6)
        self.drawSpec = self.mpDraw.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2)
        self.last_action = ""
        self.repeat = 0
        self.count = 0
        task_1 = threading.Thread(target=self.task_control_timeout, name="task_control_timeout", daemon=True)
        task_1.start()

    def close(self):
        try:
            self.hands.close()
        except Exception:
            pass

    def get_dist(self, point1, point2):
        x1, y1 = point1
        x2, y2 = point2
        return abs(math.sqrt(math.pow(abs(y1 - y2), 2) + math.pow(abs(x1 - x2), 2)))

    def calc_angle(self, pt1, pt2, pt3):
        if len(self.lmList) <= max(pt1, pt2, pt3):
            return 0.0
        point1 = self.lmList[pt1][1], self.lmList[pt1][2]
        point2 = self.lmList[pt2][1], self.lmList[pt2][2]
        point3 = self.lmList[pt3][1], self.lmList[pt3][2]
        a = self.get_dist(point1, point2)
        b = self.get_dist(point2, point3)
        c = self.get_dist(point1, point3)
        try:
            denom = 2 * a * b
            if denom == 0:
                return 0.0
            value = (math.pow(a, 2) + math.pow(b, 2) - math.pow(c, 2)) / denom
            value = max(-1.0, min(1.0, value))
            radian = math.acos(value)
            angle = radian / math.pi * 180
        except Exception:
            angle = 0.0
        return abs(angle)

    def findHands(self, frame, draw=True):
        self.lmList = []
        self.cxList = []
        self.cyList = []
        bbox_area = 0
        img = np.zeros(frame.shape, np.uint8)
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.hands.process(img_RGB)
        if self.results.multi_hand_landmarks:
            for hand_landmarks in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(frame, hand_landmarks, self.mpHand.HAND_CONNECTIONS, self.lmDrawSpec, self.drawSpec)
                self.mpDraw.draw_landmarks(img, hand_landmarks, self.mpHand.HAND_CONNECTIONS, self.lmDrawSpec, self.drawSpec)
                for idx, lm in enumerate(hand_landmarks.landmark):
                    h, w, _ = frame.shape
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    self.lmList.append([idx, cx, cy])
                    self.cxList.append(cx)
                    self.cyList.append(cy)
        if self.cxList and self.cyList:
            xmin, xmax = min(self.cxList), max(self.cxList)
            ymin, ymax = min(self.cyList), max(self.cyList)
            bbox_width = xmax - xmin
            bbox_height = ymax - ymin
            bbox_area = bbox_width * bbox_height // 100
        return frame, img, bbox_area

    def findPoint(self, point):
        if len(self.lmList) <= point:
            return 0, 0
        return self.lmList[point][1], self.lmList[point][2]

    def fingersUp(self):
        fingers = []
        if len(self.lmList) < 21:
            return [0, 0, 0, 0, 0]
        if (self.calc_angle(self.tipIds[0], self.tipIds[0] - 1, self.tipIds[0] - 2) > 150.0) and \
           (self.calc_angle(self.tipIds[0] - 1, self.tipIds[0] - 2, self.tipIds[0] - 3) > 150.0):
            fingers.append(1)
        else:
            fingers.append(0)
        for idx in range(1, 5):
            if self.lmList[self.tipIds[idx]][2] < self.lmList[self.tipIds[idx] - 2][2]:
                fingers.append(1)
            else:
                fingers.append(0)
        return fingers

    def get_gesture(self):
        gesture = ""
        fingers = self.fingersUp()
        if fingers.count(1) == 3:
            gesture = "Three"
        elif fingers.count(1) == 4:
            gesture = "Four"
        elif fingers.count(1) == 0:
            gesture = "Zero"
        elif fingers.count(1) == 1:
            gesture = "One"
        elif fingers.count(1) == 2:
            if fingers[0] == 1 and fingers[4] == 1:
                gesture = "Six"
            elif fingers[0] == 1 and self.calc_angle(4, 5, 8) > 90:
                gesture = "Eight"
            else:
                gesture = "Two"
        elif fingers.count(1) == 5:
            if len(self.lmList) >= 21 and self.get_dist(self.lmList[4][1:], self.lmList[8][1:]) < 60 and \
               self.get_dist(self.lmList[4][1:], self.lmList[12][1:]) < 60 and \
               self.get_dist(self.lmList[4][1:], self.lmList[16][1:]) < 60 and \
               self.get_dist(self.lmList[4][1:], self.lmList[20][1:]) < 60:
                gesture = "Seven"
            else:
                gesture = "Five"
        return gesture

    def task_control_timeout(self):
        while True:
            time.sleep(0.1)
            if self.count > 0:
                self.count -= 1
