#!/usr/bin/env python3
# encoding: utf-8

import os
os.environ.setdefault("MEDIAPIPE_DISABLE_TF", "1")
os.environ.setdefault("MEDIAPIPE_DISABLE_GPU", "1")

import math

import cv2 as cv
import mediapipe as mp


def get_dist(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return abs(math.sqrt(math.pow(abs(y1 - y2), 2) + math.pow(abs(x1 - x2), 2)))


def calc_angle(pt1, pt2, pt3):
    a = get_dist(pt1, pt2)
    b = get_dist(pt2, pt3)
    c = get_dist(pt1, pt3)
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
    return angle


def classify_hand_gesture(fingers, lm_list):
    """Classify 13 gestures from finger states and MediaPipe landmarks."""
    if len(fingers) != 5 or len(lm_list) < 21:
        return "Unknown"

    fingers = [1 if int(value) != 0 else 0 for value in fingers]
    finger_count = sum(fingers)

    def landmark_distance(first_index, second_index):
        return get_dist(
            lm_list[first_index][1:3],
            lm_list[second_index][1:3],
        )

    palm_size = max(
        landmark_distance(0, 9),
        landmark_distance(5, 17),
        1.0,
    )
    thumb_index_distance = landmark_distance(4, 8) / palm_size
    thumb_middle_distance = landmark_distance(4, 12) / palm_size
    index_middle_distance = landmark_distance(8, 12) / palm_size

    # Shape-specific gestures must be checked before generic finger counts.
    if fingers[2:] == [1, 1, 1] and thumb_index_distance < 0.38:
        return "OK"
    if (
        fingers[1:3] == [1, 1]
        and fingers[3:] == [0, 0]
        and thumb_index_distance < 0.48
        and thumb_middle_distance < 0.55
    ):
        return "Seven"

    if fingers == [1, 0, 0, 0, 0]:
        thumb_tip_y = float(lm_list[4][2])
        thumb_mcp_y = float(lm_list[2][2])
        vertical_offset = 0.20 * palm_size
        if thumb_tip_y < thumb_mcp_y - vertical_offset:
            return "Thumb_up"
        if thumb_tip_y > thumb_mcp_y + vertical_offset:
            return "Thumb_down"

    if finger_count == 0:
        return "Zero"
    if finger_count == 1:
        return "One"
    if finger_count == 2:
        if fingers == [1, 0, 0, 0, 1]:
            return "Six"
        if fingers == [1, 1, 0, 0, 0]:
            return "Eight"
        if fingers == [0, 1, 1, 0, 0] and index_middle_distance > 0.55:
            return "Yes"
        return "Two"
    if finger_count == 3:
        return "Three"
    if finger_count == 4:
        return "Four"
    if finger_count == 5:
        return "Five"
    return "Unknown"


class HandDetector:
    def __init__(self, mode=False, maxHands=2, detectorCon=0.5, trackCon=0.5):
        self.mpHand = mp.solutions.hands
        self.draw = False
        self.mpDraw = mp.solutions.drawing_utils
        self.hands = self.mpHand.Hands(
            static_image_mode=mode,
            max_num_hands=maxHands,
            min_detection_confidence=detectorCon,
            min_tracking_confidence=trackCon,
        )
        self.cxList = []
        self.cyList = []
        self.results = None

    def close(self):
        try:
            self.hands.close()
        except Exception:
            pass

    def findHands(self, frame):
        lmList = []
        self.cxList = []
        self.cyList = []
        bbox = 0, 0, 0, 0
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.hands.process(img_RGB)
        if self.results.multi_hand_landmarks:
            for hand_landmarks in self.results.multi_hand_landmarks:
                if not self.draw:
                    self.mpDraw.draw_landmarks(frame, hand_landmarks, self.mpHand.HAND_CONNECTIONS)
                for idx, lm in enumerate(hand_landmarks.landmark):
                    h, w, _ = frame.shape
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    lmList.append([idx, cx, cy])
                    self.cxList.append(cx)
                    self.cyList.append(cy)
        if self.cxList and self.cyList:
            xmin, xmax = min(self.cxList), max(self.cxList)
            ymin, ymax = min(self.cyList), max(self.cyList)
            bbox = xmin, ymin, xmax, ymax
            if self.draw:
                cv.rectangle(frame, (xmin - 20, ymin - 20), (xmax + 20, ymax + 20), (0, 255, 0), 2)
        return frame, lmList, bbox

    def fingersUp(self, lmList):
        if len(lmList) < 21:
            return [0, 0, 0, 0, 0]
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
        for idx in range(1, 5):
            if lmList[tipIds[idx]][2] < lmList[tipIds[idx] - 2][2]:
                fingers.append(1)
            else:
                fingers.append(0)
        return fingers

    def ThumbTOforefinger(self, lmList):
        if len(lmList) < 9:
            return 0.0
        point1 = lmList[4][1:3]
        point2 = lmList[0][1:3]
        point3 = lmList[8][1:3]
        return abs(calc_angle(point1, point2, point3))

    def get_gesture(self, lmList):
        fingers = self.fingersUp(lmList)
        return classify_hand_gesture(fingers, lmList)


class PoseDetector:
    def __init__(self, mode=False, smooth=True, detectionCon=0.5, trackCon=0.5):
        self.mpPose = mp.solutions.pose
        self.mpDraw = mp.solutions.drawing_utils
        self.pose = self.mpPose.Pose(
            static_image_mode=mode,
            smooth_landmarks=smooth,
            min_detection_confidence=detectionCon,
            min_tracking_confidence=trackCon,
        )
        self.lmDrawSpec = self.mpDraw.DrawingSpec(color=(0, 0, 255), thickness=-1, circle_radius=6)
        self.drawSpec = self.mpDraw.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2)

    def close(self):
        try:
            self.pose.close()
        except Exception:
            pass

    def pubPosePoint(self, frame, draw=True):
        pointArray = []
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.pose.process(img_RGB)
        if self.results.pose_landmarks:
            if draw:
                self.mpDraw.draw_landmarks(frame, self.results.pose_landmarks, self.mpPose.POSE_CONNECTIONS, self.lmDrawSpec, self.drawSpec)
            for idx, lm in enumerate(self.results.pose_landmarks.landmark):
                h, w, _ = frame.shape
                pointArray.append([idx, lm.x * w, lm.y * h, lm.z])
        return frame, pointArray


class Holistic:
    def __init__(self, staticMode=False, landmarks=True, detectionCon=0.5, trackingCon=0.5):
        self.mpHolistic = mp.solutions.holistic
        self.mpFaceMesh = mp.solutions.face_mesh
        self.mpHands = mp.solutions.hands
        self.mpPose = mp.solutions.pose
        self.mpDraw = mp.solutions.drawing_utils
        self.mpholistic = self.mpHolistic.Holistic(
            static_image_mode=staticMode,
            smooth_landmarks=landmarks,
            min_detection_confidence=detectionCon,
            min_tracking_confidence=trackingCon,
        )
        self.lmDrawSpec = self.mpDraw.DrawingSpec(color=(0, 0, 255), thickness=-1, circle_radius=3)
        self.drawSpec = self.mpDraw.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2)

    def close(self):
        try:
            self.mpholistic.close()
        except Exception:
            pass

    def findHolistic(self, frame, draw=True):
        poseptArray = []
        lhandptArray = []
        rhandptArray = []
        h, w, _ = frame.shape
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.mpholistic.process(img_RGB)
        if self.results.pose_landmarks:
            if draw:
                self.mpDraw.draw_landmarks(frame, self.results.pose_landmarks, self.mpPose.POSE_CONNECTIONS, self.lmDrawSpec, self.drawSpec)
            for idx, lm in enumerate(self.results.pose_landmarks.landmark):
                poseptArray.append([idx, lm.x * w, lm.y * h, lm.z])
        if self.results.left_hand_landmarks:
            if draw:
                self.mpDraw.draw_landmarks(frame, self.results.left_hand_landmarks, self.mpHands.HAND_CONNECTIONS, self.lmDrawSpec, self.drawSpec)
            for idx, lm in enumerate(self.results.left_hand_landmarks.landmark):
                lhandptArray.append([idx, lm.x * w, lm.y * h, lm.z])
        if self.results.right_hand_landmarks:
            if draw:
                self.mpDraw.draw_landmarks(frame, self.results.right_hand_landmarks, self.mpHands.HAND_CONNECTIONS, self.lmDrawSpec, self.drawSpec)
            for idx, lm in enumerate(self.results.right_hand_landmarks.landmark):
                rhandptArray.append([idx, lm.x * w, lm.y * h, lm.z])
        return frame, poseptArray, lhandptArray, rhandptArray


class FaceMesh:
    def __init__(self, staticMode=False, maxFaces=1, minDetectionCon=0.5, minTrackingCon=0.5):
        self.mpDraw = mp.solutions.drawing_utils
        self.mpFaceMesh = mp.solutions.face_mesh
        self.faceMesh = self.mpFaceMesh.FaceMesh(
            static_image_mode=staticMode,
            max_num_faces=maxFaces,
            min_detection_confidence=minDetectionCon,
            min_tracking_confidence=minTrackingCon,
        )
        self.lmDrawSpec = self.mpDraw.DrawingSpec(color=(0, 0, 255), thickness=-1, circle_radius=3)
        self.drawSpec = self.mpDraw.DrawingSpec(color=(0, 255, 0), thickness=1, circle_radius=1)

    def close(self):
        try:
            self.faceMesh.close()
        except Exception:
            pass

    def pubFaceMeshPoint(self, frame, draw=True):
        pointArray = []
        h, w, _ = frame.shape
        imgRGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.faceMesh.process(imgRGB)
        if self.results.multi_face_landmarks:
            for face_landmarks in self.results.multi_face_landmarks:
                connections = getattr(self.mpFaceMesh, "FACEMESH_CONTOURS", None)
                if draw:
                    self.mpDraw.draw_landmarks(frame, face_landmarks, connections, self.lmDrawSpec, self.drawSpec)
                for idx, lm in enumerate(face_landmarks.landmark):
                    pointArray.append([idx, lm.x * w, lm.y * h, lm.z])
        return frame, pointArray
