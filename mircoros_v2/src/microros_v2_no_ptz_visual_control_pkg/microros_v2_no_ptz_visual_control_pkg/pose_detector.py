#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pose_detector.py

MediaPipe 人体姿态检测器，从 microros_v2_mediapipe_pkg/media_common.py 提取，
仅保留 pose_tracker 所需的 PoseDetector。

运行前准备:
    source ~/workspaces/MicroROS-V2-ROS2-SRC/start_yolo_ros2_env.sh
"""

import os
os.environ.setdefault("MEDIAPIPE_DISABLE_TF", "1")
os.environ.setdefault("MEDIAPIPE_DISABLE_GPU", "1")

import cv2 as cv
import mediapipe as mp


class PoseDetector:
    """MediaPipe 人体姿态检测器 / MediaPipe pose detector."""

    def __init__(self, mode=False, smooth=True, detectionCon=0.5, trackCon=0.5):
        self.mpPose = mp.solutions.pose
        self.mpDraw = mp.solutions.drawing_utils
        self.pose = self.mpPose.Pose(
            static_image_mode=mode,
            smooth_landmarks=smooth,
            min_detection_confidence=detectionCon,
            min_tracking_confidence=trackCon,
        )
        self.results = None
        self.lmDrawSpec = self.mpDraw.DrawingSpec(color=(0, 0, 255), thickness=-1, circle_radius=6)
        self.drawSpec = self.mpDraw.DrawingSpec(color=(0, 255, 0), thickness=5, circle_radius=2)

    def close(self):
        try:
            self.pose.close()
        except Exception:
            pass

    def pubPosePoint(self, frame, draw=True):
        """检测人体姿态关键点 / Detect pose landmarks.

        Returns:
            frame: 绘制后的图像 / Frame with drawings.
            pointArray: [[idx, cx, cy, z], ...] 关键点列表 / Landmark list.
            bbox_area: 外接框面积 / Bounding box area.
        """
        pointArray = []
        bbox_area = 0
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.pose.process(img_RGB)
        if self.results.pose_landmarks:
            x_coords = []
            y_coords = []
            if draw:
                self.mpDraw.draw_landmarks(
                    frame, self.results.pose_landmarks,
                    self.mpPose.POSE_CONNECTIONS, self.lmDrawSpec, self.drawSpec,
                )
            for idx, lm in enumerate(self.results.pose_landmarks.landmark):
                h, w, _ = frame.shape
                cx, cy = lm.x * w, lm.y * h
                pointArray.append([idx, cx, cy, lm.z])
                x_coords.append(cx)
                y_coords.append(cy)
            if x_coords and y_coords:
                bbox_width = max(x_coords) - min(x_coords)
                bbox_height = max(y_coords) - min(y_coords)
                bbox_area = bbox_width * bbox_height // 100
        return frame, pointArray, bbox_area
