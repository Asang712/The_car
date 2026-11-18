#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
colorSelect.py

HSV 颜色取色工具 - 为 microros_car_v2 项目标定颜色
- 支持鼠标框选 ROI 区域自动计算 HSV 范围
- 实时预览检测效果
- 依次标定 4 种颜色：RED、GREEN、BLUE、YELLOW
- 保存到配置文件供 Dify 节点使用
"""

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from .image_conversion import image_to_bgr
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np


def ManyImgs(scale, imgarray):
    """拼接多个图像显示"""
    rows = len(imgarray)
    cols = len(imgarray[0]) if rows > 0 else 0

    if rows == 0 or cols == 0:
        return np.zeros((480, 640, 3), dtype=np.uint8)

    width = imgarray[0][0].shape[1] if len(imgarray[0][0].shape) >= 2 else 640
    height = imgarray[0][0].shape[0] if len(imgarray[0][0].shape) >= 2 else 480

    imgBlank = np.zeros((height, width, 3), np.uint8)
    hor = [imgBlank] * rows

    for y in range(rows):
        if isinstance(imgarray[y], list):
            for x in range(len(imgarray[y])):
                if len(imgarray[y][x].shape) == 2:
                    imgarray[y][x] = cv.cvtColor(imgarray[y][x], cv.COLOR_GRAY2BGR)
            hor[y] = np.hstack(imgarray[y])
        else:
            if len(imgarray[y].shape) == 2:
                imgarray[y] = cv.cvtColor(imgarray[y], cv.COLOR_GRAY2BGR)
            hor[y] = imgarray[y]

    ver = np.vstack(hor)
    return ver


class ColorFollowTool:
    """颜色跟随工具类 - 计算 HSV 范围"""

    def Roi_hsv(self, frame, roi):
        """
        从 ROI 区域计算 HSV 范围
        roi: (x1, y1, x2, y2)
        """
        x1, y1, x2, y2 = roi
        x1, x2 = max(0, min(x1, x2)), min(frame.shape[1], max(x1, x2))
        y1, y2 = max(0, min(y1, y2)), min(frame.shape[0], max(y1, y2))

        if x2 - x1 < 5 or y2 - y1 < 5:
            return None, None

        roi_frame = frame[y1:y2, x1:x2]
        hsv_roi = cv.cvtColor(roi_frame, cv.COLOR_BGR2HSV)

        # 计算 HSV 通道的最小值和最大值
        h_min, h_max = int(hsv_roi[:, :, 0].min()), int(hsv_roi[:, :, 0].max())
        s_min, s_max = int(hsv_roi[:, :, 1].min()), int(hsv_roi[:, :, 1].max())
        v_min, v_max = int(hsv_roi[:, :, 2].min()), int(hsv_roi[:, :, 2].max())

        # 扩展范围以提高鲁棒性
        h_min = max(0, h_min - 10)
        h_max = min(180, h_max + 10)
        s_min = max(0, s_min - 40)
        s_max = min(255, s_max + 40)
        v_min = max(0, v_min - 40)
        v_max = min(255, v_max + 40)

        hsv_min = (h_min, s_min, v_min)
        hsv_max = (h_max, s_max, v_max)

        return hsv_min, (hsv_min, hsv_max)


class ColorSelector(Node):
    """HSV 颜色选择器节点"""

    def __init__(self, name):
        super().__init__(name)

        # 图像相关
        self.bridge = CvBridge()
        self.cv_image = None

        # 标定状态
        self.selected_count = 1  # 当前标定的颜色索引 (1-4)
        self.colors = [None] * 4  # 存储 4 种颜色的 HSV 范围
        self.finish_flag = False

        # HSV 配置文件路径
        self.hsv_file = os.path.expanduser(
            "~/microros_ws/src/multi_brains/config/color_hsv.txt"
        )

        # 确保配置目录存在
        config_dir = os.path.dirname(self.hsv_file)
        if not os.path.exists(config_dir):
            os.makedirs(config_dir)
            self.get_logger().info(f"创建配置目录: {config_dir}")

        # 鼠标选择相关
        self.color_tool = ColorFollowTool()
        self.Roi_init = ()
        self.select_flags = False
        self.cols = (0, 0)
        self.rows = (0, 0)
        self.Mouse_XY = (0, 0)
        self.preview_hsv = None

        # 订阅图像话题
        self.declare_parameter("image_topic", "/camera/image_raw")
        img_topic = self.get_parameter("image_topic").value

        # 相机发布端使用 BEST_EFFORT，订阅端必须对齐，否则收不到图像
        camera_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.sub_image = self.create_subscription(
            Image,
            img_topic,
            self.image_callback,
            camera_qos
        )

        # 创建窗口和鼠标回调
        cv.namedWindow('Color Selector')
        cv.setMouseCallback('Color Selector', self.on_mouse)

        self.get_logger().info(f"颜色选择器已启动 | 话题: {img_topic}")
        self.get_logger().info(f"HSV 文件: {self.hsv_file}")
        self.get_logger().info("=" * 60)
        self.get_logger().info("使用说明:")
        self.get_logger().info("1. 鼠标框选目标颜色区域")
        self.get_logger().info("2. 松开鼠标后查看检测预览")
        self.get_logger().info("3. 按 [空格] 保存当前颜色")
        self.get_logger().info("4. 按 [E] 跳过当前颜色")
        self.get_logger().info("5. 按 [Q] 退出程序")
        self.get_logger().info("=" * 60)

    def image_callback(self, msg):
        """图像回调"""
        try:
            frame = image_to_bgr(msg)
            if frame is not None:
                # 统一 resize 到 640x480，使鼠标坐标、绘制、ROI 裁剪处于同一坐标系
                self.cv_image = cv.resize(frame, (640, 480))
        except Exception as e:
            self.get_logger().error(f"图像解码错误: {e}")

    def on_mouse(self, event, x, y, flags, param):
        """鼠标事件回调"""
        if event == cv.EVENT_LBUTTONDOWN:
            self.select_flags = True
            self.Mouse_XY = (x, y)
            self.preview_hsv = None
            self.cols = (x, y)
            self.rows = (x, y)

        elif event == cv.EVENT_MOUSEMOVE:
            if self.select_flags and self.cv_image is not None:
                img_h, img_w = self.cv_image.shape[:2]
                x = max(0, min(x, img_w - 1))
                y = max(0, min(y, img_h - 1))
                self.cols = (min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y))
                self.rows = (max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y))
                self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])

        elif event == cv.EVENT_LBUTTONUP:
            self.select_flags = False
            if self.Roi_init and abs(x - self.Mouse_XY[0]) > 2 and abs(y - self.Mouse_XY[1]) > 2:
                try:
                    full_frame = self.cv_image.copy()
                    roi_frame = full_frame[
                        self.Roi_init[1]:self.Roi_init[3],
                        self.Roi_init[0]:self.Roi_init[2]
                    ]
                    _, fixed_hsv = self.color_tool.Roi_hsv(
                        roi_frame,
                        (0, 0, roi_frame.shape[1], roi_frame.shape[0])
                    )
                    if fixed_hsv:
                        self.preview_hsv = fixed_hsv
                except Exception as e:
                    self.get_logger().error(f"HSV 计算错误: {e}")

    def process_frame(self, action):
        """处理图像帧和键盘事件"""
        if self.cv_image is None:
            return

        frame = self.cv_image.copy()
        frame = cv.resize(frame, (640, 480))
        binary = None

        # 绘制选择框
        if self.select_flags:
            x1, y1 = max(0, self.cols[0]), max(0, self.cols[1])
            x2, y2 = min(frame.shape[1], self.rows[0]), min(frame.shape[0], self.rows[1])
            cv.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # 预览检测效果
        if self.preview_hsv and not self.select_flags:
            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
            hsv_min = np.array(self.preview_hsv[0], dtype=np.uint8)
            hsv_max = np.array(self.preview_hsv[1], dtype=np.uint8)
            binary = cv.inRange(hsv, hsv_min, hsv_max)

            # 显示 HSV 值
            hsv_min_text = f"MIN: HSV({self.preview_hsv[0][0]}, {self.preview_hsv[0][1]}, {self.preview_hsv[0][2]})"
            hsv_max_text = f"MAX: HSV({self.preview_hsv[1][0]}, {self.preview_hsv[1][1]}, {self.preview_hsv[1][2]})"

            (text_w, _), _ = cv.getTextSize(hsv_max_text, cv.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            x_pos = (frame.shape[1] - text_w) // 2

            cv.putText(frame, hsv_max_text, (x_pos, 25),
                      cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
            cv.putText(frame, hsv_min_text, (x_pos, 45),
                      cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        # 显示当前状态
        color_map = {1: "RED", 2: "GREEN", 3: "BLUE", 4: "YELLOW"}
        prefix = color_map.get(self.selected_count, "COMPLETE")
        status_text = f"{prefix} Color {self.selected_count}/4"
        if self.finish_flag:
            status_text = "COMPLETE - All colors saved!"
        cv.putText(frame, status_text, (30, 30),
                  cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # 键盘事件处理
        # 空格键：保存当前颜色
        if action == 32:
            if not self.finish_flag and self.preview_hsv and 1 <= self.selected_count <= 4:
                self.colors[self.selected_count - 1] = self.preview_hsv
                self.save_hsv_to_file()

                self.get_logger().info(
                    f"✓ 颜色 {self.selected_count} ({prefix}) 已保存: "
                    f"MIN={self.preview_hsv[0]}, MAX={self.preview_hsv[1]}"
                )

                if self.selected_count < 4:
                    self.selected_count += 1
                    next_color = color_map.get(self.selected_count, "")
                    self.get_logger().info(f"→ 请标定颜色 {self.selected_count}: {next_color}")
                else:
                    self.finish_flag = True
                    self.get_logger().info("=" * 60)
                    self.get_logger().info("✓ 所有颜色标定完成！")
                    self.get_logger().info(f"HSV 配置已保存到: {self.hsv_file}")
                    self.get_logger().info("=" * 60)

                self.preview_hsv = None
                self.Roi_init = ()

        # E 键：跳过当前颜色
        elif action == ord('e') or action == ord('E'):
            if not self.finish_flag:
                self.selected_count += 1
                self.preview_hsv = None
                if self.selected_count > 4:
                    self.selected_count = 1
                next_color = color_map.get(self.selected_count, "")
                self.get_logger().info(f"→ 跳过，当前颜色: {self.selected_count} ({next_color})")

        # 显示图像
        if binary is not None:
            display_frame = ManyImgs(1, [[frame, cv.cvtColor(binary, cv.COLOR_GRAY2BGR)]])
        else:
            display_frame = frame

        cv.imshow('Color Selector', display_frame)

    def save_hsv_to_file(self):
        """保存 HSV 配置到文件"""
        try:
            # 读取现有文件（如果存在）
            if os.path.exists(self.hsv_file):
                with open(self.hsv_file, 'r') as f:
                    lines = f.readlines()
            else:
                lines = ['\n'] * 4

            # 确保有 4 行
            lines = lines[:4]
            while len(lines) < 4:
                lines.append('\n')

            # 更新当前颜色的 HSV 值
            if self.colors[self.selected_count - 1] is not None:
                hsv_min, hsv_max = self.colors[self.selected_count - 1]
                new_line = f"{hsv_min[0]}, {hsv_min[1]}, {hsv_min[2]}, {hsv_max[0]}, {hsv_max[1]}, {hsv_max[2]}\n"
                lines[self.selected_count - 1] = new_line

            # 写入文件
            with open(self.hsv_file, 'w') as f:
                f.writelines(lines)

        except Exception as e:
            self.get_logger().error(f"保存 HSV 文件失败: {e}")


def main(args=None):
    rclpy.init(args=args)
    selector = ColorSelector("color_selector")

    try:
        while rclpy.ok():
            rclpy.spin_once(selector, timeout_sec=0.1)
            action = cv.waitKey(10) & 0xFF
            selector.process_frame(action)
            if action == ord('q'):
                selector.get_logger().info("退出颜色选择器")
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv.destroyAllWindows()
        selector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
