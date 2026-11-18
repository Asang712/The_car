#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import threading
import time
import math
# ROS2 messages
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
# ROS2 core
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
# TF2
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
# CV bridge
from cv_bridge import CvBridge
from robot_util.robot_vision import SetLAB
ENCODING = ['16UC1', '32FC1']
class VolumeCalculatorNode(Node):
    def __init__(self, name):
        super().__init__(name)

        self.declare_parameter("camera_frame_id", "ascamera_hp60c_camera_link_0")
        self.declare_parameter("depth_measure", [0.0, 0.0, 0.0])
        self.declare_parameter("blue_lab", [30, 80, 0, 220, 150, 110])
        self.declare_parameter("green_lab", [30, 0, 0, 255, 120, 140])
        self.declare_parameter("red_lab", [40, 150, 100, 255, 255, 180])
        self.declare_parameter("yellow_lab", [100, 100, 150, 255, 160, 255])
        self.declare_parameter("min_block_area", 500)

        self.depth_measure = list(self.get_parameter("depth_measure")
                                   .get_parameter_value().double_array_value)
        self.camera_frame_id = self.get_parameter("camera_frame_id") \
            .get_parameter_value().string_value

        # LAB颜色范围
        self.blue_lab = list(self.get_parameter("blue_lab")
                             .get_parameter_value().integer_array_value)
        self.green_lab = list(self.get_parameter("green_lab")
                              .get_parameter_value().integer_array_value)
        self.red_lab = list(self.get_parameter("red_lab")
                            .get_parameter_value().integer_array_value)
        self.yellow_lab = list(self.get_parameter("yellow_lab")
                               .get_parameter_value().integer_array_value)
        self.min_block_area = self.get_parameter("min_block_area") \
            .get_parameter_value().integer_value

        self.color_lab_map = {
            'red': self.red_lab,
            'green': self.green_lab,
            'blue': self.blue_lab,
            'yellow': self.yellow_lab,
        }

        # ========== 初始化状态变量 ==========
        self.rgb_bridge = CvBridge()
        self.depth_bridge = CvBridge()
        self.camera_K = None
        self.scale_x = 1.0
        self.scale_y = 1.0

        # 测量模式状态
        self._measuring = False          # 是否已框选目标
        self._target_lab = None          # 框选提取的LAB范围
        self._roi_start = (0, 0)
        self._roi_end = (0, 0)
        self._mouse_down = False
        self._measured_blocks = []       # 测量结果列表
        self._measure_frame = None       # 暂存当前帧

        # ========== LAB标定模式状态 ==========
        self._calib_active = False
        self._calib_color = None
        self._calib_mouse_down = False
        self._calib_roi_start = (0, 0)
        self._calib_roi_end = (0, 0)
        self._calib_lab_range = None
        self._calib_saved_flag = False
        self._calib_saved_time = 0.0
        self._calib_latest_frame = None

        # ========== TF2 ==========
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ========== 相机内参订阅 ==========
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/ascamera_hp60c/camera_publisher/rgb0/camera_info',
            self.camera_info_callback,
            10
        )

        # ========== RGB图像订阅 ==========
        self.rgb_sub = self.create_subscription(
            Image, '/ascamera_hp60c/camera_publisher/rgb0/image',
            self._rgb_callback, 10
        )

        # ========== 深度图像缓存 ==========
        self._depth_lock = threading.Lock()
        self._latest_depth = None
        self.depth_sub = self.create_subscription(
            Image, '/ascamera_hp60c/camera_publisher/depth0/image_raw',
            self._depth_callback, 10
        )

        # ========== 显示窗口 ==========
        try:
            cv2.namedWindow("result_image", cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback("result_image", self._mouse_callback)
        except Exception as e:
            self.get_logger().warn(f"Failed to create display window: {e}")

        self.get_logger().info("VolumeCalculatorNode initialized. "
                               "Drag ROI on block to measure, 'c'=calib, 'r'=reset, ESC=exit")

    # =================================================================
    #  相机内参
    # =================================================================

    def camera_info_callback(self, msg):
        if self.camera_K is None:
            self.camera_K = np.array(msg.k).reshape(3, 3)
            fx = self.camera_K[0, 0]
            fy = self.camera_K[1, 1]
            cx = self.camera_K[0, 2]
            cy = self.camera_K[1, 2]
            self.scale_x = 640.0 / msg.width
            self.scale_y = 480.0 / msg.height
            self.get_logger().info(
                f"Camera intrinsics: fx={fx:.2f}, fy={fy:.2f}, "
                f"cx={cx:.2f}, cy={cy:.2f}"
            )

    # =================================================================
    #  深度缓存
    # =================================================================

    def _depth_callback(self, depth_msg):
        try:
            depth_img = self.depth_bridge.imgmsg_to_cv2(depth_msg, ENCODING[1])
            depth_img = cv2.resize(depth_img, (640, 480)).astype(np.float32)
            with self._depth_lock:
                self._latest_depth = depth_img
        except Exception as e:
            self.get_logger().warn(f"Depth conversion failed: {e}")

    # =================================================================
    #  LAB颜色掩码
    # =================================================================

    @staticmethod
    def _create_lab_mask(lab_img, l_min, a_min, b_min, l_max, a_max, b_max):
        l_min_c = max(0, min(l_min, 255))
        l_max_c = max(0, min(l_max, 255))
        a_min_c = max(0, min(a_min, 255))
        a_max_c = max(0, min(a_max, 255))
        b_min_c = max(0, min(b_min, 255))
        b_max_c = max(0, min(b_max, 255))
        lower = np.array([l_min_c, a_min_c, b_min_c], dtype=np.uint8)
        upper = np.array([l_max_c, a_max_c, b_max_c], dtype=np.uint8)
        return cv2.inRange(lab_img, lower, upper)

    # =================================================================
    #  TF2：像素→measure坐标系
    # =================================================================

    def _pixel_to_measure(self, pixel_x, pixel_y, depth_image):
        if self.camera_K is None:
            return None
        h, w = depth_image.shape
        px, py = int(round(pixel_x)), int(round(pixel_y))
        if px < 0 or px >= w or py < 0 or py >= h:
            return None

        depth_z = depth_image[py, px] / 1000.0
        if depth_z <= 0.01:
            return None

        fx = self.camera_K[0, 0]
        fy = self.camera_K[1, 1]
        cx = self.camera_K[0, 2]
        cy = self.camera_K[1, 2]

        cam_x = (pixel_x - cx) * depth_z / fx
        cam_y = (pixel_y - cy) * depth_z / fy
        cam_z = depth_z

        point_cam = PointStamped()
        point_cam.header.stamp = rclpy.time.Time(seconds=0).to_msg()
        point_cam.header.frame_id = self.camera_frame_id
        point_cam.point.x = float(cam_x)
        point_cam.point.y = float(cam_y)
        point_cam.point.z = float(cam_z)

        try:
            for attempt in range(5):
                try:
                    point_measure = self.tf_buffer.transform(
                        point_cam, 'measure',
                        timeout=Duration(seconds=1.0)
                    )
                    mx = point_measure.point.x + self.depth_measure[0]
                    my = point_measure.point.y + self.depth_measure[1]
                    mz = point_measure.point.z + self.depth_measure[2]
                    return [mx, my, mz]
                except Exception:
                    if attempt < 4:
                        time.sleep(0.05)
                        continue
                    raise
        except Exception as e:
            self.get_logger().warn(f"TF to 'measure' failed: {e}")
            return None

    # =================================================================
    #  形状分类
    # =================================================================

    @staticmethod
    def _classify_shape(contour):
        """对轮廓进行形状分类

        Returns:
            (shape_name, shape_params_dict) 或 ('unknown', None)
        """
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 0 or area <= 0:
            return 'unknown', None

        circularity = 4.0 * math.pi * area / (perimeter * perimeter)

        # 圆柱体：圆形度高
        if circularity > 0.78:
            (cx, cy), radius_px = cv2.minEnclosingCircle(contour)
            return 'cylinder', {
                'radius_px': radius_px,
                'area_px': area,
                'circularity': circularity,
            }
        else:
            # 正方体/长方体：矩形
            rect = cv2.minAreaRect(contour)
            (cx, cy), (rw, rh), angle = rect
            if min(rw, rh) <= 0:
                return 'unknown', None
            aspect = max(rw, rh) / min(rw, rh)
            if 0.75 <= aspect <= 1.35:
                shape = 'cube'
            else:
                shape = 'cuboid'
            return shape, {
                'width_px': min(rw, rh),
                'length_px': max(rw, rh),
                'area_px': area,
                'aspect_ratio': aspect,
            }

    # =================================================================
    #  体积计算
    # =================================================================

    def _calculate_volume(self, contour, depth_image):
        """根据轮廓和深度图计算木块体积

        Returns:
            dict with shape, area_m2, height_m, volume_m3 或 None
        """
        shape_name, params = self._classify_shape(contour)
        if shape_name == 'unknown' or params is None:
            return None

        # 块中心像素 → measure坐标（用于获取比例尺和顶部高度）
        M = cv2.moments(contour)
        if M['m00'] <= 0:
            return None
        cx = M['m10'] / M['m00']
        cy = M['m01'] / M['m00']

        center_pose = self._pixel_to_measure(cx, cy, depth_image)
        if center_pose is None:
            return None
        center_z = center_pose[2]

        # 深度比例尺（每像素对应多少米，在中心深度处近似）
        fx = self.camera_K[0, 0]
        fy = self.camera_K[1, 1]
        # 使用中心点在相机坐标系下的深度
        depth_at_center = depth_image[int(round(cy)), int(round(cx))] / 1000.0
        scale = depth_at_center / ((fx + fy) / 2.0)

        # 计算实际面积
        if shape_name == 'cylinder':
            real_radius = params['radius_px'] * scale
            real_area = math.pi * real_radius * real_radius
        elif shape_name in ('cube', 'cuboid'):
            real_w = params['width_px'] * scale
            real_l = params['length_px'] * scale
            real_area = real_w * real_l
        else:
            return None

        # 高度：块边缘（桌面）z与块中心（顶面）z的差值
        # 沿轮廓边界采样若干点作为桌面深度
        cnt = contour.squeeze()
        if cnt.ndim != 2 or cnt.shape[0] < 3:
            return None
        n_samples = min(12, cnt.shape[0])
        step = max(1, cnt.shape[0] // n_samples)
        edge_z_vals = []
        for i in range(0, cnt.shape[0], step):
            px, py = cnt[i]
            # 向外偏移5px，更可靠地落在桌面上
            dx = px - cx
            dy = py - cy
            dist = math.sqrt(dx * dx + dy * dy)
            if dist > 0:
                px_out = int(round(px + dx / dist * 6))
                py_out = int(round(py + dy / dist * 6))
            else:
                px_out, py_out = int(round(px)), int(round(py))
            edge_pose = self._pixel_to_measure(px_out, py_out, depth_image)
            if edge_pose is not None:
                edge_z_vals.append(edge_pose[2])

        if len(edge_z_vals) < 3:
            # 回退：无法获取足够边缘点，无法计算高度
            return None

        table_z = float(np.median(edge_z_vals))
        height_m = abs(center_z - table_z)

        # 避免异常小的"高度"（可能是浮点噪音或桌面检测不准）
        if height_m < 0.002:
            return None

        volume_m3 = real_area * height_m

        return {
            'shape': shape_name,
            'center_px': (int(round(cx)), int(round(cy))),
            'area_m2': round(real_area, 6),
            'height_m': round(height_m, 4),
            'height_mm': round(height_m * 1000.0, 1),
            'volume_m3': round(volume_m3, 9),
            'volume_cm3': round(volume_m3 * 1e6, 2),
            'center_pose': center_pose,
        }

    # =================================================================
    #  鼠标回调（主窗口：框选ROI进行测量）
    # =================================================================

    def _mouse_callback(self, event, x, y, flags, param):
        if self._calib_active:
            return
        if event == cv2.EVENT_LBUTTONDOWN:
            self._mouse_down = True
            self._roi_start = (x, y)
            self._roi_end = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self._mouse_down:
            self._roi_end = (x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            self._mouse_down = False
            self._roi_end = (x, y)
            self._extract_roi_and_measure()

    def _extract_roi_and_measure(self):
        """从框选的ROI提取颜色LAB范围，然后检测并测量木块"""
        x1, y1 = self._roi_start
        x2, y2 = self._roi_end
        if x1 > x2:
            x1, x2 = x2, x1
        if y1 > y2:
            y1, y2 = y2, y1

        if x2 - x1 < 5 or y2 - y1 < 5:
            self.get_logger().warn("ROI too small")
            return

        if self._measure_frame is None:
            return

        roi = self._measure_frame[y1:y2, x1:x2]
        lab_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)

        l_min = max(0, int(np.min(lab_roi[:, :, 0])) - 10)
        a_min = max(0, int(np.min(lab_roi[:, :, 1])) - 10)
        b_min = max(0, int(np.min(lab_roi[:, :, 2])) - 10)
        l_max = min(255, int(np.max(lab_roi[:, :, 0])) + 10)
        a_max = min(255, int(np.max(lab_roi[:, :, 1])) + 10)
        b_max = min(255, int(np.max(lab_roi[:, :, 2])) + 10)

        self._target_lab = [l_min, a_min, b_min, l_max, a_max, b_max]
        self._measuring = True

        self.get_logger().info(
            f"ROI extracted LAB: L:[{l_min},{l_max}] "
            f"A:[{a_min},{a_max}] B:[{b_min},{b_max}]"
        )

    # =================================================================
    #  检测与测量
    # =================================================================

    def _detect_and_measure(self, rgb_image, depth_image):
        """根据目标LAB检测木块并计算体积"""
        lab = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2LAB)
        mask = self._create_lab_mask(
            lab,
            self._target_lab[0], self._target_lab[1], self._target_lab[2],
            self._target_lab[3], self._target_lab[4], self._target_lab[5],
        )

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        results = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_block_area:
                continue

            hull = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)
            if hull_area < self.min_block_area:
                continue

            vol = self._calculate_volume(hull, depth_image)
            if vol is not None:
                vol['contour'] = hull
                results.append(vol)

        return results, mask

    # =================================================================
    #  RGB回调
    # =================================================================

    def _rgb_callback(self, color_frame):
        if self.camera_K is None:
            return

        with self._depth_lock:
            depth_image = self._latest_depth
        if depth_image is None:
            depth_image = np.zeros((480, 640), dtype=np.float32)

        rgb_image = self.rgb_bridge.imgmsg_to_cv2(color_frame, 'rgb8')
        result_image = cv2.resize(rgb_image, (640, 480))

        key_raw = cv2.waitKeyEx(1)
        key = key_raw & 0xFF if key_raw != -1 else -1

        # ---------- 标定模式 ----------
        if self._calib_active:
            self._calib_latest_frame = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
            self._render_calib_ui(key)
            if key == 27:
                self._exit_calib_mode()
            return

        # 'c' → 进入LAB标定
        if key == ord('c'):
            self._calib_latest_frame = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
            self._enter_calib_mode()
            return

        # 'r' → 重置测量
        if key == ord('r'):
            self._measuring = False
            self._target_lab = None
            self._measured_blocks = []
            self._roi_start = (0, 0)
            self._roi_end = (0, 0)
            self.get_logger().info("Measurement reset")

        # ESC → 退出
        if key == 27:
            self.get_logger().info("ESC pressed, shutting down...")
            raise KeyboardInterrupt

        # 暂存当前BGR帧供ROI提取使用
        self._measure_frame = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)

        # ---------- 测量模式 ----------
        if self._measuring and self._target_lab is not None:
            self._measured_blocks, mask = self._detect_and_measure(
                result_image, depth_image
            )
        else:
            self._measured_blocks = []
            mask = np.zeros((480, 640), dtype=np.uint8)

        # ---------- 绘制 ----------
        display = self._draw_results(result_image, mask)
        display_bgr = cv2.cvtColor(display, cv2.COLOR_RGB2BGR)
        cv2.imshow("result_image", display_bgr)

    # =================================================================
    #  绘制
    # =================================================================

    def _draw_results(self, result_image, mask):
        display = result_image.copy()

        # 顶部提示
        if not self._measuring:
            cv2.putText(display, "Drag ROI on a block to measure | 'c'=calib | 'r'=reset",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

        # 绘制框选中的ROI
        if self._mouse_down or self._roi_start != self._roi_end:
            cv2.rectangle(display, self._roi_start, self._roi_end,
                          (0, 255, 0), 2)

        # 绘制测量结果
        y_offset = 55
        for block in self._measured_blocks:
            cnt = block['contour']
            shape = block['shape']

            # 颜色编码形状
            if shape == 'cube':
                color = (0, 255, 0)       # 绿色
                shape_cn = "正方体"
            elif shape == 'cuboid':
                color = (255, 200, 0)     # 橙色
                shape_cn = "长方体"
            elif shape == 'cylinder':
                color = (255, 0, 255)     # 紫色
                shape_cn = "圆柱体"
            else:
                color = (128, 128, 128)
                shape_cn = "?" 

            # 绘制轮廓
            cv2.drawContours(display, [cnt], -1, color, 2)

            # 中心点
            cpx, cpy = block['center_px']
            cv2.circle(display, (cpx, cpy), 4, color, -1)

            # 信息文本
            info = (
                f"{shape_cn} | "
                f"Area:{block['area_m2']:.4f}m2 | "
                f"H:{block['height_mm']:.1f}mm | "
                f"Vol:{block['volume_cm3']:.1f}cm3"
            )
            cv2.putText(display, info, (10, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
            y_offset += 20

        if self._measuring and len(self._measured_blocks) == 0:
            cv2.putText(display, "No block detected. Try re-selecting ROI.",
                        (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # 并排显示LAB掩码
        if mask is not None and self._measuring:
            mask_rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
            cv2.putText(mask_rgb, "LAB Mask", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            display = np.hstack((display, mask_rgb))

        return display

    # =================================================================
    #  LAB标定模式
    # =================================================================

    @staticmethod
    def _calib_trackbar_noop(x):
        pass

    def _calib_read_trackbars(self):
        return {
            'LMin': cv2.getTrackbarPos('LMin', 'calib'),
            'AMin': cv2.getTrackbarPos('AMin', 'calib'),
            'BMin': cv2.getTrackbarPos('BMin', 'calib'),
            'LMax': cv2.getTrackbarPos('LMax', 'calib'),
            'AMax': cv2.getTrackbarPos('AMax', 'calib'),
            'BMax': cv2.getTrackbarPos('BMax', 'calib'),
        }

    def _enter_calib_mode(self):
        self._calib_active = True
        self._calib_color = 'wood'  # 通用标定标签
        self._calib_mouse_down = False
        self._calib_roi_start = (0, 0)
        self._calib_roi_end = (0, 0)
        self._calib_lab_range = None
        self._calib_saved_flag = False
        self._calib_saved_time = 0.0

        try:
            cv2.destroyWindow('result_image')
        except Exception:
            pass

        try:
            cv2.namedWindow('calib', cv2.WINDOW_AUTOSIZE)
            cv2.createTrackbar('LMin', 'calib', 0, 255, self._calib_trackbar_noop)
            cv2.createTrackbar('AMin', 'calib', 0, 255, self._calib_trackbar_noop)
            cv2.createTrackbar('BMin', 'calib', 0, 255, self._calib_trackbar_noop)
            cv2.createTrackbar('LMax', 'calib', 255, 255, self._calib_trackbar_noop)
            cv2.createTrackbar('AMax', 'calib', 255, 255, self._calib_trackbar_noop)
            cv2.createTrackbar('BMax', 'calib', 255, 255, self._calib_trackbar_noop)
            cv2.setMouseCallback('calib', self._calib_mouse_callback)
        except Exception as e:
            self.get_logger().warn(f"Failed to create calib windows: {e}")

        self.get_logger().info(
            "=== LAB Calibration === "
            "Drag ROI on 'calib' window, fine-tune with trackbars, "
            "'s' to save to common.yaml, ESC to exit"
        )

    def _exit_calib_mode(self):
        self._calib_active = False
        self._calib_color = None
        self._calib_mouse_down = False
        self._calib_lab_range = None
        self._calib_saved_flag = False

        try:
            cv2.destroyWindow('calib')
        except Exception:
            pass

        try:
            cv2.namedWindow("result_image", cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback("result_image", self._mouse_callback)
        except Exception:
            pass

        self.get_logger().info("=== Exited LAB Calibration ===")

    def _calib_mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self._calib_mouse_down = True
            self._calib_roi_start = (x, y)
            self._calib_roi_end = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self._calib_mouse_down:
            self._calib_roi_end = (x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            self._calib_mouse_down = False
            self._calib_roi_end = (x, y)
            self._calib_extract_roi_lab()

    def _calib_extract_roi_lab(self):
        x1, y1 = self._calib_roi_start
        x2, y2 = self._calib_roi_end
        if x1 > x2:
            x1, x2 = x2, x1
        if y1 > y2:
            y1, y2 = y2, y1
        if x2 - x1 < 2 or y2 - y1 < 2:
            self.get_logger().warn("ROI too small")
            return
        if self._calib_latest_frame is None:
            self.get_logger().warn("No frame available")
            return

        roi = self._calib_latest_frame[y1:y2, x1:x2]
        lab_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)

        l_min = max(0, int(np.min(lab_roi[:, :, 0])))
        a_min = max(0, int(np.min(lab_roi[:, :, 1])))
        b_min = max(0, int(np.min(lab_roi[:, :, 2])))
        l_max = min(255, int(np.max(lab_roi[:, :, 0])))
        a_max = min(255, int(np.max(lab_roi[:, :, 1])))
        b_max = min(255, int(np.max(lab_roi[:, :, 2])))

        self._calib_lab_range = ((l_min, a_min, b_min), (l_max, a_max, b_max))

        cv2.setTrackbarPos('LMin', 'calib', l_min)
        cv2.setTrackbarPos('AMin', 'calib', a_min)
        cv2.setTrackbarPos('BMin', 'calib', b_min)
        cv2.setTrackbarPos('LMax', 'calib', l_max)
        cv2.setTrackbarPos('AMax', 'calib', a_max)
        cv2.setTrackbarPos('BMax', 'calib', b_max)

        self.get_logger().info(
            f"ROI LAB: L:[{l_min},{l_max}] A:[{a_min},{a_max}] B:[{b_min},{b_max}]"
        )

    def _render_calib_ui(self, key):
        if self._calib_latest_frame is None:
            return

        frame = self._calib_latest_frame.copy()
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        tb = self._calib_read_trackbars()

        mask = self._create_lab_mask(
            lab,
            tb['LMin'], tb['AMin'], tb['BMin'],
            tb['LMax'], tb['AMax'], tb['BMax'],
        )
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask_processed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask_processed = cv2.morphologyEx(mask_processed, cv2.MORPH_OPEN, kernel, iterations=1)

        display = frame.copy()
        if self._calib_mouse_down or self._calib_roi_start != self._calib_roi_end:
            cv2.rectangle(display, self._calib_roi_start, self._calib_roi_end,
                          (0, 255, 0), 2)

        info = (
            f"CALIB | L:[{tb['LMin']},{tb['LMax']}] "
            f"A:[{tb['AMin']},{tb['AMax']}] B:[{tb['BMin']},{tb['BMax']}]"
        )
        cv2.putText(display, info, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(display, "Drag ROI | Trackbars | 's'=save | ESC=exit",
                    (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        if self._calib_saved_flag:
            elapsed = time.time() - self._calib_saved_time
            if elapsed < 2.0 and int(elapsed * 4) % 2 == 0:
                cv2.putText(display, "SAVED!", (200, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 0), 4)
            elif elapsed >= 2.0:
                self._calib_saved_flag = False

        mask_bgr = cv2.cvtColor(mask_processed, cv2.COLOR_GRAY2BGR)
        cv2.putText(mask_bgr, "LAB Mask", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        combined = np.hstack((display, mask_bgr))
        cv2.imshow('calib', combined)

        # 's' 保存到common.yaml（统一保存到 wood 键）
        if key == ord('s'):
            lab_vals = [tb['LMin'], tb['AMin'], tb['BMin'],
                        tb['LMax'], tb['AMax'], tb['BMax']]
            self._calib_lab_range = ((tb['LMin'], tb['AMin'], tb['BMin']),
                                     (tb['LMax'], tb['AMax'], tb['BMax']))
            # 保存到所有颜色键（也可按需调整）
            success = True
            err_msgs = []
            for cn in ['red', 'green', 'blue', 'yellow']:
                ok, err = SetLAB(cn, lab_vals)
                if not ok:
                    success = False
                    err_msgs.append(f"{cn}:{err}")
                else:
                    self.color_lab_map[cn] = lab_vals
            if success:
                self._calib_saved_flag = True
                self._calib_saved_time = time.time()
                self.get_logger().info(f"LAB written to common.yaml: {lab_vals}")
            else:
                self.get_logger().error(f"SetLAB failed: {'; '.join(err_msgs)}")

    def __del__(self):
        cv2.destroyAllWindows()


def main():
    rclpy.init()
    node = VolumeCalculatorNode('VolumeCalculatorNode')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
