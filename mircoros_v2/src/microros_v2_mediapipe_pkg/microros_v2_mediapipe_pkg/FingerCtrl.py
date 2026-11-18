#!/usr/bin/env python3
# encoding: utf-8

import os

# 在导入 mediapipe 前设置，适合 Ubuntu 虚拟机 CPU-only 环境
# 这里的 TF 是 TensorFlow，不是 ROS 的 tf/tf2
os.environ.setdefault("MEDIAPIPE_DISABLE_TF", "1")
os.environ.setdefault("MEDIAPIPE_DISABLE_GPU", "1")

import math
import cv2 as cv
import numpy as np
import mediapipe as mp
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image


def to_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in ("true", "1", "yes", "on")
    return bool(value)


def to_int(value, default=0):
    try:
        return int(value)
    except Exception:
        return default


def to_float(value, default=0.0):
    try:
        return float(value)
    except Exception:
        return default


class FingerCtrl(Node):
    def __init__(self):
        super().__init__("finger_ctrl_node")

        # =========================
        # 基础参数
        # =========================
        self.declare_parameter("rgb_topic", "/camera/image_raw")
        self.declare_parameter("static_image_mode", False)

        # 推荐默认只识别 1 只手，避免多手干扰导致百分比跳动
        self.declare_parameter("max_hands", 1)
        self.declare_parameter("detector_con", 0.7)
        self.declare_parameter("track_con", 0.7)

        # =========================
        # 显示窗口参数
        # 左右拼接画面默认约为 1280x480
        # keep_aspect_ratio=True 时会等比例缩放并自动补黑边，保证画面完整不裁剪
        # =========================
        self.declare_parameter("window_name", "finger_ctrl")
        self.declare_parameter("display_width", 1280)
        self.declare_parameter("display_height", 480)
        self.declare_parameter("resize_display", True)
        self.declare_parameter("keep_aspect_ratio", True)

        # =========================
        # 控制量优化参数
        # 使用距离比例替代角度直接映射，闭合状态更稳定
        # =========================
        self.declare_parameter("use_distance_control", True)
        self.declare_parameter("control_deadzone_percent", 10.0)
        self.declare_parameter("zero_snap_percent", 12.0)
        self.declare_parameter("smooth_alpha", 0.25)
        self.declare_parameter("min_dist_ratio", 0.25)
        self.declare_parameter("max_dist_ratio", 1.20)

        self.rgb_topic = str(self.get_parameter("rgb_topic").value)
        static_image_mode = to_bool(self.get_parameter("static_image_mode").value)
        max_hands = to_int(self.get_parameter("max_hands").value, 1)
        detector_con = to_float(self.get_parameter("detector_con").value, 0.7)
        track_con = to_float(self.get_parameter("track_con").value, 0.7)

        self.window_name = str(self.get_parameter("window_name").value)
        self.display_width = to_int(self.get_parameter("display_width").value, 1280)
        self.display_height = to_int(self.get_parameter("display_height").value, 480)
        self.resize_display = to_bool(self.get_parameter("resize_display").value)
        self.keep_aspect_ratio = to_bool(self.get_parameter("keep_aspect_ratio").value)

        self.use_distance_control = to_bool(self.get_parameter("use_distance_control").value)
        self.control_deadzone_percent = to_float(
            self.get_parameter("control_deadzone_percent").value,
            10.0,
        )
        self.zero_snap_percent = to_float(
            self.get_parameter("zero_snap_percent").value,
            12.0,
        )
        self.smooth_alpha = to_float(
            self.get_parameter("smooth_alpha").value,
            0.25,
        )
        self.min_dist_ratio = to_float(
            self.get_parameter("min_dist_ratio").value,
            0.25,
        )
        self.max_dist_ratio = to_float(
            self.get_parameter("max_dist_ratio").value,
            1.20,
        )

        # =========================
        # OpenCV 窗口设置
        # WINDOW_NORMAL 表示窗口可拖拽调整大小
        # =========================
        cv.namedWindow(self.window_name, cv.WINDOW_NORMAL)
        if self.display_width > 0 and self.display_height > 0:
            cv.resizeWindow(self.window_name, self.display_width, self.display_height)

        # =========================
        # QoS 设置
        # 摄像头图像建议使用 BEST_EFFORT，减少图像堆积导致的延迟
        # =========================
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # =========================
        # FingerCtrl 功能变量
        # =========================
        self.effect = ["color", "thresh", "blur", "hue", "enhance"]

        self.vol_per = 0.0
        self.filtered_vol_per = 0.0
        self.value = 0.0
        self.index = 0

        # =========================
        # MediaPipe Hands 初始化
        # =========================
        self.mp_hand = mp.solutions.hands
        self.mp_draw = mp.solutions.drawing_utils

        self.hands = self.mp_hand.Hands(
            static_image_mode=static_image_mode,
            max_num_hands=max_hands,
            min_detection_confidence=detector_con,
            min_tracking_confidence=track_con,
        )

        self.lm_draw_spec = self.mp_draw.DrawingSpec(
            color=(0, 0, 255),
            thickness=-1,
            circle_radius=15,
        )

        self.draw_spec = self.mp_draw.DrawingSpec(
            color=(0, 255, 0),
            thickness=10,
            circle_radius=10,
        )

        self.lm_list = []
        self.results = None

        video_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )  
        self.sub_rgb = self.create_subscription(
            Image,
            self.rgb_topic,
            self.get_rgb_image_callback,
            video_qos,
        )

        self.get_logger().info("FingerCtrl node started.")
        self.get_logger().info(f"Subscribe image topic: {self.rgb_topic}")
        self.get_logger().info(
            f"Display window: {self.window_name}, "
            f"resize_display={self.resize_display}, "
            f"keep_aspect_ratio={self.keep_aspect_ratio}, "
            f"display_width={self.display_width}, "
            f"display_height={self.display_height}"
        )
        self.get_logger().info(
            f"Control: use_distance_control={self.use_distance_control}, "
            f"deadzone={self.control_deadzone_percent}, "
            f"zero_snap={self.zero_snap_percent}, "
            f"smooth_alpha={self.smooth_alpha}, "
            f"min_dist_ratio={self.min_dist_ratio}, "
            f"max_dist_ratio={self.max_dist_ratio}"
        )

    def get_rgb_image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to decode Image to OpenCV image: {e}")
            return

        action = cv.waitKey(1) & 0xFF

        hand_img = self.find_hands(frame)
        lm_list = self.find_position(frame, draw=False)

        if len(lm_list) >= 10:
            x1, y1 = lm_list[4][1], lm_list[4][2]
            x2, y2 = lm_list[8][1], lm_list[8][2]
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

            cv.circle(hand_img, (x1, y1), 15, (255, 0, 255), cv.FILLED)
            cv.circle(hand_img, (x2, y2), 15, (255, 0, 255), cv.FILLED)
            cv.line(hand_img, (x1, y1), (x2, y2), (255, 0, 255), 3)
            cv.circle(hand_img, (cx, cy), 15, (255, 0, 255), cv.FILLED)

            raw_percent = self.compute_control_percent(lm_list)
            self.vol_per = self.smooth_control_percent(raw_percent)

            # 闭合状态显示绿色圆点
            if self.vol_per <= 0.0:
                cv.circle(hand_img, (cx, cy), 15, (0, 255, 0), cv.FILLED)

            self.value = np.interp(self.vol_per, [0, 100], [0, 255])

        # =========================
        # 根据当前 effect 处理原图
        # =========================
        current_effect = self.effect[self.index]

        if current_effect == "thresh":
            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            frame = cv.threshold(gray, self.value, 255, cv.THRESH_BINARY)[1]

        elif current_effect == "blur":
            sigma = np.interp(self.value, [0, 255], [0, 11])
            frame = cv.GaussianBlur(frame, (21, 21), sigma)

        elif current_effect == "hue":
            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
            hsv[:, :, 0] = (hsv[:, :, 0].astype(np.int16) + int(self.value)) % 180
            frame = cv.cvtColor(hsv.astype(np.uint8), cv.COLOR_HSV2BGR)

        elif current_effect == "enhance":
            enh_val = max(float(self.value) / 40.0, 0.1)
            clahe = cv.createCLAHE(clipLimit=enh_val, tileGridSize=(8, 8))
            lab = cv.cvtColor(frame, cv.COLOR_BGR2LAB)
            lab[:, :, 0] = clahe.apply(lab[:, :, 0])
            frame = cv.cvtColor(lab, cv.COLOR_LAB2BGR)

        # 按 f 切换效果
        if action == ord("f") or action == ord("F"):
            self.index += 1
            if self.index >= len(self.effect):
                self.index = 0

        # 按 q 退出
        if action == ord("q") or action == ord("Q"):
            self.get_logger().info("Pressed q, shutting down FingerCtrl node.")
            os._exit(0)  # pylint: disable=protected-access
            return

        # 右侧手部图像上绘制完整控制面板
        self.draw_control_panel(hand_img)

        # 原图 + 手部骨架图左右拼接
        dst = self.frame_combine(frame, hand_img)

        # 显示前缩放：保证完整显示，不裁剪
        dst = self.prepare_display_image(dst)

        cv.imshow(self.window_name, dst)

    # =========================
    # 控制量计算：距离比例 + 死区 + 滤波
    # =========================
    def clamp(self, value, min_value, max_value):
        return max(min_value, min(max_value, value))

    def compute_control_percent(self, lm_list):
        """
        计算手势控制百分比。

        默认使用：
        拇指尖 4 和食指尖 8 的距离 / 手掌参考长度 0->9

        这样比 calc_angle(4, 0, 8) 稳定，闭合时不会轻易跳到 6%。
        """

        if len(lm_list) < 10:
            return 0.0

        if self.use_distance_control:
            thumb_tip = lm_list[4][1], lm_list[4][2]
            index_tip = lm_list[8][1], lm_list[8][2]

            wrist = lm_list[0][1], lm_list[0][2]
            middle_mcp = lm_list[9][1], lm_list[9][2]

            palm_ref = self.get_dist(wrist, middle_mcp)
            if palm_ref < 1.0:
                return 0.0

            thumb_index_dist = self.get_dist(thumb_tip, index_tip)
            dist_ratio = thumb_index_dist / palm_ref

            raw_percent = np.interp(
                dist_ratio,
                [self.min_dist_ratio, self.max_dist_ratio],
                [0.0, 100.0],
            )

        else:
            # 保留角度控制作为可选方案
            angle = self.calc_angle(4, 0, 8)
            raw_percent = np.interp(angle, [0, 70], [0.0, 100.0])

        raw_percent = self.clamp(float(raw_percent), 0.0, 100.0)

        # 零点死区：闭合时小抖动直接认为是 0
        if raw_percent < self.control_deadzone_percent:
            raw_percent = 0.0

        return raw_percent

    def smooth_control_percent(self, raw_percent):
        """
        对控制百分比做低通滤波。
        raw_percent 为 0 时直接吸附到 0，避免闭合后还缓慢回落。
        """

        if raw_percent <= 0.0:
            self.filtered_vol_per = 0.0
            return 0.0

        alpha = self.clamp(self.smooth_alpha, 0.05, 1.0)

        self.filtered_vol_per = (
            alpha * raw_percent
            + (1.0 - alpha) * self.filtered_vol_per
        )

        # 低值强制归零，避免 3%~8% 来回跳
        if self.filtered_vol_per < self.zero_snap_percent:
            self.filtered_vol_per = 0.0

        self.filtered_vol_per = self.clamp(self.filtered_vol_per, 0.0, 100.0)

        if self.filtered_vol_per >= 99.0:
            self.filtered_vol_per = 100.0

        return self.filtered_vol_per

    # =========================
    # 图像显示与绘制
    # =========================
    def draw_control_panel(self, img):
        """
        动态绘制进度条、百分比和提示文字。
        所有坐标都根据当前图像尺寸计算，避免不同分辨率下显示不全。
        """

        img_h, img_w = img.shape[:2]

        margin_x = max(15, int(img_w * 0.04))
        bar_w = max(28, int(img_w * 0.045))

        bar_x1 = margin_x
        bar_x2 = min(img_w - margin_x - 1, bar_x1 + bar_w)

        bar_y1 = max(45, int(img_h * 0.20))
        bar_y2 = min(img_h - 70, int(img_h * 0.78))

        # 极小画面保护
        if bar_y2 <= bar_y1 + 30:
            bar_y1 = max(20, int(img_h * 0.15))
            bar_y2 = max(bar_y1 + 30, img_h - 50)

        bar_fill_y = int(
            np.interp(
                self.vol_per,
                [0.0, 100.0],
                [bar_y2, bar_y1],
            )
        )

        # 进度条外框
        cv.rectangle(
            img,
            (bar_x1, bar_y1),
            (bar_x2, bar_y2),
            (255, 0, 0),
            3,
        )

        # 进度条填充
        cv.rectangle(
            img,
            (bar_x1, bar_fill_y),
            (bar_x2, bar_y2),
            (0, 255, 0),
            cv.FILLED,
        )

        text_x = min(bar_x2 + 15, max(10, img_w - 260))

        percent_text = f"{int(self.vol_per)}%"
        percent_y = min(bar_y2, img_h - 25)

        cv.putText(
            img,
            percent_text,
            (text_x, percent_y),
            cv.FONT_HERSHEY_COMPLEX,
            0.9,
            (0, 255, 0),
            2,
        )

        effect_text = f"effect: {self.effect[self.index]}"
        effect_y = max(30, bar_y1 - 20)

        cv.putText(
            img,
            effect_text,
            (text_x, effect_y),
            cv.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
        )

        hint_text = "f: switch | q: quit"
        hint_y = min(img_h - 20, bar_y2 + 35)

        cv.putText(
            img,
            hint_text,
            (text_x, hint_y),
            cv.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            1,
        )

    def prepare_display_image(self, image):
        """
        显示前处理：
        - resize_display=False：显示原始拼接图
        - keep_aspect_ratio=True：等比例缩放并补黑边，保证完整显示不裁剪、不变形
        - keep_aspect_ratio=False：直接缩放到目标窗口大小
        """

        if (
            not self.resize_display
            or self.display_width <= 0
            or self.display_height <= 0
        ):
            return image

        if self.keep_aspect_ratio:
            return self.resize_with_padding(
                image,
                self.display_width,
                self.display_height,
            )

        return cv.resize(
            image,
            (self.display_width, self.display_height),
            interpolation=cv.INTER_AREA,
        )

    def resize_with_padding(self, image, target_w, target_h):
        """
        等比例缩放到目标尺寸内，并用黑边填充。
        这样不会裁剪图像，保证左右拼接画面完整显示。
        """

        src_h, src_w = image.shape[:2]

        if src_w <= 0 or src_h <= 0:
            return image

        scale = min(target_w / src_w, target_h / src_h)

        new_w = max(1, int(src_w * scale))
        new_h = max(1, int(src_h * scale))

        resized = cv.resize(
            image,
            (new_w, new_h),
            interpolation=cv.INTER_AREA,
        )

        canvas = np.zeros((target_h, target_w, 3), dtype=np.uint8)

        x_offset = (target_w - new_w) // 2
        y_offset = (target_h - new_h) // 2

        canvas[
            y_offset:y_offset + new_h,
            x_offset:x_offset + new_w,
        ] = resized

        return canvas

    # =========================
    # MediaPipe 手部处理
    # =========================
    def find_hands(self, frame, draw=True):
        img = np.zeros(frame.shape, np.uint8)

        img_rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.hands.process(img_rgb)

        if self.results.multi_hand_landmarks:
            for hand_lms in self.results.multi_hand_landmarks:
                if draw:
                    self.mp_draw.draw_landmarks(
                        img,
                        hand_lms,
                        self.mp_hand.HAND_CONNECTIONS,
                        self.lm_draw_spec,
                        self.draw_spec,
                    )

        return img

    def find_position(self, frame, draw=True):
        self.lm_list = []

        if self.results and self.results.multi_hand_landmarks:
            # 只取第一只手
            first_hand = self.results.multi_hand_landmarks[0]

            for idx, lm in enumerate(first_hand.landmark):
                h, w, _ = frame.shape
                cx, cy = int(lm.x * w), int(lm.y * h)

                self.lm_list.append([idx, cx, cy])

                if draw:
                    cv.circle(frame, (cx, cy), 15, (0, 0, 255), cv.FILLED)

        return self.lm_list

    # =========================
    # 数学辅助函数
    # =========================
    def get_dist(self, point1, point2):
        x1, y1 = point1
        x2, y2 = point2

        return abs(
            math.sqrt(
                math.pow(abs(y1 - y2), 2)
                + math.pow(abs(x1 - x2), 2)
            )
        )

    def calc_angle(self, pt1, pt2, pt3):
        point1 = self.lm_list[pt1][1], self.lm_list[pt1][2]
        point2 = self.lm_list[pt2][1], self.lm_list[pt2][2]
        point3 = self.lm_list[pt3][1], self.lm_list[pt3][2]

        a = self.get_dist(point1, point2)
        b = self.get_dist(point2, point3)
        c = self.get_dist(point1, point3)

        try:
            radian = math.acos(
                (math.pow(a, 2) + math.pow(b, 2) - math.pow(c, 2))
                / (2 * a * b)
            )
            angle = radian / math.pi * 180
        except Exception:
            angle = 0

        return abs(angle)

    def ensure_bgr(self, image):
        """
        保证图像是 BGR 三通道。
        这样 thresh 模式生成灰度图后，也能正常和右侧彩色图拼接。
        """

        if len(image.shape) == 2:
            return cv.cvtColor(image, cv.COLOR_GRAY2BGR)

        if len(image.shape) == 3 and image.shape[2] == 1:
            return cv.cvtColor(image, cv.COLOR_GRAY2BGR)

        return image

    def frame_combine(self, frame, src):
        frame = self.ensure_bgr(frame)
        src = self.ensure_bgr(src)

        frame_h, frame_w = frame.shape[:2]
        src_h, src_w = src.shape[:2]

        dst = np.zeros(
            (max(frame_h, src_h), frame_w + src_w, 3),
            np.uint8,
        )

        dst[:frame_h, :frame_w] = frame[:, :]
        dst[:src_h, frame_w:frame_w + src_w] = src[:, :]

        return dst

    def destroy_node(self):
        try:
            self.hands.close()
        except Exception:
            pass

        try:
            cv.destroyAllWindows()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = FingerCtrl()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, shutting down.")
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
