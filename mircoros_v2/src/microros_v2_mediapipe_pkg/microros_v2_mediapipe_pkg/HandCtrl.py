#!/usr/bin/env python3
# encoding: utf-8
"""
HandCtrl.py

中文说明:
    ROS2 Humble 手势控制节点。
    本版本订阅原始图像话题 /camera/image_raw，并在视频画面中显示手势代码含义。
    同时支持通过 launch 参数调大 OpenCV 视频显示窗口尺寸，默认使用小字体叠加文字，避免遮挡画面。

English Description:
    ROS2 Humble hand gesture control node.
    This version subscribes to the raw image topic /camera/image_raw and displays
    the gesture code meaning on the video frame. It also supports larger OpenCV display size through
    launch parameters. The overlay text uses small font by default to avoid blocking the video frame.
"""

import os

# MEDIAPIPE_DISABLE_TF: 禁用 MediaPipe TensorFlow 初始化 / Disable MediaPipe TensorFlow initialization.
os.environ.setdefault("MEDIAPIPE_DISABLE_TF", "1")
# MEDIAPIPE_DISABLE_GPU: 禁用 MediaPipe GPU 初始化 / Disable MediaPipe GPU initialization.
os.environ.setdefault("MEDIAPIPE_DISABLE_GPU", "1")

import threading
from time import sleep, time
from typing import List, Tuple

import cv2 as cv
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image

from .media_common import handDetector


class HandCtrl(Node):
    """手势控制节点 / Hand gesture control node."""

    def __init__(self):
        """初始化手势控制节点 / Initialize hand gesture control node."""
        super().__init__("hand_ctrl_node")

        # rgb_topic: 原始图像输入话题 / Raw image input topic.
        self.declare_parameter("rgb_topic", "/camera/image_raw")
        # cmd_vel_topic: 底盘速度输出话题 / Chassis velocity output topic.
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        # linear_speed: 手势控制线速度 / Linear speed for gesture control.
        self.declare_parameter("linear_speed", 0.1)
        # angular_speed: 手势控制角速度 / Angular speed for gesture control.
        self.declare_parameter("angular_speed", 0.7)
        # command_duration: 每次手势命令持续时间 / Duration for each gesture command.
        self.declare_parameter("command_duration", 0.5)
        # window_name: OpenCV 显示窗口名称 / OpenCV display window name.
        self.declare_parameter("window_name", "hand_ctrl")
        # display_width: 视频显示窗口宽度 / Video display window width.
        self.declare_parameter("display_width", 960)
        # display_height: 视频显示窗口高度 / Video display window height.
        self.declare_parameter("display_height", 720)
        # overlay_font_scale: 画面叠加文字大小 / Overlay text font scale.
        self.declare_parameter("overlay_font_scale", 0.32)
        # overlay_line_height: 画面叠加文字行高 / Overlay text line height.
        self.declare_parameter("overlay_line_height", 13)
        # overlay_show_legend: 是否显示手势说明表 / Whether to show gesture legend.
        self.declare_parameter("overlay_show_legend", False)

        # rgb_topic: 原始图像输入话题 / Raw image input topic.
        self.rgb_topic = self.get_parameter("rgb_topic").value
        # cmd_vel_topic: 底盘速度输出话题 / Chassis velocity output topic.
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        # linear_speed: 手势控制线速度 / Linear speed for gesture control.
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        # angular_speed: 手势控制角速度 / Angular speed for gesture control.
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        # command_duration: 每次手势命令持续时间 / Duration for each gesture command.
        self.command_duration = float(self.get_parameter("command_duration").value)
        # window_name: OpenCV 显示窗口名称 / OpenCV display window name.
        self.window_name = str(self.get_parameter("window_name").value)
        # display_width: 视频显示窗口宽度 / Video display window width.
        self.display_width = int(self.get_parameter("display_width").value)
        # display_height: 视频显示窗口高度 / Video display window height.
        self.display_height = int(self.get_parameter("display_height").value)
        # overlay_font_scale: 画面叠加文字大小 / Overlay text font scale.
        self.overlay_font_scale = float(self.get_parameter("overlay_font_scale").value)
        # overlay_line_height: 画面叠加文字行高 / Overlay text line height.
        self.overlay_line_height = int(self.get_parameter("overlay_line_height").value)
        # overlay_show_legend: 是否显示手势说明表 / Whether to show gesture legend.
        self.overlay_show_legend = bool(self.get_parameter("overlay_show_legend").value)

        # qos_profile: 图像订阅 QoS 配置 / QoS profile for image subscription.
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # bridge: CV Bridge 工具 / CV Bridge helper.
        self.bridge = CvBridge()
        # subscription: 原始图像订阅器 / Raw image subscriber.
        self.subscription = self.create_subscription(
            Image,
            self.rgb_topic,
            self.image_callback,
            self.qos_profile,
        )
        # cmd_vel_qos: 底盘速度 QoS（与 ESP32 固件端 BEST_EFFORT 对齐） / Chassis velocity QoS.
        self.cmd_vel_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        # pub_cmdVel: 底盘速度发布器 / Chassis velocity publisher.
        self.pub_cmdVel = self.create_publisher(Twist, self.cmd_vel_topic, self.cmd_vel_qos)
        # hand_detector: 手部检测器 / Hand detector.
        self.hand_detector = handDetector()
        # previous_time: 上一次 FPS 统计时间 / Previous FPS timestamp.
        self.previous_time = 0.0
        # command_event: 手势命令互斥事件 / Gesture command mutex event.
        self.command_event = threading.Event()
        self.command_event.set()
        self._shutting_down = False
        # gesture_state_lock: 手势显示状态互斥锁 / Gesture display state mutex.
        self.gesture_state_lock = threading.Lock()
        # latest_finger_code: 最近手势代码 / Latest finger code.
        self.latest_finger_code = "None"
        # latest_action_text: 最近动作含义 / Latest action meaning.
        self.latest_action_text = "No hand detected - Stop"
        # latest_twist_text: 最近速度命令文本 / Latest velocity command text.
        self.latest_twist_text = "linear.x=0.00 angular.z=0.00"
        # display_window_initialized: 显示窗口初始化标志 / Display window initialized flag.
        self.display_window_initialized = False
        self.get_logger().info(f"HandCtrl node started. Subscribe raw image topic: {self.rgb_topic}")

    def process(self, frame):
        """
        处理 OpenCV 图像并识别手势 / Process OpenCV frame and recognize gesture.

        Args:
            frame: BGR 图像 / BGR image.

        Returns:
            frame: 绘制后的 BGR 图像 / Rendered BGR image.
        """
        # frame: 水平翻转后的显示图像 / Horizontally flipped display image.
        frame = cv.flip(frame, 1)
        frame, _, _ = self.hand_detector.findHands(frame)
        if len(self.hand_detector.lmList) != 0 and not self._shutting_down:
            threading.Thread(target=self.hand_threading, args=(self.hand_detector.lmList,), daemon=True).start()
        else:
            self.pub_cmdVel.publish(Twist())
            self.update_gesture_display_state("None", "No hand detected - Stop", Twist())

        # current_time: 当前时间 / Current time.
        current_time = time()
        # fps: 画面帧率 / Frame rate.
        fps = 0 if self.previous_time == 0 else 1 / max(current_time - self.previous_time, 1e-3)
        self.previous_time = current_time
        self.draw_overlay_text(frame, fps)
        return frame

    def hand_threading(self, landmark_list):
        """
        根据手指状态发布底盘速度 / Publish chassis velocity according to finger state.

        Args:
            landmark_list: 手部关键点列表 / Hand landmark list.
        """
        del landmark_list
        if not self.command_event.is_set():
            return
        self.command_event.clear()
        try:
            # fingers: 手指伸展状态列表 / Finger up state list.
            fingers = self.hand_detector.fingersUp()
            self.get_logger().info(f"fingers: {fingers}")
            # twist_command: 底盘速度命令 / Chassis velocity command.
            twist_command = Twist()
            # finger_code: 手势代码 / Gesture finger code.
            finger_code = self.format_finger_code(fingers)
            # action_text: 手势动作含义 / Gesture action meaning.
            action_text = self.get_action_text_from_fingers(fingers)

            if sum(fingers) == 5:
                twist_command.linear.x = self.linear_speed
                twist_command.angular.z = 0.0
            elif sum(fingers) == 3:
                twist_command.linear.x = -self.linear_speed
                twist_command.angular.z = 0.0
            elif sum(fingers) == 1 and fingers[1] == 1:
                twist_command.linear.x = self.linear_speed
                twist_command.angular.z = -self.angular_speed
            elif sum(fingers) == 2 and fingers[1] == 1 and fingers[2] == 1:
                twist_command.linear.x = 0.0
                twist_command.angular.z = 0.0
            elif sum(fingers) == 4 and fingers[0] == 0:
                twist_command.linear.x = self.linear_speed
                twist_command.angular.z = self.angular_speed
            else:
                twist_command.linear.x = 0.0
                twist_command.angular.z = 0.0

            self.update_gesture_display_state(finger_code, action_text, twist_command)
            self.pub_cmdVel.publish(twist_command)
            sleep(self.command_duration)
        finally:
            self.command_event.set()

    @staticmethod
    def format_finger_code(fingers: List[int]) -> str:
        """
        格式化手势代码 / Format finger state code.

        Args:
            fingers: 手指伸展状态列表 / Finger up state list.

        Returns:
            str: 手势代码字符串 / Gesture code string.
        """
        return "[" + ",".join(str(int(value)) for value in fingers) + "]"

    def get_action_text_from_fingers(self, fingers: List[int]) -> str:
        """
        获取手势动作含义 / Get gesture action meaning.

        Args:
            fingers: 手指伸展状态列表 / Finger up state list.

        Returns:
            str: 动作含义 / Action meaning.
        """
        if sum(fingers) == 5:
            return "Five fingers: Forward"
        if sum(fingers) == 3:
            return "Three fingers: Backward"
        if sum(fingers) == 1 and fingers[1] == 1:
            return "Index finger: Forward + Right turn"
        if sum(fingers) == 2 and fingers[1] == 1 and fingers[2] == 1:
            return "Index + Middle: Stop"
        if sum(fingers) == 4 and fingers[0] == 0:
            return "Four fingers without thumb: Forward + Left turn"
        return "Unknown gesture: Stop"

    def update_gesture_display_state(self, finger_code: str, action_text: str, twist_command: Twist) -> None:
        """
        更新手势显示状态 / Update gesture display state.

        Args:
            finger_code: 手势代码 / Gesture code.
            action_text: 动作含义 / Action meaning.
            twist_command: 速度命令 / Velocity command.
        """
        with self.gesture_state_lock:
            self.latest_finger_code = finger_code
            self.latest_action_text = action_text
            self.latest_twist_text = "linear.x=%.2f angular.z=%.2f" % (
                twist_command.linear.x,
                twist_command.angular.z,
            )

    def draw_overlay_text(self, frame, fps: float) -> None:
        """
        在画面中绘制 FPS、当前手势和手势说明 / Draw FPS, current gesture, and gesture legend.

        Args:
            frame: BGR 图像 / BGR image.
            fps: 当前帧率 / Current FPS.
        """
        with self.gesture_state_lock:
            # finger_code: 当前手势代码 / Current gesture code.
            finger_code = self.latest_finger_code
            # action_text: 当前动作含义 / Current action meaning.
            action_text = self.latest_action_text
            # twist_text: 当前速度命令 / Current velocity command.
            twist_text = self.latest_twist_text

        # overlay_lines: 当前状态文字 / Current status overlay lines.
        overlay_lines = [
            "FPS: %d" % int(fps),
            "Gesture code: %s" % finger_code,
            "Action: %s" % action_text,
            "Cmd: %s" % twist_text,
        ]
        if self.overlay_show_legend:
            overlay_lines.extend([
                "5F:Forward | 3F:Back",
                "Index:Right | I+M:Stop | 4F:Left",
            ])

        for line_index, line_text in enumerate(overlay_lines):
            # text_origin: 文字绘制位置 / Text drawing position.
            text_origin = (6, 15 + line_index * self.overlay_line_height)
            cv.putText(
                frame,
                line_text,
                text_origin,
                cv.FONT_HERSHEY_SIMPLEX,
                self.overlay_font_scale,
                (0, 255, 255),
                1,
                cv.LINE_AA,
            )

    def initialize_display_window_if_needed(self) -> None:
        """按需初始化显示窗口 / Initialize display window if needed."""
        if self.display_window_initialized:
            return
        cv.namedWindow(self.window_name, cv.WINDOW_NORMAL)
        cv.resizeWindow(self.window_name, int(self.display_width), int(self.display_height))
        self.display_window_initialized = True

    def image_callback(self, image_msg):
        """
        原始图像回调 / Raw image callback.

        Args:
            image_msg: 原始图像消息 / Raw image message.
        """
        try:
            # frame: 解码后的 BGR 图像 / Decoded BGR image.
            frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        except Exception as exception:  # pylint: disable=broad-except
            self.get_logger().error(f"Failed to convert ROS Image to OpenCV image: {exception}")
            return

        frame = self.process(frame)
        self.initialize_display_window_if_needed()
        # display_frame: 调整后的显示图像 / Resized display image.
        display_frame = cv.resize(frame, (int(self.display_width), int(self.display_height)))
        cv.imshow(self.window_name, display_frame)
        if cv.waitKey(1) & 0xFF == ord("q"):
            self.get_logger().info("Pressed q, shutting down HandCtrl node.")
            self.cancel()
            os._exit(0)  # pylint: disable=protected-access

    def cancel(self):
        """停止底盘运动 / Stop chassis movement."""
        self.pub_cmdVel.publish(Twist())

    def destroy_node(self):
        """销毁节点并释放资源 / Destroy node and release resources."""
        self._shutting_down = True
        self.command_event.set()  # 唤醒可能在 sleep 中的 hand_threading
        self.cancel()
        try:
            self.hand_detector.close()
        except Exception:
            pass
        try:
            cv.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    """
    节点入口函数 / Node entry point.

    Args:
        args: ROS 参数 / ROS arguments.
    """
    rclpy.init(args=args)
    # hand_ctrl_node: 手势控制节点 / Hand gesture control node.
    hand_ctrl_node = HandCtrl()
    try:
        rclpy.spin(hand_ctrl_node)
    except KeyboardInterrupt:
        hand_ctrl_node.get_logger().info("KeyboardInterrupt received, shutting down.")
    finally:
        hand_ctrl_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
