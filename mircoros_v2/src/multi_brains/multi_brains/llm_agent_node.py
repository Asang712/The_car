#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
llm_agent_node.py

LLM智能体核心节点：文字交互 + 意图理解 + 动作执行
LLM Agent Core Node: Text interaction + Intent understanding + Action execution

本节点只负责装配与反馈闭环编排：Dify 决策 → 动作执行 → 结果反馈。
具体的硬件控制逻辑拆分到独立控制器：

- :class:`PtzController`        云台舵机
- :class:`MotionController`     底盘 cmd_vel
- :class:`NavigationController` Nav2 导航 + zero 动态点
- :class:`DifyCoordinator`      Dify SSE 交互 + 响应解析
- :class:`KcfTracker`           KCF 视觉跟随控制
"""

import os
import re
import json
import time
import sys
import subprocess
import tempfile
import threading

import yaml
import cv2

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
)

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

from microros_v2_bringup.robot_tools.robot_tool import GetRobotInfo

from interfaces.msg import LlmTextRequest, LlmTextResponse

from multi_brains.core.hardware_profile import (
    CAR_TYPE_LABELS,
    VALID_CAR_TYPES,
    has_camera,
    has_ptz,
    resolve_car_type,
)
from multi_brains.core.action_registry import (
    describe_action,
    is_finishtask,
    is_follow_line,
    is_servo_action,
    is_stop_action,
    is_visual_follow,
)
from multi_brains.core.feedback_protocol import (
    build_action_feedback,
    select_round_action,
)
from multi_brains.controllers.kcf_tracker import KcfTracker
from multi_brains.controllers.follow_line_controller import FollowLineController
from multi_brains.controllers.motion_controller import MotionController
from multi_brains.controllers.navigation_controller import NavigationController
from multi_brains.controllers.ptz_controller import PtzController
from multi_brains.controllers.visual_servo_controller import VisualServoController
from multi_brains.core.command_state import CommandState
from multi_brains.core.local_command_parser import LocalCommandParser
from multi_brains.services.dify_coordinator import DifyCoordinator
from multi_brains.utils.common_tools import kill_process_tree


class LlmAgentNode(Node):
    """LLM智能体核心节点"""

    def __init__(self):
        super().__init__("llm_agent_node")

        self.get_logger().info("Initializing LLM Agent Node...")

        # 加载配置（含路径可配置化回退）
        self.load_config()

        # 初始化ROS通信：文字输入输出 + 底盘速度
        self.init_ros_communication()

        # 初始化视觉模块
        self.init_vision()

        # 抢占和每条指令的独立状态必须先于控制器初始化。
        self.command_lock = threading.Lock()
        self.current_command_id = 0
        self.worker_thread = None
        self.worker_context = threading.local()
        self.fallback_command_state = CommandState(command_id=0)
        self.active_action_lock = threading.Lock()
        self.active_action_ids = set()
        self.follow_line_process = None
        self.camera_preview_process = None

        # 装配控制器
        self.ptz = PtzController(self, self.config, self.car_type)
        self.motion = MotionController(self, self.cmd_vel_pub, self.config)
        self.visual_servo = VisualServoController(
            self, self.cmd_vel_pub, self.config
        )
        self.navigation = NavigationController(self, self.config)
        self.follow_line_controller = FollowLineController(
            self, self.config, self.cmd_vel_pub
        )
        self.local_parser = LocalCommandParser(self.config)
        self.dify = None
        if self.command_mode == "dify":
            self.dify = DifyCoordinator(self, self.config)
        else:
            self.get_logger().info("[LOCAL] 本地固定指令模式已启用，Dify 不会初始化")
        self.kcf = KcfTracker(
            self, self.config, self.car_type, self.cmd_vel_pub, self.ptz
        )

        # 初始化动作注册表
        self.init_action_registry()

        self.get_logger().info("LLM Agent Node initialized successfully")

    # ===== 配置 =====

    def load_config(self):
        """加载配置文件"""
        self.declare_parameter("config_file", "")
        config_file = self.get_parameter("config_file").value

        if not config_file or not os.path.exists(config_file):
            self.get_logger().warn("Config file not found, using defaults")
            self.config = {
                "DEBUG_MODE": False,
                "LANGUAGE": "zh",
            }
        else:
            self.get_logger().info(f"[CONFIG] 加载配置文件: {config_file}")
            with open(config_file, "r", encoding="utf-8") as config_stream:
                self.config = yaml.safe_load(config_stream) or {}

        self.command_mode = str(self.config.get("COMMAND_MODE", "local")).strip().lower()
        if self.command_mode not in ("local", "dify"):
            self.get_logger().warn(
                f"[CONFIG] 未知 COMMAND_MODE={self.command_mode}，回退到 local"
            )
            self.command_mode = "local"

        # Dify 配置改从 robot_info.json（robot_tool.GetRobotInfo）读取
        try:
            robot_info = GetRobotInfo() or {}
        except Exception as error:
            self.get_logger().warn(f"[CONFIG] 读取 robot_info.json 失败: {error}")
            robot_info = {}
        dify_base_url = str(robot_info.get("DIFY_BASE_URL", "") or "").strip()
        self.config["DIFY_BASE_URL"] = dify_base_url or "http://localhost/v1"
        dify_api_key = str(robot_info.get("DIFY_API_KEY", "") or "").strip()
        self.config["DIFY_API_KEY"] = dify_api_key
        if not dify_api_key and self.command_mode == "dify":
            self.get_logger().warn("[CONFIG] robot_info.json 中未设置 DIFY_API_KEY")

        self.debug_mode = self.config.get("DEBUG_MODE", False)
        self.language = self.config.get("LANGUAGE", "zh")
        system_config = self.config.setdefault("SYSTEM", {})
        configured_car_type = system_config.get("hardware_variant", "ptz")
        environment_car_type = os.environ.get("CAR_TYPE", "")
        if (
            environment_car_type
            and environment_car_type.strip().lower() not in VALID_CAR_TYPES
        ):
            self.get_logger().warn(
                f"[CONFIG] CAR_TYPE='{environment_car_type}' 无效，"
                f"可选值: {sorted(VALID_CAR_TYPES)}"
            )
        self.car_type = resolve_car_type(environment_car_type, configured_car_type)
        system_config["hardware_variant"] = self.car_type
        self.get_logger().info(
            f"[CONFIG] COMMAND_MODE={self.command_mode}, "
            f"DEBUG_MODE={self.debug_mode}, LANGUAGE={self.language}, "
            f"CAR_TYPE={self.car_type} ({CAR_TYPE_LABELS[self.car_type]})"
        )

        self._resolve_config_paths()

    def _resolve_config_paths(self) -> None:
        """把硬编码/不可用的路径回退到包内或可写位置。

        - map_mapping_file 不存在时回退到本包 share 目录下的同名文件；
        - cache_path 的可写性在 init_vision 中单独处理（需要 makedirs）。
        """
        nav_config = self.config.setdefault("NAVIGATION", {})
        map_file = nav_config.get("map_mapping_file", "")
        if not map_file or not os.path.exists(map_file):
            try:
                share_map = os.path.join(
                    get_package_share_directory("multi_brains"),
                    "config",
                    "map_mapping.yaml",
                )
                if os.path.exists(share_map):
                    nav_config["map_mapping_file"] = share_map
                    self.get_logger().info(
                        f"[CONFIG] map_mapping_file 回退到包内: {share_map}"
                    )
            except Exception as error:
                self.get_logger().warn(
                    f"[CONFIG] 无法定位包内 map_mapping.yaml: {error}"
                )

    # ===== ROS 通信 =====

    def init_ros_communication(self):
        """初始化ROS通信：文字输入输出 + 底盘速度发布"""
        # 文字输入订阅
        self.text_sub = self.create_subscription(
            LlmTextRequest, "/llm_text_input", self.text_callback, 10
        )

        # 文字输出发布
        self.text_pub = self.create_publisher(LlmTextResponse, "/llm_text_output", 10)

        # 速度发布 — QoS 与 ESP32 固件端 BEST_EFFORT 对齐
        self.cmd_vel_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        configured_cmd_vel_topic = self.config.get("SYSTEM", {}).get(
            "cmd_vel_topic", "/cmd_vel"
        )
        self.declare_parameter("cmd_vel_topic", configured_cmd_vel_topic)
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, self.cmd_vel_qos)
        self.get_logger().info(f"[CONFIG] cmd_vel_topic={cmd_vel_topic}")

    def init_vision(self):
        """初始化视觉模块"""
        self.bridge = CvBridge()
        self.latest_image_msg = None
        self.latest_cv_image = None
        self.latest_image_seq = 0
        self.latest_image_lock = threading.Lock()

        camera_config = self.config.get("CAMERA", {})
        configured_image_topic = camera_config.get("image_topic", "/camera/image_raw")
        self.declare_parameter("image_topic", configured_image_topic)
        image_topic = self.get_parameter("image_topic").value

        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
            ),
        )

        self.image_cache_path = self._resolve_writable_cache_path(
            camera_config.get("cache_path", "")
        )
        cache_directory = os.path.dirname(self.image_cache_path)
        if cache_directory:
            os.makedirs(cache_directory, exist_ok=True)
        self.get_logger().info(f"[CONFIG] image_cache_path={self.image_cache_path}")

    def _resolve_writable_cache_path(self, configured_path: str) -> str:
        """展开用户目录并确保缓存图所在目录存在且可写。"""
        configured_path = str(configured_path or "").strip()
        candidate = (
            os.path.abspath(os.path.expanduser(configured_path))
            if configured_path
            else os.path.join(tempfile.gettempdir(), "llm_camera.jpg")
        )
        cache_dir = os.path.dirname(candidate)
        try:
            if cache_dir:
                os.makedirs(cache_dir, exist_ok=True)
            if not cache_dir or os.access(cache_dir, os.W_OK):
                return candidate
        except OSError as error:
            self.get_logger().warn(
                f"[CONFIG] 无法创建 cache_path 目录 {cache_dir}: {error}"
            )

        fallback = os.path.join(tempfile.gettempdir(), "llm_camera.jpg")
        self.get_logger().warn(
            f"[CONFIG] cache_path 不可写: {candidate}，回退到: {fallback}"
        )
        return fallback

    # ===== 动作注册表 =====

    def init_action_registry(self):
        """初始化动作注册表"""
        self.action_registry = {
            "navigation": self.navigation.execute_navigation,
            "follow_line": self.execute_follow_line,
            "track_kcf": self.kcf.execute_track_kcf,
            "seewhat": self.execute_seewhat,
            "move_forward": self.motion.execute_move_forward,
            "move_forward_distance": self.motion.execute_move_forward_distance,
            "move_backward_distance": (
                self.motion.execute_move_backward_distance
            ),
            "find_object": self.visual_servo.execute_find_object,
            "push_object": self.motion.execute_push,
            "move_backward": self.motion.execute_move_backward,
            "turn_left": self.motion.execute_turn_left,
            "turn_right": self.motion.execute_turn_right,
            "stop": self.execute_stop,
            "servo1_move": self.ptz.execute_servo1_move,
            "servo2_move": self.ptz.execute_servo2_move,
            "servo_init": self.ptz.execute_servo_init,
            "servo_shake": self.ptz.execute_servo_shake,
            "servo_nod": self.ptz.execute_servo_nod,
            "get_current_pose": self.navigation.get_current_pose,
            "observe_surroundings": self.execute_observe_surroundings,
            "finishtask": self.execute_finish_task,
        }

    # ===== 回调函数 =====

    def text_callback(self, msg: LlmTextRequest):
        """处理用户文字输入"""
        if msg.is_feedback:
            self.get_logger().info(f"[INPUT] 收到反馈: {msg.text}")
            return

        user_text = msg.text.strip()
        if not user_text:
            return

        with self.command_lock:
            self.current_command_id += 1
            command_id = self.current_command_id

        self.get_logger().info(
            f"[INPUT] ═══ 收到用户输入#{command_id}: '{user_text}' ═══"
        )

        if self.is_action_running() or self.kcf.kcf_tracking_active:
            self.get_logger().info("[INPUT] 新指令到达，正在中断当前动作")
            self.cancel_current_action(
                recenter_camera=self._is_stop_user_command(user_text)
            )

        if user_text == "__reset_conversation__":
            if self.dify is not None:
                self.dify.reset_conversation_id()
            self.local_parser.reset_history()
            response = (
                "对话上下文已重置"
                if self.command_mode == "dify"
                else "本地指令历史已重置"
            )
            self.publish_latest_response(command_id, response, "[]", "")
            return

        self.worker_thread = threading.Thread(
            target=self.process_text_command, args=(command_id, user_text), daemon=True
        )
        self.worker_thread.start()

    def process_text_command(self, command_id: int, user_text: str):
        """通过 Dify 决策 -> 动作执行 -> 结果反馈闭环处理文字指令。"""
        t_start = time.time()
        self.worker_context.state = CommandState(command_id=command_id)
        if self.should_interrupt():
            self.get_logger().info(
                f"[INPUT] 指令#{command_id} 已被更新指令替代，停止处理"
            )
            return

        if self.command_mode == "local":
            self.process_local_command(command_id, user_text)
            return

        self.publish_progress("[决策] 正在思考...")
        self.clear_pending_action_feedback()
        self.set_terminal_action_result_text("")

        feedback_config = self.config.get("FEEDBACK_LOOP", {})
        max_rounds = max(1, int(feedback_config.get("max_rounds", 12)))
        request_text = user_text
        robot_feedback = None
        feedback_image_paths = []
        final_response_text = ""
        executed_actions = []
        deferred_finish = False
        camera_prepared = False

        for round_index in range(1, max_rounds + 1):
            if self.should_interrupt():
                self.get_logger().info(
                    f"[INPUT] 指令#{command_id} 已被更新指令替代，停止反馈闭环"
                )
                return

            if self.debug_mode:
                self.get_logger().info(
                    f"[DIFY] 第 {round_index}/{max_rounds} 轮请求, "
                    f"robot_feedback={robot_feedback is True}, "
                    f"images={len(feedback_image_paths)}"
                )
            dify_response = self.dify.call_agent(
                request_text,
                robot_feedback=robot_feedback,
                image_paths=feedback_image_paths,
            )

            if self.should_interrupt():
                self.get_logger().info(
                    f"[DIFY] 指令#{command_id} 的Dify响应已过期，丢弃"
                )
                return
            if not dify_response:
                final_response_text = "Dify服务调用失败"
                break

            actions, response_text = self.dify.parse_response(dify_response)
            if response_text:
                final_response_text = response_text

            if self.debug_mode:
                self.get_logger().info(
                    f"[DIFY] 第{round_index}轮解析: "
                    f"response='{response_text[:80]}', actions={actions}"
                )

            # 兼容仍按旧纯文本视觉协议配置的 Dify。纯文本没有后续动作时直接结束。
            if not actions:
                if deferred_finish:
                    self.execute_finish_task()
                    executed_actions.append("finishtask()")
                break

            action_batch, ignored_actions, finish_requested = select_round_action(
                actions
            )
            deferred_finish = deferred_finish or finish_requested

            # 中间动作决策作为进度发布；纯结束回答留到 result，仅输出一次。
            if response_text and action_batch:
                stage_name = "决策" if round_index == 1 else "反馈决策"
                self.publish_progress(f"[{stage_name}] {response_text}")

            if ignored_actions:
                self.get_logger().warn(
                    "[ACTION_SERVICE] Dify本轮返回了多个动作，"
                    f"按反馈协议仅执行第一个: {action_batch[0]}; "
                    f"其余动作等待下一轮重新决策: {ignored_actions}"
                )

            self.publish_decision_plan(action_batch)

            if not action_batch and finish_requested:
                self.execute_finish_task()
                executed_actions.append("finishtask()")
                break

            if action_batch and not camera_prepared:
                if not self.prepare_camera_for_actions(action_batch):
                    final_response_text = "摄像头复位失败，任务已停止"
                    break
                camera_prepared = True

            batch_success = self.execute_action_batch(action_batch, executed_actions)
            if batch_success is None:
                return

            if self.should_interrupt():
                return

            if self._should_end_feedback_loop_after_actions(action_batch):
                final_response_text = (
                    self.get_terminal_action_result_text()
                    or self._build_terminal_action_result_text(
                        action_batch, batch_success
                    )
                )
                self.execute_finish_task()
                if self.debug_mode:
                    self.get_logger().info(
                        "[ACTION_SERVICE] 持续型视觉动作已结束，" "终止本轮Dify反馈闭环"
                    )
                    self.get_logger().info("[ACTION_SERVICE] 退出动作执行")
                break

            pending_query, pending_images = self.consume_pending_action_feedback()
            if pending_query:
                if self.debug_mode:
                    self.get_logger().info(f"[ACTION_SERVICE] {pending_query}")
                    self.get_logger().info("[ACTION_SERVICE] 退出动作执行")
                request_text = pending_query
                feedback_image_paths = pending_images
                robot_feedback = True
                continue

            request_text = self.build_action_feedback_query(action_batch, batch_success)
            if self.debug_mode:
                self.get_logger().info(f"[ACTION_SERVICE] {request_text}")
                self.get_logger().info("[ACTION_SERVICE] 退出动作执行")
            feedback_image_paths = []
            robot_feedback = True
        else:
            final_response_text = (
                f"任务反馈超过最大轮次 {max_rounds}，已停止继续请求Dify"
            )
            self.get_logger().error(f"[LOOP] {final_response_text}")
            self.execute_stop(recenter_camera=False)
            self.execute_finish_task()

        if self.should_interrupt():
            return

        if not final_response_text:
            final_response_text = "任务处理完成"
        action_json = json.dumps(executed_actions, ensure_ascii=False)
        final_conversation_id = (
            self.dify.get_conversation_id() or "" if self.dify is not None else ""
        )
        self.publish_latest_response(
            command_id, final_response_text, action_json, final_conversation_id
        )

        t_total = time.time() - t_start
        self.get_logger().info(
            f"[INPUT] ═══ 指令#{command_id}处理完成, "
            f"反馈轮次≤{max_rounds}, 总耗时: {t_total:.2f}s ═══"
        )

    def process_local_command(self, command_id: int, user_text: str):
        """离线解析并执行固定指令，失败时不继续执行后续动作。"""
        t_start = time.time()
        try:
            actions = self.local_parser.parse(user_text)
        except ValueError as error:
            self.get_logger().warn(f"[LOCAL] {error}")
            self.publish_latest_response(command_id, str(error), "[]", "")
            return

        if self.should_interrupt():
            return

        self.clear_pending_action_feedback()
        self.set_terminal_action_result_text("")
        self.publish_progress(f"[本地] 已识别 {len(actions)} 个动作，无需联网")
        self.publish_decision_plan(actions)

        if not self.prepare_camera_for_actions(actions):
            self.publish_latest_response(
                command_id, "摄像头复位失败，任务已停止", "[]", ""
            )
            return

        executed_actions = []
        success = True
        for action in actions:
            batch_success = self.execute_action_batch([action], executed_actions)
            if batch_success is None:
                return
            if not batch_success:
                success = False
                self.execute_stop(recenter_camera=False)
                break

        if self.should_interrupt():
            return

        response_text = "本地指令执行完成" if success else "动作执行失败，已停止后续动作"
        action_json = json.dumps(executed_actions, ensure_ascii=False)
        self.publish_latest_response(command_id, response_text, action_json, "")
        self.get_logger().info(
            f"[LOCAL] 指令#{command_id}处理完成，"
            f"结果={'成功' if success else '失败'}，总耗时: {time.time() - t_start:.2f}s"
        )

    def image_callback(self, msg: Image):
        """缓存最新原始图像"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as error:
            self.get_logger().warn(f"[CAMERA] 图像转换失败: {error}")
            return
        with self.latest_image_lock:
            self.latest_image_msg = msg
            self.latest_cv_image = cv_image
            self.latest_image_seq += 1

    def get_latest_image_snapshot(self):
        """Return a copy of the latest converted image, if one is available."""
        with self.latest_image_lock:
            if self.latest_cv_image is None:
                return None
            return self.latest_cv_image.copy()

    def wait_for_image_after(self, sequence: int, timeout: float):
        """Wait for and copy a frame newer than ``sequence``."""
        deadline = time.monotonic() + max(0.0, timeout)
        while time.monotonic() < deadline:
            if self.should_interrupt():
                return None
            with self.latest_image_lock:
                if (
                    self.latest_image_seq > sequence
                    and self.latest_cv_image is not None
                ):
                    return self.latest_cv_image.copy()
            time.sleep(0.02)
        return None

    def build_feedback_image_path(self, prefix: str, index: int | None = None) -> str:
        """Build a fixed path so each new capture replaces the previous image."""
        directory = os.path.dirname(self.image_cache_path)
        suffix = f"_{index}" if index is not None else ""
        return os.path.join(
            directory,
            f"{prefix}{suffix}.jpg",
        )

    def show_camera_preview(self, image_path: str) -> None:
        """Launch a non-blocking viewer for a completed photo when a display exists."""
        camera_config = self.config.get("CAMERA", {})
        if not camera_config.get("show_preview", True):
            return
        if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            self.get_logger().info("[SEEWHAT] 无图形显示环境，跳过拍照预览")
            return

        duration_sec = max(0.0, float(camera_config.get("preview_duration_sec", 2.5)))
        if duration_sec == 0.0:
            return

        window_name = str(camera_config.get("preview_window_name", "LLM Camera"))
        qt_platform = str(camera_config.get("preview_qt_platform", "xcb"))
        try:
            previous_process = self.camera_preview_process
            if previous_process is not None and previous_process.poll() is None:
                previous_process.terminate()

            self.camera_preview_process = subprocess.Popen(
                [
                    sys.executable,
                    "-m",
                    "multi_brains.clients.image_preview",
                    image_path,
                    "--duration",
                    str(duration_sec),
                    "--window-name",
                    window_name,
                    "--qt-platform",
                    qt_platform,
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            threading.Thread(
                target=self._reap_camera_preview,
                args=(self.camera_preview_process,),
                daemon=True,
            ).start()
            self.get_logger().info(
                f"[SEEWHAT] 正在显示已保存照片 {duration_sec:.1f} 秒 "
                f"(Qt={qt_platform})"
            )
        except OSError as error:
            self.get_logger().warn(f"[SEEWHAT] 无法启动拍照预览: {error}")

    def _reap_camera_preview(self, process) -> None:
        """Wait for a short-lived photo viewer and clear its process reference."""
        try:
            process.wait()
        except OSError:
            return
        if self.camera_preview_process is process:
            self.camera_preview_process = None

    # ===== 动作执行 =====

    def execute_action_batch(
        self, action_batch: list, executed_actions: list
    ) -> bool | None:
        """Execute the selected round action batch.

        Returns None when the current command was interrupted.
        """
        self.clear_pending_action_feedback()
        batch_success = True
        command_id = self.get_current_worker_command_id()
        with self.active_action_lock:
            self.active_action_ids.add(command_id)
        if self.debug_mode:
            self.get_logger().info(
                "[ACTION_SERVICE] 进入动作执行，" f"准备执行动作列表: {action_batch}"
            )
        try:
            for action_index, action in enumerate(action_batch, start=1):
                if self.should_interrupt():
                    self.get_logger().info("[ACTION] 当前指令已过期，停止执行")
                    return None

                self.publish_progress(
                    f"[动作 {action_index}/{len(action_batch)}] {action}"
                )
                try:
                    result = self.execute_action(action)
                except Exception as error:
                    self.get_logger().error(
                        f"[ACTION] 动作 '{action}' 执行异常: "
                        f"{type(error).__name__}: {error}"
                    )
                    result = False

                executed_actions.append(action)
                batch_success = batch_success and bool(result)
                status_text = "成功 ✓" if result else "失败 ✗"
                self.publish_progress(
                    f"[动作 {action_index}/{len(action_batch)}] {action} {status_text}"
                )
        finally:
            with self.active_action_lock:
                self.active_action_ids.discard(command_id)

        return batch_success

    def execute_action(self, action_str: str) -> bool:
        """执行单个动作"""
        if not isinstance(action_str, str):
            self.get_logger().error(
                f"[ACTION] 动作类型无效: {type(action_str)}, 值: {action_str}"
            )
            return False

        action_str = action_str.strip()
        if self.debug_mode:
            self.get_logger().info(f"[ACTION_SERVICE] 解析动作: '{action_str}'")
        match = re.fullmatch(r"(\w+)\((.*)\)", action_str)
        if not match:
            bare_action_match = re.fullmatch(r"(\w+)", action_str)
            if not bare_action_match:
                self.get_logger().error(
                    f"[ACTION] 动作格式无效: '{action_str}', "
                    f"期望格式: action_name(args)"
                )
                return False
            action_name = bare_action_match.group(1)
            args_str = ""
            self.get_logger().warn(
                f"[ACTION] 动作缺少括号，已按零参数动作兼容处理: "
                f"'{action_str}' -> '{action_name}()'"
            )
        else:
            action_name, args_str = match.groups()

        args = [arg.strip().strip("'\"") for arg in args_str.split(",") if arg.strip()]

        handler = self.action_registry.get(action_name)
        if handler:
            if self.debug_mode:
                self.get_logger().info(
                    f"[ACTION_SERVICE] 匹配处理器: {action_name}, 参数={args}"
                )
            t0 = time.time()
            result = handler(*args)
            elapsed = time.time() - t0
            if self.debug_mode:
                self.get_logger().info(
                    f"[ACTION_SERVICE] {action_name} 执行耗时: "
                    f"{elapsed:.2f}s, 结果: {'成功' if result else '失败'}"
                )
            return result

        available = list(self.action_registry.keys())
        self.get_logger().error(
            f"[ACTION] 未知动作: '{action_name}', 可用动作: {available}"
        )
        return False

    # ===== 节点级动作 handler（与节点状态强耦合）=====

    def execute_seewhat(self) -> bool:
        """保存当前画面，交由主反馈闭环上传给 Dify。"""
        if not has_camera(self.car_type):
            return self.reject_unsupported_action(
                "seewhat()", "当前车型不带摄像头，无法拍照或查看画面"
            )
        self.get_logger().info("[SEEWHAT] 开始采集当前画面")
        cv_image = self.get_latest_image_snapshot()
        if cv_image is None:
            self.get_logger().error("[SEEWHAT] 无可用图像: /camera/image_raw 无数据")
            return False

        try:
            image_path = self.build_feedback_image_path("llm_camera")
            saved = cv2.imwrite(
                image_path,
                cv_image,
                [cv2.IMWRITE_JPEG_QUALITY, 85],
            )
            if not saved:
                self.get_logger().error(f"[SEEWHAT] 图像保存失败: {image_path}")
                return False

            self.get_logger().info(
                f"[SEEWHAT] 图像已保存: {image_path} "
                f"({cv_image.shape[1]}x{cv_image.shape[0]})"
            )
            self.show_camera_preview(image_path)

            feedback_query = self.config.get("FEEDBACK_LOOP", {}).get(
                "image_query", "机器人图像反馈完成"
            )
            self.add_pending_action_feedback(
                feedback_query,
                image_paths=[image_path],
            )
            self.get_logger().info("[SEEWHAT] 图像已登记为 Dify 反馈")
            return True
        except Exception as error:
            self.get_logger().error(
                f"[SEEWHAT] 图像采集异常: {type(error).__name__}: {error}"
            )
            return False

    def execute_observe_surroundings(self) -> bool:
        """采集多个方向的图片，交由主反馈闭环一次性上传给 Dify。"""
        if not has_ptz(self.car_type):
            reason = (
                "当前车型不带摄像头，无法观察周围环境"
                if not has_camera(self.car_type)
                else "当前车型使用固定摄像头，没有可控制云台，无法多方向观察"
            )
            return self.reject_unsupported_action("observe_surroundings()", reason)
        self.get_logger().info("[OBSERVE] 开始观察周围环境")
        observe_config = self.config.get("OBSERVE_SURROUNDINGS", {})
        directions = observe_config.get(
            "directions",
            [
                {"name": "正前方", "horizontal": 0, "pitch": -45},
                {"name": "正右方", "horizontal": 90, "pitch": -45},
                {"name": "正左方", "horizontal": -90, "pitch": -45},
            ],
        )
        settle_time = max(
            1.0, min(2.0, float(observe_config.get("settle_time_sec", 1.5)))
        )
        frame_timeout = float(observe_config.get("frame_timeout_sec", 1.0))

        if self.get_latest_image_snapshot() is None:
            self.get_logger().error("[OBSERVE] 无可用图像")
            self.publish_progress("[观察] 无法获取摄像头图像")
            return False

        captured_images = []
        captured_directions = []
        try:
            for direction_index, direction in enumerate(directions):
                if self.should_interrupt():
                    self.get_logger().info("[OBSERVE] 被新指令中断")
                    return False

                direction_name = direction["name"]
                horizontal_angle = direction["horizontal"]
                pitch_angle = direction["pitch"]
                self.publish_progress(
                    f"[观察 {direction_index + 1}/{len(directions)}] "
                    f"正在观察{direction_name}..."
                )
                if not self.ptz.publish_servo_angles(
                    servo1_angle=horizontal_angle, servo2_angle=pitch_angle
                ):
                    continue
                time.sleep(settle_time)

                if self.should_interrupt():
                    return False

                try:
                    with self.latest_image_lock:
                        settled_at_sequence = self.latest_image_seq
                    cv_image = self.wait_for_image_after(
                        settled_at_sequence, frame_timeout
                    )
                    if cv_image is None:
                        self.publish_progress(f"[观察] {direction_name}: 等待新图像超时")
                        continue

                    cache_path = self.build_feedback_image_path(
                        "llm_observe", direction_index
                    )
                    saved = cv2.imwrite(
                        cache_path,
                        cv_image,
                        [cv2.IMWRITE_JPEG_QUALITY, 85],
                    )
                    if not saved:
                        self.publish_progress(f"[观察] {direction_name}: 图像保存失败")
                        continue

                    captured_images.append(cache_path)
                    captured_directions.append(direction_name)
                    self.publish_progress(
                        f"[观察] {direction_name}: 已获取对应方位的画面"
                    )
                except Exception as error:
                    self.get_logger().error(
                        f"[OBSERVE] {direction_name} 采集异常: "
                        f"{type(error).__name__}: {error}"
                    )

            if not captured_images:
                self.get_logger().error("[OBSERVE] 未成功保存任何方向的图像")
                return False

            direction_text = "、".join(captured_directions)
            feedback_query = (
                self.config.get("FEEDBACK_LOOP", {})
                .get(
                    "surroundings_query",
                    "机器人多方向图像反馈完成，图片顺序为：{directions}",
                )
                .format(directions=direction_text)
            )
            self.add_pending_action_feedback(
                feedback_query,
                image_paths=captured_images,
            )
            self.get_logger().info(
                f"[OBSERVE] 观察图像采集完成，共 {len(captured_images)} 个方向"
            )
            return True
        finally:
            if self.should_interrupt():
                self.get_logger().info("[OBSERVE] 已被新指令接管，跳过云台归位")
            else:
                self.get_logger().info("[OBSERVE] 观察结束，云台返回初始位置")
                if not self.ptz.execute_servo_init():
                    self.get_logger().warn("[OBSERVE] 云台归位失败")

    def execute_follow_line(self, color: str = "red") -> bool:
        """视觉巡线 - 根据颜色参数读取 HSV 并由本节点直接驱动底盘。"""
        if not has_camera(self.car_type):
            self.set_terminal_action_result_text(
                "当前车型不带摄像头，无法执行视觉巡线"
            )
            return self.reject_unsupported_action(
                "follow_line()", "当前车型不带摄像头，无法执行视觉巡线"
            )
        follow_config = self.config.get("FOLLOW_LINE", {})
        if not follow_config.get("enable", False):
            self.get_logger().error("[FOLLOW] 巡线功能在配置中已禁用")
            self.set_terminal_action_result_text("巡线功能在配置中已禁用")
            return False

        color_name = self.follow_line_controller.normalize_color(color)
        if color_name is None:
            self.get_logger().error(
                f"[FOLLOW] 颜色参数无效: {color}; 可选 r/g/b/y 或 "
                "red/green/blue/yellow"
            )
            self.set_terminal_action_result_text(
                "巡线颜色参数无效，请使用 red、green、blue 或 yellow"
            )
            return False

        self.get_logger().info(
            f"[FOLLOW] 开始巡线 (car_type={self.car_type}, color={color_name})"
        )
        if has_ptz(self.car_type):
            follow_camera_yaw = follow_config.get(
                "camera_servo1_angle", self.ptz.servo1_initial_angle
            )
            follow_camera_pitch = follow_config.get("camera_servo2_angle", -90.0)
            self.publish_progress("[巡线] 正在调整摄像头巡线角度...")
            if not self.ptz.publish_servo_angles(
                servo1_angle=follow_camera_yaw,
                servo2_angle=follow_camera_pitch,
            ):
                return False
            settle_time = float(
                follow_config.get(
                    "camera_settle_time_sec",
                    self.config.get("PTZ", {}).get("settle_time_sec", 0.3),
                )
            )
            time.sleep(max(0.0, settle_time))
            if self.should_interrupt():
                return False

        self.publish_progress(f"[巡线] 正在沿{color_name}色路线行走...")
        success = False
        try:
            success = self.follow_line_controller.execute(color_name)
        finally:
            self.execute_stop(recenter_camera=not self.should_interrupt())
        reason = self.follow_line_controller.finish_reason
        self.get_logger().info(f"[FOLLOW] 巡线结束, reason={reason}")
        if success:
            if reason == "line_lost":
                self.set_terminal_action_result_text("未识别到线路，巡线已停止")
            else:
                self.set_terminal_action_result_text("巡线已结束")
            self.publish_progress("[巡线] 巡线结束")
        elif reason in ("interrupted", "stop_requested"):
            self.publish_progress("[巡线] 巡线被新指令中断")
        else:
            if reason == "no_camera_frame":
                self.set_terminal_action_result_text("未收到摄像头图像，巡线已停止")
            else:
                self.set_terminal_action_result_text("巡线执行失败，任务已停止")
            self.publish_progress("[巡线] 巡线失败，已停止")
        return success

    def execute_stop(self, recenter_camera: bool = True) -> bool:
        """停止"""
        self.kcf.stop_active_kcf_tracking()
        self.follow_line_controller.stop()
        if self._parse_bool_flag(recenter_camera, default=True):
            self.ptz.recenter_after_stop()
        return True

    def execute_finish_task(self) -> bool:
        """任务完成（空操作，仅用于Dify纯对话场景）。"""
        self.get_logger().info("[ACTION] finishtask: 任务周期结束")

        # 会话重置标志，等待最终响应发布后再清空conversation_id。
        self._get_command_state().reset_conversation_after_response = True
        return True

    # ===== 抢占与取消 =====

    def _is_stop_user_command(self, user_text: str) -> bool:
        text = (user_text or "").strip().lower()
        return any(
            keyword in text
            for keyword in ("停止", "停车", "停下", "别动", "急停", "stop")
        )

    def _parse_bool_flag(self, value, default: bool = True) -> bool:
        if isinstance(value, bool):
            return value
        if value is None:
            return default
        text = str(value).strip().strip("'\"").lower()
        if text in ("0", "false", "no", "off", "不", "否", "不要", "关闭"):
            return False
        if text in ("1", "true", "yes", "on", "是", "要", "开启"):
            return True
        return default

    def cancel_current_action(self, recenter_camera: bool = False):
        """尽快取消当前正在执行的外部动作"""
        self.navigation.cancel_active_goal()
        self.follow_line_controller.stop()

        if self.follow_line_process and self.follow_line_process.poll() is None:
            try:
                kill_process_tree(self.follow_line_process.pid)
            except Exception as error:
                self.get_logger().warn(f"[ACTION] 终止巡线进程失败: {error}")

        self.execute_stop(recenter_camera=recenter_camera)

    def check_prerequisites(self) -> list:
        """检查前提条件"""
        issues = []

        # 检查相机
        if has_camera(self.car_type) and self.latest_image_msg is None:
            issues.append("相机未启动: /camera/image_raw 无数据")

        # 检查Nav2
        if self.navigation.enabled and not self.navigation.nav_client.wait_for_server(
            timeout_sec=1.0
        ):
            issues.append("导航未启动: /navigate_to_pose Action 不可用")

        # 检查TF树
        if self.navigation.enabled:
            try:
                self.navigation.tf_buffer.lookup_transform(
                    "map", "base_footprint", rclpy.time.Time()
                )
            except Exception:
                issues.append("TF树不完整: map→base_footprint 不可用")

        return issues

    # ===== 反馈队列 =====

    def _get_command_state(self) -> CommandState:
        """Return state for the current worker, or a non-worker fallback."""
        return getattr(self.worker_context, "state", self.fallback_command_state)

    def get_current_worker_command_id(self) -> int:
        """Return the current worker command id, or zero outside a worker."""
        return self._get_command_state().command_id

    def is_action_running(self) -> bool:
        """Return whether any command worker is currently in an action."""
        with self.active_action_lock:
            return bool(self.active_action_ids)

    def clear_pending_action_feedback(self):
        """清空当前动作批次产生的反馈。"""
        state = self._get_command_state()
        state.pending_feedback_messages.clear()
        state.pending_feedback_image_paths.clear()

    def add_pending_action_feedback(
        self, message: str, image_paths: list | None = None
    ):
        """登记动作反馈，主循环将在下一轮统一发送给 Dify。"""
        state = self._get_command_state()
        if message:
            state.pending_feedback_messages.append(str(message).strip())
        for image_path in image_paths or []:
            if image_path and image_path not in state.pending_feedback_image_paths:
                state.pending_feedback_image_paths.append(image_path)

    def consume_pending_action_feedback(self) -> tuple:
        """取出并清空当前动作反馈。"""
        state = self._get_command_state()
        message = "\n".join(item for item in state.pending_feedback_messages if item)
        image_paths = list(state.pending_feedback_image_paths)
        self.clear_pending_action_feedback()
        return message, image_paths

    def set_terminal_action_result_text(self, text: str) -> None:
        """Set the terminal action result for the current command only."""
        self._get_command_state().terminal_action_result_text = str(text or "")

    def get_terminal_action_result_text(self) -> str:
        """Return the terminal action result for the current command."""
        return self._get_command_state().terminal_action_result_text

    def build_action_feedback_query(self, actions: list, success: bool) -> str:
        """构造与 action_service_usb.py 一致的动作结果反馈文本。"""
        feedback_config = self.config.get("FEEDBACK_LOOP", {})
        return build_action_feedback(feedback_config, actions, success)

    def reject_unsupported_action(self, action: str, reason: str) -> bool:
        """Record a capability failure for the next Dify feedback round."""
        car_label = CAR_TYPE_LABELS[self.car_type]
        feedback = (
            f"机器人反馈:当前车型为{car_label}（CAR_TYPE={self.car_type}），"
            f"{reason}，动作{action}未执行。请向用户明确说明硬件限制，"
            "不要重试该动作，并结束当前任务。"
        )
        self.add_pending_action_feedback(feedback)
        self.get_logger().warn(f"[CAPABILITY] {feedback}")
        return False

    # ===== 动作分类辅助 =====

    def prepare_camera_for_actions(self, actions: list) -> bool:
        """按现有 PTZ 规则在动作批次开始前把相机复位到正前方。"""
        if not actions:
            return True
        if not has_ptz(self.car_type):
            return True
        if any(
            is_follow_line(action)
            or is_visual_follow(action)
            or is_servo_action(action)
            or is_stop_action(action)
            or action.strip().lower().startswith("observe_surroundings")
            for action in actions
        ):
            return True

        self.publish_progress("[准备] 正在将摄像头调整到正前方...")
        if not self.ptz.publish_servo_angles(
            servo1_angle=self.ptz.servo1_initial_angle,
            servo2_angle=self.ptz.servo2_initial_angle,
        ):
            return False
        settle_time = float(self.config.get("PTZ", {}).get("settle_time_sec", 0.3))
        time.sleep(settle_time)
        return not self.should_interrupt()

    def _should_end_feedback_loop_after_actions(self, actions: list) -> bool:
        """持续型视觉动作结束后不再进入下一轮 Dify 反馈规划。"""
        return any(is_visual_follow(action) for action in actions)

    def _build_terminal_action_result_text(self, actions: list, success: bool) -> str:
        action_descriptions = [describe_action(action) for action in actions]
        action_text = "、".join(action_descriptions) or "动作"
        if success:
            return f"{action_text}已结束"
        return f"{action_text}执行失败，任务已停止"

    # ===== 抢占与并发原语 =====

    def should_interrupt(self) -> bool:
        """判断当前工作线程是否已被更新的用户指令替代"""
        state = getattr(self.worker_context, "state", None)
        if state is None:
            return False

        with self.command_lock:
            return state.command_id != self.current_command_id

    def wait_event_or_interrupt(
        self, event: threading.Event, timeout: float, interval: float = 0.1
    ) -> bool:
        """等待事件，同时允许最新指令抢占当前动作"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if event.wait(timeout=interval):
                return True
            if self.should_interrupt():
                return False
        return event.is_set()

    # ===== 发布 =====

    def publish_decision_plan(self, actions: list):
        """打印决策层规划：把动作列表逐条转成中文描述分点输出。"""
        if not actions:
            return
        # 纯 finishtask（闲聊/无动作）不打印规划，避免冗余
        if all(is_finishtask(action) for action in actions):
            return

        plan_lines = [
            f"{index + 1}.{describe_action(action)}"
            for index, action in enumerate(actions)
        ]
        plan_text = "决策层规划：" + " ".join(plan_lines)
        self.publish_progress(plan_text)

    def publish_progress(self, text: str):
        """发布进度消息到终端（msg_type=progress）"""
        if self.should_interrupt():
            return
        msg = LlmTextResponse()
        msg.response_text = text
        msg.action_list_json = ""
        msg.conversation_id = ""
        msg.msg_type = "progress"
        self.text_pub.publish(msg)
        self.get_logger().info(f"[PROGRESS] {text}")

    def publish_latest_response(
        self, command_id: int, response_text: str, action_json: str, conv_id: str
    ):
        """只允许最新指令发布响应，避免旧导航完成后覆盖新结果。

        finishtask 的会话重置也在锁内原子完成，消除发布→重置之间的竞态窗口。
        """
        with self.command_lock:
            if command_id != self.current_command_id:
                self.get_logger().info(
                    f"[OUTPUT] 跳过过期响应: command_id={command_id}, "
                    f"latest={self.current_command_id}"
                )
                return

            self.publish_response(response_text, action_json, conv_id)

            # finishtask 会话重置在锁内原子完成，避免发布后、重置前新指令读到旧 ID
            state = self._get_command_state()
            if state.reset_conversation_after_response:
                if self.dify is not None:
                    self.dify.reset_conversation_id()
                state.reset_conversation_after_response = False

    def publish_current_response(
        self, response_text: str, action_json: str, conv_id: str
    ):
        """动作内部发布响应时，也遵守最新指令优先规则"""
        state = getattr(self.worker_context, "state", None)
        if state is None:
            self.publish_response(response_text, action_json, conv_id)
        else:
            self.publish_latest_response(
                state.command_id, response_text, action_json, conv_id
            )

    def publish_response(self, response_text: str, action_json: str, conv_id: str):
        """发布最终结果响应（msg_type=result）"""
        msg = LlmTextResponse()
        msg.response_text = response_text
        msg.action_list_json = action_json
        msg.conversation_id = conv_id
        msg.msg_type = "result"
        self.text_pub.publish(msg)
        self.get_logger().info(f"[RESULT] {response_text}")
        if self.debug_mode:
            self.get_logger().info(f"[OUTPUT] action_list_json: {action_json}")
            self.get_logger().info(f"[OUTPUT] conversation_id: {conv_id}")


def main(args=None):
    cv2.setNumThreads(1)
    rclpy.init(args=args)
    node = LlmAgentNode()
    # 本机 Jazzy 上 MultiThreadedExecutor 会空转并大量丢消息（实测 odom
    # 4 秒只收到 4/60 条、CPU 97%），改用单线程执行器；耗时动作都在
    # worker 线程里执行，不会阻塞回调。
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        try:
            if rclpy.ok():
                node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

