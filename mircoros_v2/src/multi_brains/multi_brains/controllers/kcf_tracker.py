# -*- coding: utf-8 -*-
"""KCF coordinate tracking follow controller.

Owns the whole KCF lifecycle: discovering and subscribing to ``/tracking_bbox``,
calling ``/start_track`` / ``/cancel_track``, and the configured-rate PD follow
loop that keeps the target centered with either PTZ or chassis steering and
follows it with the chassis.

The tracker does not run OpenCV itself; it consumes bbox feedback published by
an external coordinate-tracking node and drives the PTZ controller + chassis
publisher accordingly. Control coefficients live in the ``TRACK_KCF`` config
block and are applied verbatim — do not retune during refactors.
"""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rosidl_runtime_py.utilities import get_message, get_service

from ..core.hardware_profile import has_camera, has_ptz
from .ptz_controller import clamp


class KcfTracker:
    """Start coordinate tracking and follow ``/tracking_bbox`` feedback."""

    def __init__(self, node, config, car_type: str, cmd_vel_pub, ptz):
        self.node = node
        self.car_type = car_type
        self.cmd_vel_pub = cmd_vel_pub
        self.ptz = ptz

        self._init_state()
        self._init_services(config)

    # ---- state ---------------------------------------------------------

    def _init_state(self) -> None:
        """Initialize KCF bbox feedback cache and follow-control thread state."""
        self.kcf_tracking_active = False
        self.kcf_tracking_lost_logged = False
        self.kcf_bbox_lock = threading.Lock()
        self.kcf_latest_bbox = {
            "valid": False,
            "stamp": 0.0,
            "x1": 0,
            "y1": 0,
            "x2": 0,
            "y2": 0,
            "cx": 0.0,
            "cy": 0.0,
            "width": 0.0,
            "height": 0.0,
            "image_width": 640,
            "image_height": 480,
            "confidence": 0.0,
        }
        self.kcf_follow_stop_event = threading.Event()
        self.kcf_follow_thread = None
        self.kcf_lifecycle_lock = threading.Lock()
        self.kcf_operation_cancel_event = None
        self.kcf_feedback_conversation_id = None
        self.kcf_prev_err_x = 0.0
        self.kcf_prev_err_y = 0.0
        self.kcf_prev_control_time = 0.0
        self.kcf_last_body_az = 0.0
        self.kcf_target_bbox_size = 0.0
        self.kcf_prev_distance_error = 0.0
        self.kcf_distance_integral = 0.0
        self.kcf_last_motion_log_time = 0.0
        self.kcf_body_correcting = False
        self.kcf_prev_body_error_eff = 0.0

    def _init_services(self, config) -> None:
        """Create start/cancel track clients and prepare the bbox subscription."""
        track_config = config.get("TRACK_KCF", {})
        self.start_track_service = track_config.get(
            "start_track_service", "/start_track"
        )
        self.cancel_track_service = track_config.get(
            "cancel_track_service", "/cancel_track"
        )
        self.start_track_service_type = track_config.get(
            "start_track_service_type", "smart_tracker/srv/StartTrack"
        )
        self.cancel_track_service_type = track_config.get(
            "cancel_track_service_type", "smart_tracker/srv/CancelTrack"
        )

        try:
            self.start_track_srv_type = get_service(self.start_track_service_type)
            self.start_track_client = self.node.create_client(
                self.start_track_srv_type, self.start_track_service
            )
        except Exception as error:
            self.start_track_srv_type = None
            self.start_track_client = None
            self.node.get_logger().error(
                f"[TRACK_KCF] 无法创建 {self.start_track_service} 客户端: "
                f"服务类型 {self.start_track_service_type} 加载失败: {error}"
            )
        try:
            self.cancel_track_srv_type = get_service(self.cancel_track_service_type)
            self.cancel_track_client = self.node.create_client(
                self.cancel_track_srv_type, self.cancel_track_service
            )
        except Exception as error:
            self.cancel_track_srv_type = None
            self.cancel_track_client = None
            self.node.get_logger().error(
                f"[TRACK_KCF] 无法创建 {self.cancel_track_service} 客户端: "
                f"服务类型 {self.cancel_track_service_type} 加载失败: {error}"
            )

        self.tracking_bbox_topic = track_config.get(
            "tracking_bbox_topic", "/tracking_bbox"
        )
        self.tracking_bbox_msg_type = track_config.get(
            "tracking_bbox_msg_type", "smart_tracker/msg/TrackBox"
        )
        self.tracking_bbox_confidence_threshold = float(
            track_config.get("tracking_confidence_threshold", 0.0)
        )
        self.tracking_bbox_sub = None
        self.ensure_tracking_bbox_subscription(timeout_sec=0.0, log_errors=False)

    # ---- bbox subscription ---------------------------------------------

    def _discover_topic_types(self, topic_name: str) -> list[str]:
        try:
            for name, types in self.node.get_topic_names_and_types():
                if name == topic_name:
                    return list(types)
        except Exception as error:
            self.node.get_logger().warn(
                f"[TRACK_KCF] 查询话题类型失败: {topic_name}: {error}"
            )
        return []

    def ensure_tracking_bbox_subscription(
        self, timeout_sec: float = 0.0, log_errors: bool = True
    ) -> bool:
        """Subscribe to ``/tracking_bbox``, discovering its real type when needed."""
        if self.tracking_bbox_sub is not None:
            return True

        deadline = time.time() + max(0.0, timeout_sec)
        last_error = ""
        tried_types = set()
        fallback_types = [
            self.tracking_bbox_msg_type,
            "smart_tracker/msg/TrackBox",
        ]

        while True:
            candidate_types = []
            discovered = self._discover_topic_types(self.tracking_bbox_topic)
            for type_name in fallback_types + discovered:
                if type_name and type_name not in tried_types:
                    tried_types.add(type_name)
                    candidate_types.append(type_name)

            for type_name in candidate_types:
                try:
                    msg_cls = get_message(type_name)
                    self.tracking_bbox_sub = self.node.create_subscription(
                        msg_cls,
                        self.tracking_bbox_topic,
                        self.tracking_bbox_callback,
                        10,
                    )
                    self.tracking_bbox_msg_type = type_name
                    self.node.get_logger().info(
                        f"[TRACK_KCF] 已订阅 {self.tracking_bbox_topic}, "
                        f"type={type_name}"
                    )
                    return True
                except Exception as error:
                    last_error = f"{type_name}: {error}"

            if time.time() >= deadline:
                break
            time.sleep(0.1)

        if log_errors:
            discovered = self._discover_topic_types(self.tracking_bbox_topic)
            self.node.get_logger().error(
                f"[TRACK_KCF] 无法订阅 {self.tracking_bbox_topic}; "
                f"配置类型={self.tracking_bbox_msg_type}; "
                f"ROS发现类型={discovered or '未发现'}; "
                f"最后错误={last_error or '无'}"
            )
        return False

    def tracking_bbox_callback(self, msg):
        """Cache target bbox feedback from ``/tracking_bbox``.

        Expected message fields:
        - timestamp: builtin_interfaces/Time
        - bbox: int16[4] = [x1, y1, x2, y2]
        - confidence: float32
        """
        bbox = list(msg.bbox)
        if len(bbox) != 4:
            self.node.get_logger().warn(
                f"[TRACK_KCF] 忽略格式无效的 /tracking_bbox: len={len(bbox)}"
            )
            return

        confidence = float(msg.confidence)
        track_config = self.node.config.get("TRACK_KCF", {})
        image_width = max(1, int(track_config.get("image_width", 640) or 640))
        image_height = max(1, int(track_config.get("image_height", 480) or 480))

        x1, y1, x2, y2 = [int(value) for value in bbox]
        x1, x2 = sorted((x1, x2))
        y1, y2 = sorted((y1, y2))
        width = max(0, x2 - x1)
        height = max(0, y2 - y1)
        valid = (
            confidence >= self.tracking_bbox_confidence_threshold
            and width > 0
            and height > 0
        )

        now = time.monotonic()
        with self.kcf_bbox_lock:
            self.kcf_latest_bbox.update(
                {
                    "valid": valid,
                    "stamp": now,
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2,
                    "cx": float(x1 + width / 2.0),
                    "cy": float(y1 + height / 2.0),
                    "width": float(width),
                    "height": float(height),
                    "image_width": image_width,
                    "image_height": image_height,
                    "confidence": confidence,
                }
            )

        if not valid and not self.kcf_tracking_lost_logged:
            self.node.get_logger().warn(
                f"[TRACK_KCF] /tracking_bbox 目标无效或置信度过低: "
                f"bbox={bbox}, confidence={confidence:.3f}"
            )
            self.kcf_tracking_lost_logged = True
        elif valid:
            self.kcf_tracking_lost_logged = False

    # ---- start tracking action -----------------------------------------

    def execute_track_kcf(
        self, x1: str = "-1", y1: str = "-1", x2: str = "-1", y2: str = "-1"
    ) -> bool:
        """Start coordinate tracking by ``/start_track`` and follow ``/tracking_bbox``."""
        if isinstance(x1, str) and x1.strip().lower() == "cancel":
            return self.cancel_tracking_tool()

        if not has_camera(self.car_type):
            return self.node.reject_unsupported_action(
                "track_kcf()", "当前车型不带摄像头，无法执行目标追踪"
            )
        track_config = self.node.config.get("TRACK_KCF", {})
        if not track_config.get("enable", False):
            self.node.get_logger().error("[TRACK_KCF] 坐标追踪功能在配置中已禁用")
            return False
        if self.start_track_client is None or self.start_track_srv_type is None:
            self.node.get_logger().error(
                f"[TRACK_KCF] {self.start_track_service} 客户端未初始化，"
                f"服务类型配置为 {self.start_track_service_type}"
            )
            return False

        track_image_width = int(track_config.get("image_width", 640) or 640)
        track_image_height = int(track_config.get("image_height", 480) or 480)
        if track_image_width <= 0 or track_image_height <= 0:
            self.node.get_logger().error(
                f"[TRACK_KCF] 配置的图像尺寸无效: "
                f"{track_image_width}x{track_image_height}"
            )
            return False

        try:
            roi_x1 = int(x1)
            roi_y1 = int(y1)
            roi_x2 = int(x2)
            roi_y2 = int(y2)
        except (TypeError, ValueError):
            self.node.get_logger().error(
                f"[TRACK_KCF] 坐标参数格式错误: x1={x1}, y1={y1}, x2={x2}, y2={y2}"
            )
            return False

        if (
            roi_x1 < 0
            or roi_y1 < 0
            or roi_x2 < 0
            or roi_y2 < 0
            or roi_x1 == roi_x2
            or roi_y1 == roi_y2
        ):
            self.node.get_logger().error(
                "[TRACK_KCF] 坐标无效: " f"({roi_x1},{roi_y1})-({roi_x2},{roi_y2})"
            )
            return False

        # 不再拉起 KCF 子进程；只调用已有 /start_track 服务并消费 /tracking_bbox。

        if self.kcf_tracking_active:
            self.stop_active_kcf_tracking()
            time.sleep(0.1)

        operation_cancel_event = threading.Event()
        with self.kcf_lifecycle_lock:
            self.kcf_operation_cancel_event = operation_cancel_event

        try:
            roi_x, roi_y, roi_w, roi_h = self._normalize_kcf_roi(
                roi_x1, roi_y1, roi_x2, roi_y2, track_image_width, track_image_height
            )
        except ValueError as error:
            self.node.get_logger().error(f"[TRACK_KCF] 坐标无效: {error}")
            self._finish_kcf_start(operation_cancel_event)
            return False

        if not self._wait_for_start_track_service(
            float(track_config.get("service_timeout_sec", 5.0))
        ):
            message = (
                f"{self.start_track_service} 服务不可用，无法启动坐标追踪；"
                "请确认坐标追踪节点已启动"
            )
            self.node.get_logger().error(f"[TRACK_KCF] {message}")
            self.node.set_terminal_action_result_text(message)
            self._finish_kcf_start(operation_cancel_event)
            return False

        request = self.start_track_srv_type.Request()
        self._set_request_field(request, "x1", int(roi_x))
        self._set_request_field(request, "y1", int(roi_y))
        self._set_request_field(request, "x2", int(roi_x + roi_w))
        self._set_request_field(request, "y2", int(roi_y + roi_h))
        self._set_request_field(request, "image_width", int(track_image_width))
        self._set_request_field(request, "image_height", int(track_image_height))

        self.node.get_logger().info(
            f"[TRACK_KCF] 调用 {self.start_track_service}: "
            f"ROI=({roi_x},{roi_y})-({roi_x + roi_w},{roi_y + roi_h}), "
            f"coord_frame={track_image_width}x{track_image_height}"
        )
        response = self._call_start_track_service(
            request,
            timeout_sec=float(track_config.get("start_timeout_sec", 5.0)),
            cancel_event=operation_cancel_event,
        )
        response_success = True if response is not None else False
        if response is not None and hasattr(response, "success"):
            response_success = bool(response.success)
        if not response_success:
            message = self._get_track_response_message(response)
            self.node.get_logger().error(
                f"[TRACK_KCF] {self.start_track_service} 启动失败: {message}"
            )
            self.node.set_terminal_action_result_text(f"坐标追踪启动失败: {message}")
            self._call_cancel_track_service(
                timeout_sec=self._get_cancel_track_timeout()
            )
            self._finish_kcf_start(operation_cancel_event)
            return False
        if operation_cancel_event.is_set() or self.node.should_interrupt():
            operation_cancel_event.set()
            self._call_cancel_track_service(
                timeout_sec=self._get_cancel_track_timeout()
            )
            self._finish_kcf_start(operation_cancel_event)
            self.node.get_logger().info("[TRACK_KCF] 启动响应已过期，已取消外部追踪")
            return False

        response_message = self._get_track_response_message(response)
        self.node.get_logger().info(
            f"[TRACK_KCF] {self.start_track_service} 成功: {response_message}, "
            f"selected_roi={self._format_track_response_roi(response, request)}"
        )

        if not self.ensure_tracking_bbox_subscription(
            timeout_sec=float(
                track_config.get("tracking_bbox_subscription_timeout_sec", 3.0)
            )
        ):
            message = (
                f"无法订阅 {self.tracking_bbox_topic}，无法根据坐标跟随；"
                "请确认该话题已发布并检查 tracking_bbox_msg_type 配置"
            )
            self._call_cancel_track_service(
                timeout_sec=self._get_cancel_track_timeout()
            )
            self.node.set_terminal_action_result_text(message)
            self._finish_kcf_start(operation_cancel_event)
            return False

        self._seed_kcf_bbox_from_response(
            response, request, track_image_width, track_image_height
        )
        with self.kcf_lifecycle_lock:
            if operation_cancel_event.is_set():
                activation_canceled = True
            else:
                activation_canceled = False
                self.kcf_feedback_conversation_id = self.node.dify.get_conversation_id()
                self._start_kcf_follow_thread(
                    response, request, track_image_width, track_image_height
                )
                self.kcf_tracking_active = True

        if activation_canceled or self.node.should_interrupt():
            operation_cancel_event.set()
            self.stop_active_kcf_tracking()
            self.node.get_logger().info("[TRACK_KCF] 跟随激活时指令已过期，追踪已停止")
            return False

        self.node.publish_progress(
            "[追踪] KCF 追踪已启动，实际框选坐标: "
            f"{self._format_track_response_roi(response, request)}"
        )
        control_mode = (
            "云台和底盘"
            if has_ptz(self.car_type)
            else "底盘转向和前后移动"
        )
        self.node.set_terminal_action_result_text(
            f"坐标追踪已启动；主程序会根据 /tracking_bbox 坐标持续控制{control_mode}，"
            "收到新指令或执行 stop/track_kcf(cancel) 时会取消追踪"
        )
        return True

    def _finish_kcf_start(self, operation_cancel_event) -> None:
        """Release a failed/canceled start operation without touching a newer one."""
        with self.kcf_lifecycle_lock:
            if self.kcf_operation_cancel_event is operation_cancel_event:
                self.kcf_operation_cancel_event = None

    # ---- ROI helpers ---------------------------------------------------

    @staticmethod
    def _normalize_kcf_roi(
        x1: int, y1: int, x2: int, y2: int, frame_width: int, frame_height: int
    ) -> tuple[int, int, int, int]:
        if x1 < 0 or y1 < 0 or x2 < 0 or y2 < 0:
            raise ValueError(f"ROI 坐标必须全部 >= 0: ({x1},{y1})-({x2},{y2})")
        rx = min(x1, x2)
        ry = min(y1, y2)
        rw = abs(x2 - x1)
        rh = abs(y2 - y1)
        rx = int(max(0, min(rx, frame_width - 1)))
        ry = int(max(0, min(ry, frame_height - 1)))
        rw = int(min(rw, frame_width - rx))
        rh = int(min(rh, frame_height - ry))
        if rw < 10 or rh < 10:
            raise ValueError(f"ROI 太小或超出图像范围: w={rw}, h={rh}")
        return rx, ry, rw, rh

    def _clear_kcf_bbox_state(self) -> None:
        """Invalidate cached bbox feedback without touching the external tracker node."""
        self.kcf_tracking_lost_logged = False
        now = time.monotonic()
        with self.kcf_bbox_lock:
            self.kcf_latest_bbox["valid"] = False
            self.kcf_latest_bbox["stamp"] = now
            self.kcf_latest_bbox["confidence"] = 0.0

    @staticmethod
    def _set_request_field(request, field_name: str, value) -> None:
        """Set a service request field only when that interface defines it."""
        if hasattr(request, field_name):
            setattr(request, field_name, value)

    @staticmethod
    def _get_track_coord(source, primary_name: str, fallback_name: str = "") -> int:
        if source is None:
            return 0
        if hasattr(source, primary_name):
            return int(getattr(source, primary_name))
        if fallback_name and hasattr(source, fallback_name):
            return int(getattr(source, fallback_name))
        return 0

    @staticmethod
    def _get_track_response_message(response) -> str:
        if response is None:
            return "无响应"
        if hasattr(response, "message"):
            message = str(response.message)
            return message if message else "服务已返回"
        return "服务已返回"

    def _get_cancel_track_timeout(self) -> float:
        track_config = self.node.config.get("TRACK_KCF", {})
        return max(0.1, float(track_config.get("cancel_timeout_sec", 2.0)))

    def _extract_track_roi(self, response, request) -> tuple[int, int, int, int]:
        x1 = self._get_track_coord(response, "selected_x1", "x1")
        y1 = self._get_track_coord(response, "selected_y1", "y1")
        x2 = self._get_track_coord(response, "selected_x2", "x2")
        y2 = self._get_track_coord(response, "selected_y2", "y2")
        if x1 == 0 and y1 == 0 and x2 == 0 and y2 == 0:
            x1 = self._get_track_coord(request, "x1")
            y1 = self._get_track_coord(request, "y1")
            x2 = self._get_track_coord(request, "x2")
            y2 = self._get_track_coord(request, "y2")
        x1, x2 = sorted((x1, x2))
        y1, y2 = sorted((y1, y2))
        return x1, y1, x2, y2

    def _extract_request_roi(self, request) -> tuple[int, int, int, int]:
        x1 = self._get_track_coord(request, "x1")
        y1 = self._get_track_coord(request, "y1")
        x2 = self._get_track_coord(request, "x2")
        y2 = self._get_track_coord(request, "y2")
        x1, x2 = sorted((x1, x2))
        y1, y2 = sorted((y1, y2))
        return x1, y1, x2, y2

    def _format_track_response_roi(self, response, request) -> str:
        x1, y1, x2, y2 = self._extract_track_roi(response, request)
        return f"({x1},{y1})-({x2},{y2})"

    @staticmethod
    def _get_kcf_bbox_size(width: float, height: float) -> float:
        return math.sqrt(max(1.0, float(width)) * max(1.0, float(height)))

    def _seed_kcf_bbox_from_response(
        self, response, request, image_width: int, image_height: int
    ) -> None:
        """Seed bbox cache from the accepted ROI before the first tracker feedback."""
        x1, y1, x2, y2 = self._extract_track_roi(response, request)
        width = max(1, int(x2 - x1))
        height = max(1, int(y2 - y1))
        cx = x1 + width / 2.0
        cy = y1 + height / 2.0
        req_x1, req_y1, req_x2, req_y2 = self._extract_request_roi(request)
        target_width = max(1, int(req_x2 - req_x1))
        target_height = max(1, int(req_y2 - req_y1))
        self.kcf_target_bbox_size = self._get_kcf_bbox_size(
            target_width, target_height
        )
        with self.kcf_bbox_lock:
            self.kcf_latest_bbox.update(
                {
                    "valid": True,
                    "stamp": time.monotonic(),
                    "x1": int(x1),
                    "y1": int(y1),
                    "x2": int(x2),
                    "y2": int(y2),
                    "cx": float(cx),
                    "cy": float(cy),
                    "width": float(width),
                    "height": float(height),
                    "image_width": max(1, int(image_width)),
                    "image_height": max(1, int(image_height)),
                    "confidence": 1.0,
                }
            )

    # ---- follow controller state ---------------------------------------

    def _reset_kcf_follow_controller(self) -> None:
        self.kcf_prev_err_x = 0.0
        self.kcf_prev_err_y = 0.0
        self.kcf_prev_control_time = 0.0
        self.kcf_last_body_az = 0.0
        self.kcf_prev_distance_error = 0.0
        self.kcf_distance_integral = 0.0
        self.kcf_body_correcting = False
        self.kcf_prev_body_error_eff = 0.0

    def _start_kcf_follow_thread(
        self, response, request, image_width: int, image_height: int
    ) -> None:
        """Start the main-agent KCF follow controller."""
        self._stop_kcf_follow_thread()
        self._reset_kcf_follow_controller()
        stop_event = threading.Event()
        thread = threading.Thread(
            target=self._kcf_follow_loop,
            args=(
                max(1, int(image_width)),
                max(1, int(image_height)),
                stop_event,
            ),
            daemon=True,
        )
        self.kcf_follow_stop_event = stop_event
        self.kcf_follow_thread = thread
        thread.start()
        x1, y1, x2, y2 = self._extract_track_roi(response, request)
        self.node.get_logger().info(
            "[TRACK_KCF] 主控跟随线程已启动，"
            f"target_bbox={max(1, x2 - x1)}x{max(1, y2 - y1)}px"
        )

    def _stop_kcf_follow_thread(self) -> None:
        """Stop the main-agent KCF follow controller thread."""
        if hasattr(self, "kcf_follow_stop_event") and self.kcf_follow_stop_event:
            self.kcf_follow_stop_event.set()
        thread = getattr(self, "kcf_follow_thread", None)
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        if self.kcf_follow_thread is thread:
            self.kcf_follow_thread = None

    def _get_kcf_follow_config(self) -> dict:
        track_config = self.node.config.get("TRACK_KCF", {})
        sign_x = clamp(
            float(track_config.get("sign_x", 1.0)),
            -1.0,
            1.0,
        )
        sign_y = clamp(
            float(track_config.get("sign_y", -1.0)),
            -1.0,
            1.0,
        )
        return {
            "rate_hz": max(1.0, float(track_config.get("follow_rate_hz", 20.0))),
            "stale_timeout_sec": max(
                0.1, float(track_config.get("target_stale_timeout_sec", 0.6))
            ),
            "low_confidence_stop_sec": max(
                0.0, float(track_config.get("low_confidence_stop_sec", 10.0))
            ),
            "motion_log_interval_sec": max(
                0.0, float(track_config.get("motion_log_interval_sec", 0.0))
            ),
            "pixel_tolerance": max(0.0, float(track_config.get("pixel_tolerance", 15.0))),
            "ptz_kp": max(0.0, float(track_config.get("kp_x", 0.6))),
            "ptz_kd": max(0.0, float(track_config.get("kd_x", 0.15))),
            "ptz_kp_y": max(0.0, float(track_config.get("kp_y", 0.6))),
            "ptz_kd_y": max(0.0, float(track_config.get("kd_y", 0.15))),
            "max_ptz_step": max(0.0, float(track_config.get("max_step_xy", 5.0))),
            "ptz_sign_x": sign_x,
            "ptz_sign_y": sign_y,
            "enable_body_turn_follow": bool(
                track_config.get("enable_body_turn_follow", True)
            ),
            "body_kp": max(0.0, float(track_config.get("body_kp", 0.03))),
            "body_kd": max(0.0, float(track_config.get("body_kd", 0.0))),
            "body_tolerance_deg": max(
                0.0,
                float(track_config.get("body_tolerance_deg", 3.0)),
            ),
            "servo1_engage_deg": max(
                1.0,
                float(track_config.get("servo1_engage_deg", 5.0)),
            ),
            "max_angular_z": max(
                0.0,
                float(track_config.get("max_angular_z", 0.6)),
            ),
            "body_dir": clamp(
                float(track_config.get("body_dir", -sign_x)),
                -1.0,
                1.0,
            ),
            "body_ff_gain": clamp(
                float(track_config.get("body_ff_gain", 1.0)), 0.0, 3.0
            ),
            "fixed_camera_pixel_tolerance": max(
                0.0,
                float(
                    track_config.get(
                        "fixed_camera_pixel_tolerance",
                        track_config.get("pixel_tolerance", 15.0),
                    )
                ),
            ),
            "fixed_camera_engage_px": max(
                1.0, float(track_config.get("fixed_camera_engage_px", 30.0))
            ),
            "fixed_camera_body_kp": max(
                0.0,
                float(
                    track_config.get(
                        "fixed_camera_body_kp", track_config.get("kp_x", 0.6)
                    )
                ),
            ),
            "fixed_camera_body_kd": max(
                0.0,
                float(
                    track_config.get(
                        "fixed_camera_body_kd", track_config.get("kd_x", 0.15)
                    )
                ),
            ),
            "fixed_camera_body_dir": clamp(
                float(
                    track_config.get(
                        "fixed_camera_body_dir",
                        track_config.get("body_dir", -sign_x),
                    )
                ),
                -1.0,
                1.0,
            ),
            "enable_distance_follow": bool(
                track_config.get("enable_distance_follow", True)
            ),
            "distance_tolerance_ratio": max(
                0.0, float(track_config.get("distance_tolerance_ratio", 0.06))
            ),
            "distance_kp": max(0.0, float(track_config.get("distance_kp", 0.18))),
            "distance_ki": max(0.0, float(track_config.get("distance_ki", 0.0))),
            "distance_kd": max(0.0, float(track_config.get("distance_kd", 0.02))),
            "distance_integral_limit": max(
                0.0, float(track_config.get("distance_integral_limit", 0.5))
            ),
            "max_linear_x": max(0.0, float(track_config.get("max_linear_x", 0.12))),
            "linear_dir": clamp(
                float(track_config.get("linear_dir", 1.0)),
                -1.0,
                1.0,
            ),
        }

    def _read_kcf_bbox_state(self) -> dict:
        with self.kcf_bbox_lock:
            return dict(self.kcf_latest_bbox)

    def _publish_kcf_stop_twist(self) -> None:
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self._log_kcf_motion_state(twist)

    def stop_active_kcf_tracking(
        self, timeout_sec: float | None = None, publish_stop_twist: bool = True
    ) -> bool:
        """Stop local KCF follow control and request external tracker cancellation."""
        with self.kcf_lifecycle_lock:
            operation_cancel_event = self.kcf_operation_cancel_event
            if operation_cancel_event is not None:
                operation_cancel_event.set()
            was_active = self.kcf_tracking_active
            self.kcf_tracking_active = False
            self.kcf_operation_cancel_event = None
            self.kcf_feedback_conversation_id = None

        if was_active or operation_cancel_event is not None:
            self._stop_kcf_follow_thread()
            if publish_stop_twist:
                self._publish_kcf_stop_twist()
            cancel_ok = self._call_cancel_track_service(
                timeout_sec=(
                    self._get_cancel_track_timeout()
                    if timeout_sec is None
                    else timeout_sec
                )
            )
            self._clear_kcf_bbox_state()
            return cancel_ok

        if publish_stop_twist:
            self._publish_kcf_stop_twist()
        return True

    # ---- follow control (PD) -------------------------------------------

    def _apply_kcf_ptz_follow(self, state: dict, cfg: dict, dt: float) -> None:
        """VOR-first PTZ tracking — matches visual_track.py reference."""
        if not has_ptz(self.car_type):
            return

        # ---- VOR (body-FF) first: compensate chassis yaw on PTZ angle ----
        if abs(self.kcf_last_body_az) > 1e-6:
            body_yaw_deg = math.degrees(self.kcf_last_body_az * dt)
            vor_deg = cfg["body_ff_gain"] * cfg["ptz_sign_x"] * body_yaw_deg
            self.ptz.servo1_angle = clamp(
                self.ptz.servo1_angle + vor_deg,
                self.ptz.servo1_min_angle,
                self.ptz.servo1_max_angle,
            )

        image_width = max(1.0, float(state["image_width"]))
        image_height = max(1.0, float(state["image_height"]))
        half_w = image_width * 0.5
        half_h = image_height * 0.5

        # ---- servo1 (x / pan) -------------------------------------------
        err_x = float(state["cx"]) - half_w
        if abs(err_x) <= cfg["pixel_tolerance"]:
            self.kcf_prev_err_x = 0.0
            step_x = 0.0
        else:
            norm_x = clamp(err_x / half_w, -1.0, 1.0)
            deriv_x = norm_x - self.kcf_prev_err_x
            out_x = clamp(
                cfg["ptz_kp"] * norm_x + cfg["ptz_kd"] * deriv_x, -1.0, 1.0
            )
            step_x = out_x * cfg["max_ptz_step"]
            self.kcf_prev_err_x = norm_x

        # ---- servo2 (y / tilt) ------------------------------------------
        err_y = float(state["cy"]) - half_h
        if abs(err_y) <= cfg["pixel_tolerance"]:
            self.kcf_prev_err_y = 0.0
            step_y = 0.0
        else:
            norm_y = clamp(err_y / half_h, -1.0, 1.0)
            deriv_y = norm_y - self.kcf_prev_err_y
            out_y = clamp(
                cfg["ptz_kp_y"] * norm_y + cfg["ptz_kd_y"] * deriv_y, -1.0, 1.0
            )
            step_y = out_y * cfg["max_ptz_step"]
            self.kcf_prev_err_y = norm_y

        next_servo1 = clamp(
            self.ptz.servo1_angle + cfg["ptz_sign_x"] * step_x,
            self.ptz.servo1_min_angle, self.ptz.servo1_max_angle,
        )
        next_servo2 = clamp(
            self.ptz.servo2_angle + cfg["ptz_sign_y"] * step_y,
            self.ptz.servo2_min_angle, self.ptz.servo2_max_angle,
        )

        self.ptz.publish_servo_angles(
            servo1_angle=next_servo1, servo2_angle=next_servo2, repeat=False,
        )

    def _get_kcf_pan_error_deg(self) -> float:
        if not has_ptz(self.car_type):
            return 0.0
        initial_angle = float(getattr(self.ptz, "servo1_initial_angle", 0.0))
        servo_angle = float(getattr(self.ptz, "servo1_angle", initial_angle))
        return servo_angle - initial_angle

    def _log_kcf_motion_state(self, twist: Twist, cfg: dict | None = None) -> None:
        if cfg is None:
            cfg = self._get_kcf_follow_config()
        log_interval_sec = float(cfg.get("motion_log_interval_sec", 0.0))
        if log_interval_sec <= 0.0:
            return

        now = time.monotonic()
        last_log_time = float(getattr(self, "kcf_last_motion_log_time", 0.0))
        if now - last_log_time < log_interval_sec:
            return
        self.kcf_last_motion_log_time = now
        linear = getattr(twist, "linear", None)
        angular = getattr(twist, "angular", None)
        linear_x = float(getattr(linear, "x", 0.0))
        angular_z = float(getattr(angular, "z", 0.0))
        servo1_angle = float(getattr(self.ptz, "servo1_angle", 0.0))
        self.node.get_logger().info(
            "[TRACK_KCF][CMD] "
            f"servo1={servo1_angle:.2f}deg, x={linear_x:.3f}, z={angular_z:.3f}"
        )

    def _auto_stop_kcf_tracking_from_follow_loop(self, reason: str) -> None:
        """Stop tracking from the follow thread without joining the current thread."""
        with self.kcf_lifecycle_lock:
            if not self.kcf_tracking_active:
                return
            self.kcf_tracking_active = False
            self.kcf_operation_cancel_event = None
            feedback_conversation_id = self.kcf_feedback_conversation_id
            self.kcf_feedback_conversation_id = None

        self.node.get_logger().warn(f"[TRACK_KCF] {reason}，自动取消追踪")
        self.node.set_terminal_action_result_text(
            "目标丢失或置信度过低，KCF 跟随已自动停止"
        )
        self.node.publish_progress("[追踪] 目标丢失或置信度过低，已自动停止")
        self._publish_kcf_stop_twist()
        self._call_cancel_track_service(timeout_sec=self._get_cancel_track_timeout())
        self._clear_kcf_bbox_state()
        self._notify_dify_kcf_auto_stop(reason, feedback_conversation_id)

    def _notify_dify_kcf_auto_stop(
        self, reason: str, conversation_id: str | None
    ) -> None:
        """Send an asynchronous tracking-stop feedback turn to Dify."""
        if not conversation_id:
            self.node.get_logger().warn(
                "[TRACK_KCF] 自动停止事件没有可用 Dify conversation_id，无法回传给 Dify"
            )
            return

        feedback = (
            '机器人反馈:执行["track_kcf"]失败，原因：'
            f"{reason}。KCF 跟随已自动停止，请向用户说明目标丢失或置信度过低，"
            "并结束当前任务。"
        )
        self.node.get_logger().info(
            f"[DIFY] >>> KCF自动停止反馈: conversation_id={conversation_id}, "
            f"query='{feedback}'"
        )
        try:
            dify_response = self.node.dify.call_agent(
                feedback,
                robot_feedback=True,
                conversation_id=conversation_id,
                update_conversation_id=False,
            )
            if not dify_response:
                self.node.get_logger().warn("[DIFY] KCF自动停止反馈发送失败")
                return

            actions, response_text = self.node.dify.parse_response(dify_response)
            if response_text:
                self.node.get_logger().info(
                    f"[DIFY] KCF自动停止反馈响应: {response_text}"
                )
            if actions:
                self.node.get_logger().warn(
                    "[DIFY] KCF自动停止反馈返回了动作，后台事件不执行动作: "
                    f"{actions}"
                )
        except Exception as error:
            self.node.get_logger().warn(
                f"[DIFY] KCF自动停止反馈异常: {type(error).__name__}: {error}"
            )

    def _compute_kcf_body_turn(self, cfg: dict, dt: float) -> float:
        """Body follow — matches visual_track.py reference.

        Hysteresis on |servo1|: arm above engage_deg, disarm below stop_deg.
        PD on servo1 angle pulls it back toward 0 (car faces target).
        VOR (applied first in ``_apply_kcf_ptz_follow``) keeps the image
        stable while the chassis turns.
        """
        servo_err = self._get_kcf_pan_error_deg()

        if abs(servo_err) >= cfg["servo1_engage_deg"]:
            self.kcf_body_correcting = True
        elif abs(servo_err) <= cfg["body_tolerance_deg"]:
            self.kcf_body_correcting = False

        if not self.kcf_body_correcting:
            self.kcf_prev_body_error_eff = 0.0
            return 0.0

        deriv = (servo_err - self.kcf_prev_body_error_eff) / max(dt, 1e-3)
        angular_z = cfg["body_dir"] * (
            cfg["body_kp"] * servo_err + cfg["body_kd"] * deriv
        )
        angular_z = clamp(angular_z, -cfg["max_angular_z"], cfg["max_angular_z"])
        self.kcf_prev_body_error_eff = servo_err
        return angular_z

    def _compute_kcf_fixed_camera_body_turn(
        self, state: dict, cfg: dict
    ) -> float:
        """Turn a fixed-camera chassis directly from horizontal bbox error."""
        image_width = max(1.0, float(state["image_width"]))
        half_width = image_width * 0.5
        pixel_error = float(state["cx"]) - half_width

        if abs(pixel_error) <= cfg["fixed_camera_pixel_tolerance"]:
            self.kcf_body_correcting = False
        elif abs(pixel_error) >= cfg["fixed_camera_engage_px"]:
            self.kcf_body_correcting = True

        if not self.kcf_body_correcting:
            self.kcf_prev_body_error_eff = 0.0
            return 0.0

        normalized_error = clamp(pixel_error / half_width, -1.0, 1.0)
        derivative = normalized_error - self.kcf_prev_body_error_eff
        angular_z = cfg["fixed_camera_body_dir"] * (
            cfg["fixed_camera_body_kp"] * normalized_error
            + cfg["fixed_camera_body_kd"] * derivative
        )
        self.kcf_prev_body_error_eff = normalized_error
        return clamp(angular_z, -cfg["max_angular_z"], cfg["max_angular_z"])

    def _compute_kcf_distance_follow(
        self, state: dict, cfg: dict, dt: float
    ) -> float:
        if not cfg["enable_distance_follow"]:
            self.kcf_prev_distance_error = 0.0
            self.kcf_distance_integral = 0.0
            return 0.0

        target_size = max(1.0, float(self.kcf_target_bbox_size))
        current_size = self._get_kcf_bbox_size(state["width"], state["height"])
        size_error = (target_size - current_size) / target_size
        if abs(size_error) <= cfg["distance_tolerance_ratio"]:
            self.kcf_prev_distance_error = 0.0
            self.kcf_distance_integral = 0.0
            return 0.0

        dt = max(dt, 1e-3)
        self.kcf_distance_integral += size_error * dt
        integral_limit = cfg["distance_integral_limit"]
        if integral_limit > 0.0:
            self.kcf_distance_integral = clamp(
                self.kcf_distance_integral, -integral_limit, integral_limit
            )

        derivative = (size_error - self.kcf_prev_distance_error) / dt
        linear_x = cfg["linear_dir"] * (
            cfg["distance_kp"] * size_error
            + cfg["distance_ki"] * self.kcf_distance_integral
            + cfg["distance_kd"] * derivative
        )
        self.kcf_prev_distance_error = size_error
        return clamp(linear_x, -cfg["max_linear_x"], cfg["max_linear_x"])

    def _compute_kcf_chassis_twist(
        self, state: dict, cfg: dict, dt: float
    ) -> Twist:
        twist = Twist()

        if cfg["enable_body_turn_follow"]:
            if has_ptz(self.car_type):
                twist.angular.z = self._compute_kcf_body_turn(cfg, dt)
            else:
                twist.angular.z = self._compute_kcf_fixed_camera_body_turn(
                    state, cfg
                )
            self.kcf_last_body_az = float(twist.angular.z)
        else:
            self.kcf_last_body_az = 0.0

        twist.linear.x = self._compute_kcf_distance_follow(state, cfg, dt)
        return twist

    def _kcf_follow_loop(
        self, image_width: int, image_height: int, stop_event: threading.Event
    ) -> None:
        """Follow KCF bbox feedback with PTZ and chassis controls."""
        cfg = self._get_kcf_follow_config()
        period = 1.0 / cfg["rate_hz"]
        body_mode = (
            "vor+hysteresis"
            if has_ptz(self.car_type)
            else "fixed-camera-pixel-error"
        )
        body_thresholds = (
            f"engage={cfg['servo1_engage_deg']:.0f}deg "
            f"stop={cfg['body_tolerance_deg']:.0f}deg"
            if has_ptz(self.car_type)
            else f"engage={cfg['fixed_camera_engage_px']:.0f}px "
            f"stop={cfg['fixed_camera_pixel_tolerance']:.0f}px"
        )
        body_direction = (
            cfg["body_dir"]
            if has_ptz(self.car_type)
            else cfg["fixed_camera_body_dir"]
        )
        vor_mode = f"{cfg['body_ff_gain']:.2f}" if has_ptz(self.car_type) else "off"
        self.node.get_logger().info(
            f"[TRACK_KCF] 跟随控制参数: bbox_topic={self.tracking_bbox_topic}, "
            f"rate={cfg['rate_hz']:.1f}Hz, "
            f"body_turn={cfg['enable_body_turn_follow']}, "
            f"distance_follow={cfg['enable_distance_follow']}, "
            "confidence_threshold="
            f"{self.tracking_bbox_confidence_threshold:.3f}, "
            f"stale_timeout={cfg['stale_timeout_sec']:.1f}s, "
            f"auto_stop={cfg['low_confidence_stop_sec']:.1f}s, "
            f"body_mode={body_mode}, {body_thresholds} "
            f"max_az={cfg['max_angular_z']:.2f}, "
            f"max_x={cfg['max_linear_x']:.2f}, "
            f"body_dir={body_direction:.0f}, "
            f"vor={vor_mode}"
        )

        target_lost_since = None
        while not stop_event.is_set() and rclpy.ok():
            state = self._read_kcf_bbox_state()
            now = time.monotonic()
            fresh = (now - float(state["stamp"])) <= cfg["stale_timeout_sec"]
            if not state["valid"] or not fresh:
                self._publish_kcf_stop_twist()
                self._reset_kcf_follow_controller()
                if target_lost_since is None:
                    target_lost_since = now
                low_confidence_stop_sec = cfg["low_confidence_stop_sec"]
                if (
                    low_confidence_stop_sec > 0.0
                    and now - target_lost_since >= low_confidence_stop_sec
                ):
                    confidence = float(state.get("confidence", 0.0))
                    reason = (
                        f"/tracking_bbox 连续无有效目标超过 "
                        f"{low_confidence_stop_sec:.1f}s "
                        f"(valid={state['valid']}, fresh={fresh}, "
                        f"confidence={confidence:.3f}, "
                        f"threshold={self.tracking_bbox_confidence_threshold:.3f})"
                    )
                    self._auto_stop_kcf_tracking_from_follow_loop(reason)
                    stop_event.set()
                    break
                time.sleep(period)
                continue
            target_lost_since = None

            if state["image_width"] <= 0:
                state["image_width"] = image_width
            if state["image_height"] <= 0:
                state["image_height"] = image_height

            dt = (
                now - self.kcf_prev_control_time
                if self.kcf_prev_control_time > 0.0
                else period
            )
            dt = max(dt, 1e-3)

            self._apply_kcf_ptz_follow(state, cfg, dt)
            twist = self._compute_kcf_chassis_twist(state, cfg, dt)
            self.cmd_vel_pub.publish(twist)
            self._log_kcf_motion_state(twist, cfg)

            self.kcf_prev_control_time = now
            time.sleep(period)

        self._publish_kcf_stop_twist()

    # ---- service calls -------------------------------------------------

    def _wait_for_start_track_service(self, timeout_sec: float) -> bool:
        """Wait for the external coordinate-tracking service."""
        deadline = time.time() + max(timeout_sec, 0.1)
        while time.time() < deadline:
            if self.start_track_client.wait_for_service(timeout_sec=0.2):
                return True
            if self.node.should_interrupt():
                return False
        discovered = self._describe_discovered_service(self.start_track_service)
        self.node.get_logger().warn(
            f"[TRACK_KCF] 等待 {self.start_track_service} 超时；"
            f"期望类型={self.start_track_service_type}；ROS发现={discovered}"
        )
        return self.start_track_client.service_is_ready()

    def _describe_discovered_service(self, service_name: str) -> str:
        """Return discovered ROS service types for diagnostics."""
        try:
            for name, types in self.node.get_service_names_and_types():
                if name == service_name:
                    return ",".join(types) if types else "无类型信息"
        except Exception as error:
            return f"查询失败: {error}"
        return "未发现该服务名"

    def _call_start_track_service(
        self, request, timeout_sec: float, cancel_event: threading.Event | None = None
    ):
        """Call ``/start_track`` and wait briefly for the accepted ROI response."""
        try:
            future = self.start_track_client.call_async(request)
        except Exception as error:
            self.node.get_logger().error(
                f"[TRACK_KCF] {self.start_track_service} 调用异常: {error}"
            )
            return None

        deadline = time.time() + max(timeout_sec, 0.1)
        while not future.done() and time.time() < deadline:
            if self.node.should_interrupt() or (
                cancel_event is not None and cancel_event.is_set()
            ):
                self._call_cancel_track_service(
                    timeout_sec=self._get_cancel_track_timeout()
                )
                self.node.get_logger().info("[TRACK_KCF] 坐标追踪启动被中断")
                return None
            time.sleep(0.02)

        if not future.done():
            self.node.get_logger().warn(
                f"[TRACK_KCF] {self.start_track_service} 调用超时"
            )
            return None

        try:
            return future.result()
        except Exception as error:
            self.node.get_logger().error(
                f"[TRACK_KCF] {self.start_track_service} 响应异常: {error}"
            )
            return None

    def cancel_tracking_tool(self) -> bool:
        """Cancel KCF tracking and stop residual chassis motion."""
        self.stop_active_kcf_tracking()
        self.ptz.recenter_after_stop()
        self.node.set_terminal_action_result_text("KCF 追踪已取消")
        self.node.publish_progress("[追踪] KCF 追踪已取消")
        return True

    def _call_cancel_track_service(self, timeout_sec: float = 1.0) -> bool:
        """Request external coordinate tracking cancellation when available."""
        try:
            if self.cancel_track_client is None or self.cancel_track_srv_type is None:
                return False
            if not self.cancel_track_client.wait_for_service(timeout_sec=0.05):
                return False
            future = self.cancel_track_client.call_async(
                self.cancel_track_srv_type.Request()
            )
            deadline = time.time() + max(timeout_sec, 0.0)
            while not future.done() and time.time() < deadline:
                time.sleep(0.02)
            if not future.done():
                self.node.get_logger().warn(
                    f"[TRACK_KCF] {self.cancel_track_service} 响应超时 "
                    f"({timeout_sec:.1f}s)，本地已停止跟随线程并发布零速度"
                )
                return False
            response = future.result()
            response_success = True if response is not None else False
            if response is not None and hasattr(response, "success"):
                response_success = bool(response.success)
            if response_success:
                self.node.get_logger().info(
                    f"[TRACK_KCF] {self._get_track_response_message(response)}"
                )
                return True
            message = self._get_track_response_message(response)
            self.node.get_logger().warn(
                f"[TRACK_KCF] {self.cancel_track_service} 失败: {message}"
            )
            return False
        except Exception as error:
            self.node.get_logger().warn(
                f"[TRACK_KCF] {self.cancel_track_service} 调用异常: {error}"
            )
            return False

