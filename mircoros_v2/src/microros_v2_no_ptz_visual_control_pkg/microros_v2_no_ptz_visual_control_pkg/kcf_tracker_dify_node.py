#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
kcf_tracker_dify_node.py

Dify 专用 KCF 追踪节点 - No-PTZ 版本（车身控制）
- 无 GUI 窗口，后台运行
- 支持通过 /start_track 服务传入 ROI 坐标
- 持续发布 /kcf_tracking_bbox，供 llm_agent_node 做小车跟随
- 兼容通过 ROI 参数 (auto_init_roi_x1/y1/x2/y2) 启动即自动初始化
- 默认只反馈坐标，不直接控制车身
- 专为 Dify 工作流优化
"""

from __future__ import annotations

import threading
import time
from typing import Optional, Tuple

import cv2 as cv
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from microros_v2_interface_pkg.srv import CancelTrack, StartTrack
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray


# ── KCF 追踪器包装 ───────────────────────────────────────────
class KcfTracker:
    """OpenCV KCF 追踪器包装"""

    def __init__(self, tracker_type: str = "KCF"):
        self._type = tracker_type.upper()
        self._tracker = None
        self._working = False
        self._create()

    def _create(self) -> None:
        t = self._type
        if t == "KCF":
            self._tracker = self._try_create("TrackerKCF_create")
        elif t == "CSRT":
            self._tracker = self._try_create("TrackerCSRT_create")
        elif t == "MOSSE":
            self._tracker = self._try_create("legacy.TrackerMOSSE_create")

        if self._tracker is None:
            self._tracker = self._try_create(f"legacy.Tracker{t}_create")
        if self._tracker is None:
            raise RuntimeError(f"Tracker {t} not available")

    @staticmethod
    def _try_create(path: str):
        try:
            parts = path.split(".")
            obj = cv
            for p in parts:
                obj = getattr(obj, p)
            return obj()
        except Exception:
            return None

    def init(self, frame: np.ndarray, roi: Tuple[int, int, int, int]) -> bool:
        try:
            ok = self._tracker.init(frame, roi)
            self._working = ok if ok is not None else True
            return self._working
        except Exception:
            self._working = False
            return False

    def update(self, frame: np.ndarray) -> Tuple[bool, Tuple[int, int, int, int]]:
        if not self._working:
            return False, (0, 0, 0, 0)
        try:
            ok, bbox = self._tracker.update(frame)
            if ok:
                self._working = True
                return True, tuple(int(v) for v in bbox)
            else:
                self._working = False
                return False, (0, 0, 0, 0)
        except Exception:
            self._working = False
            return False, (0, 0, 0, 0)

    @property
    def is_working(self) -> bool:
        return self._working


def clamp(v: float, lo: float, hi: float) -> float:
    return max(float(lo), min(float(hi), float(v)))


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


# ── Dify 专用 KCF 追踪节点 (No-PTZ) ──────────────────────────
class KcfTrackerDifyNode(Node):
    """
    Dify 专用 KCF 追踪节点 - No-PTZ 版本

    特点：
    - 无 GUI，纯后台运行
    - 必须提供 ROI 坐标参数
    - 启动即自动初始化
    - 仅控制车身角速度，保持目标在画面中心
    """

    def __init__(self) -> None:
        super().__init__("kcf_tracker_dify_node")

        self._img_lock = threading.Lock()
        self._tgt_lock = threading.Lock()

        self._latest_frame: Optional[np.ndarray] = None
        self._latest_seq = 0
        self._processed_seq = 0
        self._bridge = CvBridge()

        # 追踪器状态
        self._tracker: Optional[KcfTracker] = None
        self._tracker_ready = False
        self._auto_init_pending = False

        # 目标位置
        self._has_target = False
        self._target_x: Optional[float] = None
        self._target_y: Optional[float] = None

        # PD 控制（默认关闭，主控节点订阅 bbox 后统一控制）
        self._prev_err_x = 0.0
        self._prev_ctrl_t = 0.0

        # 追踪框（供可视化）
        self._bbox: Optional[Tuple[int, int, int, int]] = None

        # FPS 显示
        self._fps = 0.0
        self._fps_n = 0
        self._fps_t = time.perf_counter()

        # 声明和加载参数
        self._declare_params()
        self._load_params()

        auto_roi_valid = self._has_valid_roi_corners(
            self._auto_roi_x1,
            self._auto_roi_y1,
            self._auto_roi_x2,
            self._auto_roi_y2,
        )
        self._auto_init_pending = auto_roi_valid

        self._start_srv = self.create_service(
            StartTrack, "/start_track", self._handle_start_track)
        self._cancel_srv = self.create_service(
            CancelTrack, "/cancel_track", self._handle_cancel_track)

        # 订阅图像话题
        self._sub = self.create_subscription(
            Image,
            self._img_topic,
            self._on_image,
            QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE
            ),
        )

        # 发布车身速度控制话题
        self._pub = self.create_publisher(
            Twist,
            self._cmd_vel_topic,
            QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE
            ),
        )
        self._bbox_pub = self.create_publisher(
            Int32MultiArray, self._bbox_topic, 10)

        # 图像处理定时器
        self._img_timer = self.create_timer(
            1.0 / max(self._img_rate, 1.0),
            self._process_image,
        )

        # 控制定时器
        self._ctrl_timer = self.create_timer(
            1.0 / max(self._ctrl_rate, 1.0),
            self._ctrl_tick,
        )

        # 可视化窗口（show_debug_window 为真时开启，无交互，仅显示追踪结果）
        if self._show_win:
            cv.namedWindow(self._win_name, cv.WINDOW_NORMAL)
            cv.resizeWindow(self._win_name, self._win_w, self._win_h)

        roi_text = (
            f"参数ROI=({self._auto_roi_x1},{self._auto_roi_y1})-"
            f"({self._auto_roi_x2},{self._auto_roi_y2})，等待图像后自动追踪"
            if auto_roi_valid else "等待 /start_track 服务传入ROI"
        )
        self.get_logger().info(
            f"Dify KCF 追踪节点已启动 (No-PTZ) | {roi_text} | "
            f"窗口={'开' if self._show_win else '关'}")

    def _declare_params(self) -> None:
        """声明参数"""
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("tracking_bbox_topic", "/kcf_tracking_bbox")
        self.declare_parameter("enable_internal_control", False)
        # KCF/Dify ROI 坐标统一按 640x480 处理，不再按输入来源分辨率二次换算。
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("pd_kp", 0.005)
        self.declare_parameter("pd_kd", 0.002)
        self.declare_parameter("deadband_x", 30.0)
        self.declare_parameter("max_angular_speed", 0.8)
        self.declare_parameter("linear_speed", 0.0)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("image_processing_rate_hz", 15.0)
        self.declare_parameter("tracker_type", "KCF")
        # 可视化窗口（无鼠标/键盘交互，仅显示追踪结果）
        self.declare_parameter("show_debug_window", False)
        self.declare_parameter("window_name", "kcf_tracker_dify")
        self.declare_parameter("window_width", 640)
        self.declare_parameter("window_height", 480)
        self.declare_parameter("overlay_font_scale", 0.5)
        self.declare_parameter("overlay_line_height", 18)
        # Dify 必需参数
        self.declare_parameter("auto_init_roi_x1", -1)
        self.declare_parameter("auto_init_roi_y1", -1)
        self.declare_parameter("auto_init_roi_x2", -1)
        self.declare_parameter("auto_init_roi_y2", -1)
        # 兼容 yahboomcar 风格参数
        self.declare_parameter("x1", -1)
        self.declare_parameter("y1", -1)
        self.declare_parameter("x2", -1)
        self.declare_parameter("y2", -1)
        # PID 参数（兼容性）
        self.declare_parameter("linear_Kp", 5.0)
        self.declare_parameter("linear_Ki", 0.0)
        self.declare_parameter("linear_Kd", 0.5)
        self.declare_parameter("scale", 1000)
        self.declare_parameter("minDistance", 0.4)

    def _load_params(self) -> None:
        """加载参数"""
        self._img_topic = str(self.get_parameter("image_topic").value)
        self._cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._bbox_topic = str(self.get_parameter("tracking_bbox_topic").value)
        self._enable_internal_control = as_bool(
            self.get_parameter("enable_internal_control").value)
        self._img_w = int(self.get_parameter("image_width").value)
        self._img_h = int(self.get_parameter("image_height").value)
        self._kp = max(float(self.get_parameter("pd_kp").value), 0.0)
        self._kd = max(float(self.get_parameter("pd_kd").value), 0.0)
        self._db_x = float(self.get_parameter("deadband_x").value)
        self._max_ang = float(self.get_parameter("max_angular_speed").value)
        self._lin_spd = float(self.get_parameter("linear_speed").value)
        self._ctrl_rate = float(self.get_parameter("control_rate_hz").value)
        self._img_rate = float(self.get_parameter("image_processing_rate_hz").value)
        self._trk_type = str(self.get_parameter("tracker_type").value)
        # 可视化窗口参数
        self._show_win = bool(self.get_parameter("show_debug_window").value)
        self._win_name = str(self.get_parameter("window_name").value)
        self._win_w = int(self.get_parameter("window_width").value)
        self._win_h = int(self.get_parameter("window_height").value)
        self._font = float(self.get_parameter("overlay_font_scale").value)
        self._lh = int(self.get_parameter("overlay_line_height").value)
        # ROI 参数 - 优先使用 x1/y1/x2/y2（兼容），其次使用 auto_init_roi_*
        x1_compat = int(self.get_parameter("x1").value)
        y1_compat = int(self.get_parameter("y1").value)
        x2_compat = int(self.get_parameter("x2").value)
        y2_compat = int(self.get_parameter("y2").value)

        if x1_compat >= 0 and y1_compat >= 0 and x2_compat >= 0 and y2_compat >= 0:
            self._auto_roi_x1 = x1_compat
            self._auto_roi_y1 = y1_compat
            self._auto_roi_x2 = x2_compat
            self._auto_roi_y2 = y2_compat
        else:
            self._auto_roi_x1 = int(self.get_parameter("auto_init_roi_x1").value)
            self._auto_roi_y1 = int(self.get_parameter("auto_init_roi_y1").value)
            self._auto_roi_x2 = int(self.get_parameter("auto_init_roi_x2").value)
            self._auto_roi_y2 = int(self.get_parameter("auto_init_roi_y2").value)

        # PID 参数（保留兼容性，目前未使用但可供扩展）
        self._linear_Kp = float(self.get_parameter("linear_Kp").value)
        self._linear_Ki = float(self.get_parameter("linear_Ki").value)
        self._linear_Kd = float(self.get_parameter("linear_Kd").value)
        self._scale = int(self.get_parameter("scale").value)
        self._minDistance = float(self.get_parameter("minDistance").value)

    @staticmethod
    def _has_valid_roi_corners(x1: int, y1: int, x2: int, y2: int) -> bool:
        return x1 >= 0 and y1 >= 0 and x2 >= 0 and y2 >= 0

    def _normalize_roi(
        self,
        x1: int,
        y1: int,
        x2: int,
        y2: int,
        frame_width: int,
        frame_height: int,
    ) -> Tuple[int, int, int, int]:
        if not self._has_valid_roi_corners(x1, y1, x2, y2):
            raise ValueError(
                f"ROI 坐标必须全部 >= 0: ({x1},{y1})-({x2},{y2})")

        rx = min(x1, x2)
        ry = min(y1, y2)
        rw = abs(x2 - x1)
        rh = abs(y2 - y1)

        rx = int(clamp(rx, 0, frame_width - 1))
        ry = int(clamp(ry, 0, frame_height - 1))
        rw = int(min(rw, frame_width - rx))
        rh = int(min(rh, frame_height - ry))

        if rw < 10 or rh < 10:
            raise ValueError(f"ROI 太小或超出图像范围: w={rw}, h={rh}")
        return rx, ry, rw, rh

    def _handle_start_track(
            self, request: StartTrack.Request,
            response: StartTrack.Response) -> StartTrack.Response:
        with self._img_lock:
            frame = None if self._latest_frame is None else self._latest_frame.copy()

        if frame is None:
            response.success = False
            response.message = "尚未收到相机图像，无法启动KCF追踪"
            return response

        try:
            roi = self._normalize_roi(
                int(request.x1),
                int(request.y1),
                int(request.x2),
                int(request.y2),
                frame.shape[1],
                frame.shape[0],
            )
            self._init_tracker(frame, roi)
        except Exception as error:
            response.success = False
            response.message = f"KCF追踪启动失败: {error}"
            self.get_logger().error(response.message)
            return response

        rx, ry, rw, rh = roi
        response.success = True
        response.message = "KCF追踪已启动"
        response.selected_x1 = int(rx)
        response.selected_y1 = int(ry)
        response.selected_x2 = int(rx + rw)
        response.selected_y2 = int(ry + rh)
        self.get_logger().info(
            f"/start_track 成功 | 输入=({request.x1},{request.y1})-"
            f"({request.x2},{request.y2}) coord_frame={request.image_width}x"
            f"{request.image_height} | ROI=({response.selected_x1},"
            f"{response.selected_y1})-({response.selected_x2},"
            f"{response.selected_y2})")
        return response

    def _handle_cancel_track(
            self, request: CancelTrack.Request,
            response: CancelTrack.Response) -> CancelTrack.Response:
        self._tracker = None
        self._tracker_ready = False
        self._auto_init_pending = False
        with self._tgt_lock:
            self._has_target = False
            self._target_x = None
            self._target_y = None
        self._publish_bbox(False)

        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self._pub.publish(cmd)

        response.success = True
        response.message = "KCF追踪已取消"
        self.get_logger().info("/cancel_track 成功，已停止KCF追踪")
        return response

    def _on_image(self, msg: Image) -> None:
        """图像回调"""
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if frame is None:
                return
            if self._img_w > 0 and self._img_h > 0:
                frame = cv.resize(frame, (self._img_w, self._img_h))
            with self._img_lock:
                self._latest_frame = frame
                self._latest_seq += 1
        except Exception as e:
            self.get_logger().error(f"图像解码错误: {e}")

    def _publish_bbox(
            self,
            valid: bool,
            x: int = 0,
            y: int = 0,
            w: int = 0,
            h: int = 0) -> None:
        """Publish current KCF bbox for the main LLM agent follow controller.

        data = [valid, x1, y1, x2, y2, cx, cy, width, height, image_width, image_height]
        """
        msg = Int32MultiArray()
        if valid:
            cx = int(round(x + w / 2.0))
            cy = int(round(y + h / 2.0))
            msg.data = [
                1, int(x), int(y), int(x + w), int(y + h),
                cx, cy, int(w), int(h), int(self._img_w), int(self._img_h),
            ]
        else:
            msg.data = [
                0, 0, 0, 0, 0, 0, 0, 0, 0,
                int(self._img_w), int(self._img_h),
            ]
        self._bbox_pub.publish(msg)

    def _process_image(self) -> None:
        """图像处理 - 自动初始化 + 追踪更新"""
        with self._img_lock:
            if self._latest_frame is None:
                return
            seq = self._latest_seq

        if seq == self._processed_seq:
            return
        self._processed_seq = seq

        with self._img_lock:
            frame = self._latest_frame.copy()

        # 自动初始化追踪器（仅执行一次）
        if self._auto_init_pending:
            self._auto_init_pending = False
            self._init_tracker(frame)
            return

        # 追踪更新
        if self._tracker_ready and self._tracker is not None:
            ok, bbox = self._tracker.update(frame)
            if ok:
                x, y, w, h = bbox
                cx = x + w / 2.0
                cy = y + h / 2.0
                with self._tgt_lock:
                    self._has_target = True
                    self._target_x = float(cx)
                    self._target_y = float(cy)
                self._publish_bbox(True, x, y, w, h)
                # 可视化：画追踪框
                if self._show_win:
                    cv.rectangle(frame, (int(x), int(y)),
                                 (int(x + w), int(y + h)), (255, 0, 0), 2)
                    cv.circle(frame, (int(cx), int(y + h / 2.0)), 3, (0, 0, 255), -1)
            else:
                with self._tgt_lock:
                    self._has_target = False
                self._publish_bbox(False)
                self.get_logger().warn("追踪丢失目标")

        self._render_window(frame)

        # FPS 统计
        self._fps_n += 1
        tnow = time.perf_counter()
        if tnow - self._fps_t >= 1.0:
            self._fps = self._fps_n / (tnow - self._fps_t)
            self._fps_n = 0
            self._fps_t = tnow

    def _render_window(self, frame: np.ndarray) -> None:
        """可视化窗口（仅显示，无交互）。show_debug_window 为假时直接返回。"""
        if not self._show_win:
            return
        iw, ih = frame.shape[1], frame.shape[0]
        icx = iw // 2
        # 画面中心竖线 + 死区
        cv.line(frame, (icx, 0), (icx, ih), (0, 255, 255), 1)
        cv.line(frame, (icx - int(self._db_x), 0), (icx - int(self._db_x), ih), (0, 180, 255), 1)
        cv.line(frame, (icx + int(self._db_x), 0), (icx + int(self._db_x), ih), (0, 180, 255), 1)
        with self._tgt_lock:
            ok = self._has_target
            tx = self._target_x
        tgt_txt = f"Target x:{int(tx)}" if ok and tx is not None else "Target:None"
        lines = [
            f"KCF FPS:{int(self._fps)} {tgt_txt}",
            f"ROI:({self._auto_roi_x1},{self._auto_roi_y1})-({self._auto_roi_x2},{self._auto_roi_y2})",
        ]
        for i, t in enumerate(lines):
            cv.putText(frame, t, (8, 16 + i * self._lh), cv.FONT_HERSHEY_SIMPLEX,
                       self._font, (0, 255, 255), 1, cv.LINE_AA)
        cv.imshow(self._win_name, frame)
        cv.waitKey(1)

    def _init_tracker(
            self, frame: np.ndarray,
            roi: Optional[Tuple[int, int, int, int]] = None) -> None:
        """初始化追踪器"""
        if roi is None:
            roi = self._normalize_roi(
                self._auto_roi_x1,
                self._auto_roi_y1,
                self._auto_roi_x2,
                self._auto_roi_y2,
                frame.shape[1],
                frame.shape[0],
            )
        rx, ry, rw, rh = roi

        try:
            self._tracker = KcfTracker(self._trk_type)
            self._tracker.init(frame, roi)
            self._tracker_ready = True
            self._reset_pd()
            with self._tgt_lock:
                self._has_target = True
                self._target_x = float(rx + rw / 2.0)
                self._target_y = float(ry + rh / 2.0)
            self._publish_bbox(True, rx, ry, rw, rh)
            self.get_logger().info(f"追踪器初始化成功 | ROI={roi} | 开始追踪")
        except Exception as e:
            self.get_logger().error(f"追踪器初始化失败: {e}")
            self._tracker = None
            self._tracker_ready = False
            raise

    def _reset_pd(self) -> None:
        """重置 PD 控制器状态"""
        self._prev_err_x = 0.0
        self._prev_ctrl_t = 0.0

    def _ctrl_tick(self) -> None:
        """PD 控制循环 - 车身跟踪目标中心"""
        if not self._enable_internal_control:
            return
        if not self._tracker_ready:
            return

        with self._tgt_lock:
            tx, ok = self._target_x, self._has_target

        if not ok or tx is None:
            # 目标丢失，发送停止指令
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self._pub.publish(cmd)
            return

        # 计算归一化误差
        icx = self._img_w * 0.5
        pex = tx - icx

        # 死区
        if abs(pex) < self._db_x:
            pex = 0.0

        # 时间步长
        n = time.perf_counter()
        dt = n - self._prev_ctrl_t if self._prev_ctrl_t > 0.0 else 1.0 / max(self._ctrl_rate, 1.0)
        dt = max(dt, 1e-3)

        # 归一化误差 + 微分
        nex = pex / icx
        dx = (nex - self._prev_err_x) / dt

        # PD 控制计算角速度
        angular_z = clamp(
            -(self._kp * nex + self._kd * dx),
            -self._max_ang,
            self._max_ang
        )

        # 发布速度指令
        cmd = Twist()
        cmd.linear.x = self._lin_spd
        cmd.angular.z = angular_z
        self._pub.publish(cmd)

        # 更新状态
        self._prev_err_x = nex
        self._prev_ctrl_t = n


def main(args=None):
    rclpy.init(args=args)
    try:
        node = KcfTrackerDifyNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"节点错误: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
