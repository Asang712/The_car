#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
qr_follow_ptz_node.py

中文说明:
    ROS2 Humble 二维码跟随节点（云台 + 车身）。
    二维码检测保留原有实现，云台和车身控制与
    tracking_bbox_follow_node 使用同一套 KCF 风格控制器。

    距离跟随使用 KCF 节点相同的外接框像素尺寸代理量：
    current_qr_size_px = sqrt(width * height)
"""

from __future__ import annotations

import math
import os
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from .image_conversion import image_to_bgr
from .kcf_style_follow_control import (
    CONTROL_DYNAMIC_PARAMETER_NAMES,
    CONTROL_RESTART_REQUIRED_PARAMETER_NAMES,
    KcfStyleFollowController,
)

try:
    import pyzbar.pyzbar as pyzbar_module
except Exception:
    pyzbar_module = None


DEFAULT_NODE_NAME = "qr_follow_ptz_node"

STATE_IDLE = "idle"
STATE_TRACKING = "tracking"

KEY_CODE_SPACE = 32
KEY_CODE_ESCAPE = 27
MINIMUM_TIMER_RATE_HZ = 1.0
QR_CANDIDATE_COLOR = (0, 255, 0)
QR_PRIMARY_COLOR = (255, 0, 255)


@dataclass
class QrTarget:
    found: bool
    payload: str = ""
    rect: Tuple[int, int, int, int] = (0, 0, 0, 0)
    polygon: Tuple[Tuple[int, int], ...] = tuple()
    center: Tuple[float, float] = (0.0, 0.0)


DYNAMIC_PARAMETER_NAMES = CONTROL_DYNAMIC_PARAMETER_NAMES | frozenset({
    "show_debug_window",
    "image_width", "image_height", "window_width", "window_height",
    "overlay_font_scale", "overlay_line_height",
    "overlay_margin_x", "overlay_margin_y", "overlay_show_background",
    "enable_horizontal_flip", "enable_vertical_flip", "rotate_image_code",
    "image_processing_rate_hz", "lost_timeout", "follow_stable_frames",
    "lost_frame_threshold",
    "qr_detector_backend", "qr_select_strategy",
})

RESTART_REQUIRED_PARAMETER_NAMES = CONTROL_RESTART_REQUIRED_PARAMETER_NAMES | frozenset(
    {"image_topic", "window_name"}
)


def to_bool(v: Any, default: bool) -> bool:
    if isinstance(v, bool):
        return v
    if isinstance(v, str):
        n = v.strip().lower()
        if n in ("true", "1", "yes", "y", "on"):
            return True
        if n in ("false", "0", "no", "n", "off"):
            return False
    if isinstance(v, (int, float)):
        return bool(v)
    return default


class QrFollowPtzNode(Node):

    def __init__(self) -> None:
        super().__init__(DEFAULT_NODE_NAME)
        self._tgt_lock = threading.Lock()
        self._image_lock = threading.Lock()
        self._latest_image_msg: Optional[Image] = None
        self._latest_image_seq = 0
        self._processed_image_seq = 0
        self._latest_frame: Optional[np.ndarray] = None

        self._state = STATE_TRACKING
        self._shutdown = False

        self._has_target = False
        self._lost_frames = 0
        self._stable_track_frames = 0
        self._last_target_time = 0.0
        self._target_x: Optional[float] = None
        self._target_y: Optional[float] = None
        self._target_size_px: Optional[float] = None
        self._target_payload = ""
        self._target_polygon: Tuple[Tuple[int, int], ...] = tuple()
        self._target_rect: Tuple[int, int, int, int] = (0, 0, 0, 0)

        self._fps = 0.0
        self._fps_n = 0
        self._fps_t = time.perf_counter()
        self._win_ok = False
        self._last_debug_log_time = 0.0

        self._qr_detector = cv2.QRCodeDetector()

        self._declare_params()
        self._load_params()
        self._controller = KcfStyleFollowController(
            self, self._get_control_target, self._control_is_active
        )

        video_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self._sub = self.create_subscription(
            Image, self._img_topic, self._on_image, video_qos
        )

        self._image_timer = self.create_timer(
            1.0 / max(self._img_rate, MINIMUM_TIMER_RATE_HZ), self._process_latest_image
        )
        self._key_timer = None
        if self._show_win:
            self._init_win()
            self._key_timer = self.create_timer(0.05, self._poll_keys)

        self._param_cb = self.add_on_set_parameters_callback(self._on_param_update)
        self.get_logger().info(
            f"QR follow started with KCF-style PTZ/body control at "
            f"{self._controller.follow_rate_hz:.1f}Hz. backend={self._qr_detector_backend}"
        )

    def _declare_params(self) -> None:
        KcfStyleFollowController.declare_parameters(self)
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("window_name", "qr_follow_ptz")
        self.declare_parameter("show_debug_window", True)
        self.declare_parameter("window_width", 640)
        self.declare_parameter("window_height", 480)
        self.declare_parameter("overlay_font_scale", 0.38)
        self.declare_parameter("overlay_line_height", 16)
        self.declare_parameter("overlay_margin_x", 8)
        self.declare_parameter("overlay_margin_y", 16)
        self.declare_parameter("overlay_show_background", True)
        self.declare_parameter("enable_horizontal_flip", False)
        self.declare_parameter("enable_vertical_flip", False)
        self.declare_parameter("rotate_image_code", "none")
        self.declare_parameter("image_processing_rate_hz", 30.0)
        self.declare_parameter("lost_timeout", 0.5)
        self.declare_parameter("follow_stable_frames", 3)
        self.declare_parameter("lost_frame_threshold", 5)
        self.declare_parameter("qr_detector_backend", "pyzbar")
        self.declare_parameter("qr_select_strategy", "largest")

    def _load_params(self) -> None:
        self._img_topic = str(self.get_parameter("image_topic").value)
        self._win_name = str(self.get_parameter("window_name").value)
        self._show_win = to_bool(self.get_parameter("show_debug_window").value, True)
        self._img_w = int(self.get_parameter("image_width").value)
        self._img_h = int(self.get_parameter("image_height").value)
        self._win_w = int(self.get_parameter("window_width").value)
        self._win_h = int(self.get_parameter("window_height").value)
        self._font = float(self.get_parameter("overlay_font_scale").value)
        self._lh = int(self.get_parameter("overlay_line_height").value)
        self._mx = int(self.get_parameter("overlay_margin_x").value)
        self._my = int(self.get_parameter("overlay_margin_y").value)
        self._ov_bg = to_bool(self.get_parameter("overlay_show_background").value, True)
        self._flip_h = to_bool(self.get_parameter("enable_horizontal_flip").value, False)
        self._flip_v = to_bool(self.get_parameter("enable_vertical_flip").value, False)
        self._rot = str(self.get_parameter("rotate_image_code").value).lower()
        self._img_rate = max(float(self.get_parameter("image_processing_rate_hz").value), MINIMUM_TIMER_RATE_HZ)
        self._lost_timeout = max(float(self.get_parameter("lost_timeout").value), 0.05)
        self._follow_stable_frames = max(1, int(self.get_parameter("follow_stable_frames").value))
        self._lost_frame_threshold = max(1, int(self.get_parameter("lost_frame_threshold").value))
        self._qr_detector_backend = str(self.get_parameter("qr_detector_backend").value).lower()
        if self._qr_detector_backend not in {"pyzbar", "opencv", "auto"}:
            self._qr_detector_backend = "pyzbar"
        self._qr_select_strategy = str(self.get_parameter("qr_select_strategy").value).lower()
        if self._qr_select_strategy not in {"largest", "nearest_center", "first"}:
            self._qr_select_strategy = "largest"
        self._rqt = to_bool(self.get_parameter("enable_rqt_dynamic_parameters").value, True)

    def _on_image(self, msg: Image) -> None:
        with self._image_lock:
            self._latest_image_msg = msg
            self._latest_image_seq += 1

    def _process_latest_image(self) -> None:
        with self._image_lock:
            if self._latest_image_msg is None:
                return
            image_msg = self._latest_image_msg
            image_seq = self._latest_image_seq

        if image_seq == self._processed_image_seq:
            self._update_debug_window()
            return
        self._processed_image_seq = image_seq

        frame = self._decode_image(image_msg)
        if frame is None:
            return
        frame = self._prepare_frame(frame)
        self._latest_frame = frame

        display_frame = frame.copy()
        candidates = self._detect_qr(display_frame)
        primary_index = self._select_target_index(candidates)
        primary_target = (
            candidates[primary_index]
            if primary_index is not None
            else QrTarget(False)
        )
        self._update_target(primary_target)
        self._draw_overlay(display_frame, len(candidates))
        self._draw_qr_targets(display_frame, candidates, primary_index)
        self._latest_frame = display_frame

        self._fps_n += 1
        now = time.perf_counter()
        if now - self._fps_t >= 1.0:
            self._fps = self._fps_n / (now - self._fps_t)
            self._fps_n = 0
            self._fps_t = now
        self._update_debug_window()

    def _decode_image(self, image_msg: Image) -> Optional[np.ndarray]:
        try:
            return image_to_bgr(image_msg)
        except Exception:
            return None

    def _prepare_frame(self, frame: np.ndarray) -> np.ndarray:
        if self._img_w > 0 and self._img_h > 0:
            frame = cv2.resize(frame, (self._img_w, self._img_h))
        if self._flip_h:
            frame = cv2.flip(frame, 1)
        if self._flip_v:
            frame = cv2.flip(frame, 0)
        if self._rot == "90_clockwise":
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif self._rot == "90_counterclockwise":
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        elif self._rot == "180":
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        return frame

    def _detect_qr(self, frame: np.ndarray) -> List[QrTarget]:
        backend = self._qr_detector_backend
        candidates: List[QrTarget] = []
        if backend in {"pyzbar", "auto"}:
            candidates.extend(self._detect_qr_with_pyzbar(frame))
            if candidates:
                return candidates
        if backend == "pyzbar":
            return []
        candidates.extend(self._detect_qr_with_opencv(frame))
        return candidates

    def _detect_qr_with_pyzbar(self, frame: np.ndarray) -> List[QrTarget]:
        if pyzbar_module is None:
            self._throttled_debug_log("pyzbar unavailable. Install libzbar0/python3-pyzbar or set qr_detector_backend:=opencv.")
            return []
        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            decoded = pyzbar_module.decode(gray)
        except Exception as exc:
            self._throttled_debug_log(f"pyzbar QR detection failed: {exc}")
            return []
        targets: List[QrTarget] = []
        for item in decoded:
            payload = item.data.decode("utf-8", errors="ignore").strip()
            if not payload:
                continue
            x, y, w, h = item.rect
            polygon = tuple((int(p.x), int(p.y)) for p in item.polygon)
            if not polygon:
                polygon = ((x, y), (x + w, y), (x + w, y + h), (x, y + h))
            targets.append(QrTarget(
                True,
                payload,
                (int(x), int(y), int(w), int(h)),
                polygon,
                (float(x + w * 0.5), float(y + h * 0.5)),
            ))
        return targets

    def _detect_qr_with_opencv(self, frame: np.ndarray) -> List[QrTarget]:
        targets: List[QrTarget] = []
        try:
            if hasattr(self._qr_detector, "detectAndDecodeMulti"):
                ok, decoded_info, points, _ = self._qr_detector.detectAndDecodeMulti(frame)
                if ok and points is not None:
                    for payload, point_array in zip(decoded_info, points):
                        payload = str(payload).strip()
                        if payload:
                            targets.append(self._target_from_points(payload, point_array))
            if targets:
                return targets
            payload, points, _ = self._qr_detector.detectAndDecode(frame)
            payload = str(payload).strip()
            if payload and points is not None:
                targets.append(self._target_from_points(payload, points))
        except Exception as exc:
            self._throttled_debug_log(f"OpenCV QR detection failed: {exc}")
        return targets

    def _select_target_index(
        self,
        candidates: List[QrTarget],
    ) -> Optional[int]:
        if not candidates:
            return None
        if self._qr_select_strategy == "nearest_center":
            cx = self._img_w * 0.5
            cy = self._img_h * 0.5
            return min(
                range(len(candidates)),
                key=lambda index: math.hypot(
                    candidates[index].center[0] - cx,
                    candidates[index].center[1] - cy,
                ),
            )
        if self._qr_select_strategy == "first":
            return 0
        return max(
            range(len(candidates)),
            key=lambda index: candidates[index].rect[2] * candidates[index].rect[3],
        )

    def _target_from_points(self, payload: str, point_array: np.ndarray) -> QrTarget:
        points = np.array(point_array, dtype=np.float32).reshape(-1, 2)
        polygon = tuple((int(point[0]), int(point[1])) for point in points)
        x, y, w, h = cv2.boundingRect(np.array(polygon, dtype=np.int32))
        center = (float(np.mean(points[:, 0])), float(np.mean(points[:, 1])))
        return QrTarget(True, payload, (int(x), int(y), int(w), int(h)), polygon, center)

    def _throttled_debug_log(self, text: str) -> None:
        now = time.perf_counter()
        if now - self._last_debug_log_time >= 2.0:
            self.get_logger().warn(text)
            self._last_debug_log_time = now

    def _update_target(self, target: QrTarget) -> None:
        now = time.monotonic()
        with self._tgt_lock:
            if target.found:
                self._lost_frames = 0
                self._stable_track_frames += 1
                self._has_target = True
                self._last_target_time = now
                self._target_payload = target.payload
                self._target_x = float(target.center[0])
                self._target_y = float(target.center[1])
                self._target_rect = target.rect
                self._target_polygon = target.polygon
                self._target_size_px = math.sqrt(
                    max(1.0, float(target.rect[2]))
                    * max(1.0, float(target.rect[3]))
                )
            else:
                self._lost_frames += 1
                if now - self._last_target_time > self._lost_timeout:
                    self._has_target = False
                    self._stable_track_frames = 0
                    self._target_size_px = None

    def _control_is_active(self) -> bool:
        return self._state == STATE_TRACKING

    def _get_control_target(self) -> Dict[str, Any]:
        with self._tgt_lock:
            tx, ty = self._target_x, self._target_y
            x, y, width, height = self._target_rect
            target_age = (
                time.monotonic() - self._last_target_time
                if self._last_target_time > 0.0
                else 999.0
            )
            valid = (
                self._has_target
                and tx is not None
                and ty is not None
                and target_age <= self._lost_timeout
                and self._stable_track_frames >= self._follow_stable_frames
                and width > 0
                and height > 0
            )
        return {
            "valid": valid,
            "stamp": self._last_target_time,
            "cx": float(tx or 0.0),
            "cy": float(ty or 0.0),
            "width": float(width),
            "height": float(height),
            "image_width": self._img_w,
            "image_height": self._img_h,
        }

    def _pub_zero_cmd(self) -> None:
        self._controller.publish_zero_twist()

    def _update_debug_window(self) -> None:
        if not self._show_win:
            if self._win_ok:
                cv2.destroyWindow(self._win_name)
                self._win_ok = False
            return
        if self._latest_frame is None:
            return
        if not self._win_ok:
            self._init_win()
        cv2.imshow(self._win_name, self._latest_frame)

    def _init_win(self) -> None:
        cv2.namedWindow(self._win_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self._win_name, self._win_w, self._win_h)
        self._win_ok = True

    def _poll_keys(self) -> None:
        try:
            k = cv2.waitKey(1) & 0xFF
        except Exception:
            return
        if k in (255, -1):
            return
        if k in (ord("q"), ord("Q"), KEY_CODE_ESCAPE):
            self._shutdown = True
            self._pub_zero_cmd()
            try:
                self.get_logger().info("Force exiting.")
            except Exception:
                pass
            os._exit(0)
        elif k == KEY_CODE_SPACE:
            if self._state == STATE_TRACKING:
                self._state = STATE_IDLE
                self._reset_pd()
                self._pub_zero_cmd()
                self.get_logger().info("IDLE")
            else:
                self._state = STATE_TRACKING
                self._reset_pd()
                self.get_logger().info("TRACKING (PTZ+body)")
        elif k in (ord("i"), ord("I")):
            self._state = STATE_IDLE
            self._reset_pd()
            self._pub_zero_cmd()
            self.get_logger().info("IDLE")

    def _pub_ptz(self) -> None:
        self._controller.publish_ptz_angle()

    def _reset_pd(self) -> None:
        self._controller.reset()
        self._lost_frames = 0
        self._stable_track_frames = 0

    def _draw_qr_targets(
        self,
        frame: np.ndarray,
        candidates: List[QrTarget],
        primary_index: Optional[int],
    ) -> None:
        for index, target in enumerate(candidates):
            if index == primary_index:
                continue
            self._draw_qr_annotation(frame, target, index, False)

        if primary_index is not None and 0 <= primary_index < len(candidates):
            self._draw_qr_annotation(
                frame,
                candidates[primary_index],
                primary_index,
                True,
            )

    def _draw_qr_annotation(
        self,
        frame: np.ndarray,
        target: QrTarget,
        index: int,
        is_primary: bool,
    ) -> None:
        if not target.found:
            return

        x, y, width, height = target.rect
        color = QR_PRIMARY_COLOR if is_primary else QR_CANDIDATE_COLOR
        thickness = 3 if is_primary else 2
        if target.polygon:
            polygon = np.array(target.polygon, dtype=np.int32).reshape(
                (-1, 1, 2)
            )
            cv2.polylines(frame, [polygon], True, color, thickness)
        else:
            cv2.rectangle(
                frame,
                (x, y),
                (x + width, y + height),
                color,
                thickness,
            )
        cv2.circle(
            frame,
            (int(target.center[0]), int(target.center[1])),
            5,
            color,
            -1,
        )

        payload = (
            target.payload[:32] + "..."
            if len(target.payload) > 32
            else target.payload
        )
        prefix = "PRIMARY " if is_primary else ""
        label = "%sQR[%d]: %s" % (prefix, index + 1, payload)
        (label_width, _), _ = cv2.getTextSize(
            label,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            thickness,
        )
        text_x = max(0, min(x, frame.shape[1] - label_width - 1))
        text_y = max(22, y - 8)
        cv2.putText(
            frame,
            label,
            (text_x, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            thickness,
            cv2.LINE_AA,
        )

    def _draw_overlay(self, frame: np.ndarray, qr_count: int) -> None:
        iw, ih = frame.shape[1], frame.shape[0]
        icx, icy = iw // 2, ih // 2
        cv2.drawMarker(frame, (icx, icy), (0, 255, 255), cv2.MARKER_CROSS, 20, 2)
        tolerance = int(self._controller.pixel_tolerance)
        cv2.rectangle(frame, (icx - tolerance, icy - tolerance),
                      (icx + tolerance, icy + tolerance), (0, 180, 255), 2)

        with self._tgt_lock:
            ok = self._has_target
            tx, ty = self._target_x, self._target_y
            size_px = self._target_size_px
            payload = self._target_payload
            lf = self._lost_frames

        payload = payload[:32] + "..." if len(payload) > 32 else payload
        tgt_txt = f"QR:{payload} center:{int(tx)},{int(ty)}" if ok and tx is not None and ty is not None else f"QR:None(lost{lf}/{self._lost_frame_threshold})"
        dist_txt = (
            f"Dist size:{int(size_px)}/{int(self._controller.target_bbox_size)}px"
            if size_px else "Dist size:--"
        )
        lines = [
            f"State:{self._state} FPS:{int(self._fps)} count:{qr_count} {tgt_txt}",
            f"stable:{self._stable_track_frames}/{self._follow_stable_frames} age:{(time.monotonic() - self._last_target_time) if self._last_target_time > 0.0 else 999.0:.2f}s",
            f"PTZ s1:{self._controller.horizontal_angle:.1f} s2:{self._controller.pitch_angle:.1f} body_kp:{self._controller.body_kp:.2f}",
            f"{dist_txt}",
            f"PD x:{self._controller.ptz_kp_x:.2f}/{self._controller.ptz_kd_x:.2f} step:{self._controller.max_ptz_step:.1f} FF:{self._controller.body_ff_gain:.1f}",
            "SPACE:toggle | i:idle | q:quit",
        ]
        if self._ov_bg:
            bh = self._lh * len(lines) + 8
            cv2.rectangle(frame, (0, 0), (iw, min(ih, bh)), (0, 0, 0), -1)
        for i, t in enumerate(lines):
            cv2.putText(frame, t, (self._mx, self._my + i * self._lh),
                        cv2.FONT_HERSHEY_SIMPLEX, self._font, (0, 255, 255), 1, cv2.LINE_AA)

    def _on_param_update(self, params: List[Parameter]) -> SetParametersResult:
        if not params:
            return SetParametersResult(successful=True)
        names = {p.name for p in params}
        bad = names - DYNAMIC_PARAMETER_NAMES - RESTART_REQUIRED_PARAMETER_NAMES
        if bad:
            return SetParametersResult(successful=False, reason=f"Unsupported: {sorted(bad)}")
        if names & RESTART_REQUIRED_PARAMETER_NAMES:
            return SetParametersResult(successful=False, reason="Restart required")
        if not self._rqt and names != {"enable_rqt_dynamic_parameters"}:
            return SetParametersResult(successful=False, reason="Disabled")

        old_img = self._img_rate
        old_debug, old_size = self._show_win, (self._win_w, self._win_h)
        v = self._merge(params)
        err = self._check(v)
        if err:
            return SetParametersResult(successful=False, reason=err)
        self._apply(v)

        if abs(self._img_rate - old_img) > 1e-9:
            self.destroy_timer(self._image_timer)
            self._image_timer = self.create_timer(1.0 / max(self._img_rate, MINIMUM_TIMER_RATE_HZ), self._process_latest_image)
        self._sync_win(old_debug, old_size)
        return SetParametersResult(successful=True)

    def _merge(self, params: List[Parameter]) -> Dict[str, Any]:
        d = dict(
            show_debug_window=self._show_win,
            image_width=self._img_w, image_height=self._img_h, window_width=self._win_w, window_height=self._win_h,
            overlay_font_scale=self._font, overlay_line_height=self._lh, overlay_margin_x=self._mx, overlay_margin_y=self._my,
            overlay_show_background=self._ov_bg, enable_horizontal_flip=self._flip_h, enable_vertical_flip=self._flip_v,
            rotate_image_code=self._rot, image_processing_rate_hz=self._img_rate, lost_timeout=self._lost_timeout, follow_stable_frames=self._follow_stable_frames,
            lost_frame_threshold=self._lost_frame_threshold,
            qr_detector_backend=self._qr_detector_backend,
            qr_select_strategy=self._qr_select_strategy,
            enable_rqt_dynamic_parameters=self._rqt
        )
        for p in params:
            if p.type_ != Parameter.Type.NOT_SET:
                d[p.name] = p.value
        return d

    def _check(self, v: Dict[str, Any]) -> str:
        control_error = self._controller.validate_parameter_overrides(v)
        if control_error:
            return control_error
        try:
            if int(v["lost_frame_threshold"]) < 1:
                return "lost_frame_threshold >=1"
            if float(v["lost_timeout"]) < 0.05:
                return "lost_timeout >= 0.05"
            if int(v["follow_stable_frames"]) < 1:
                return "follow_stable_frames >= 1"
            if float(v["image_processing_rate_hz"]) < 1.0:
                return "rate >=1"
        except Exception as e:
            return str(e)
        return ""

    def _apply(self, v: Dict[str, Any]) -> None:
        self._show_win = to_bool(v["show_debug_window"], True)
        self._img_w = int(v["image_width"])
        self._img_h = int(v["image_height"])
        self._win_w = int(v["window_width"])
        self._win_h = int(v["window_height"])
        self._font = float(v["overlay_font_scale"])
        self._lh = int(v["overlay_line_height"])
        self._mx = int(v["overlay_margin_x"])
        self._my = int(v["overlay_margin_y"])
        self._ov_bg = to_bool(v["overlay_show_background"], True)
        self._flip_h = to_bool(v["enable_horizontal_flip"], False)
        self._flip_v = to_bool(v["enable_vertical_flip"], False)
        self._rot = str(v["rotate_image_code"]).lower()
        self._img_rate = max(float(v["image_processing_rate_hz"]), MINIMUM_TIMER_RATE_HZ)
        self._lost_timeout = max(float(v["lost_timeout"]), 0.05)
        self._follow_stable_frames = max(1, int(v["follow_stable_frames"]))
        self._lost_frame_threshold = max(1, int(v["lost_frame_threshold"]))
        self._qr_detector_backend = str(v["qr_detector_backend"]).lower()
        if self._qr_detector_backend not in {"pyzbar", "opencv", "auto"}:
            self._qr_detector_backend = "pyzbar"
        self._qr_select_strategy = str(v["qr_select_strategy"]).lower()
        if self._qr_select_strategy not in {"largest", "nearest_center", "first"}:
            self._qr_select_strategy = "largest"
        self._rqt = to_bool(v["enable_rqt_dynamic_parameters"], True)
        self._controller.apply_parameter_overrides(v)
        self._reset_pd()

    def _sync_win(self, prev_debug: bool, prev_size: Tuple[int, int]) -> None:
        try:
            chg = prev_size != (self._win_w, self._win_h)
            if self._show_win and not prev_debug:
                self._init_win()
            elif not self._show_win and prev_debug:
                cv2.destroyWindow(self._win_name)
            elif self._show_win and chg:
                cv2.resizeWindow(self._win_name, self._win_w, self._win_h)
        except Exception:
            pass

    def destroy_node(self) -> bool:
        self._controller.shutdown()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = QrFollowPtzNode()
    try:
        while rclpy.ok() and not node._shutdown:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
