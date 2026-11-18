# -*- coding: utf-8 -*-
"""Vision line-follow controller driven by the main LLM agent node.

The controller reuses the main node's cached camera frame and publishes chassis
velocity directly. HSV color profiles are read from ``color_hsv.txt`` where the
first four non-empty lines map to red, green, blue, and yellow.
"""

import math
import os
import threading
import time

import cv2 as cv
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, UInt16

from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class ScalarPid:
    """Small scalar PID controller for line-centering angular velocity."""

    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.reset()

    def reset(self) -> None:
        self.last_error = 0.0
        self.integrator = 0.0
        self.last_time = None

    def update(self, error: float) -> float:
        now = time.perf_counter()
        if self.last_time is None:
            self.last_time = now
            self.last_error = error
            return 0.0

        dt = max(1e-3, now - self.last_time)
        self.integrator += error * dt
        derivative = (error - self.last_error) / dt
        self.last_error = error
        self.last_time = now
        return self.kp * error + self.ki * self.integrator + self.kd * derivative


class FollowLineController:
    """Run HSV-based visual line following until finished or interrupted."""

    COLOR_ORDER = ("red", "green", "blue", "yellow")
    COLOR_ALIASES = {
        "1": "red",
        "r": "red",
        "red": "red",
        "2": "green",
        "g": "green",
        "green": "green",
        "3": "blue",
        "b": "blue",
        "blue": "blue",
        "4": "yellow",
        "y": "yellow",
        "yellow": "yellow",
    }
    DEFAULT_HSV_FILE = (
        "~/microros_ws/src/multi_brains/config/color_hsv.txt"
    )

    def __init__(self, node, config: dict, cmd_vel_pub):
        self.node = node
        self.cmd_vel_pub = cmd_vel_pub
        self.follow_config = (config or {}).get("FOLLOW_LINE", {})

        self.hsv_file = str(
            self.follow_config.get("hsv_file", self.DEFAULT_HSV_FILE) or ""
        )
        self.finished_topic = str(self.follow_config.get("finished_topic", "finished"))
        self.joy_state_topic = str(
            self.follow_config.get("joy_state_topic", "/JoyState")
        )
        self.scan_topic = str(self.follow_config.get("scan_topic", "/scan"))
        self.buzzer_topic = str(self.follow_config.get("buzzer_topic", "/beep"))

        self.image_width = max(1, int(self.follow_config.get("image_width", 640)))
        self.image_height = max(1, int(self.follow_config.get("image_height", 480)))
        self.linear_speed = float(self.follow_config.get("linear", 0.45))
        self.scale = max(1.0, float(self.follow_config.get("scale", 1000)))
        self.kp = float(self.follow_config.get("Kp", 80)) / self.scale
        self.ki = float(self.follow_config.get("Ki", 0)) / self.scale
        self.kd = float(self.follow_config.get("Kd", 20)) / self.scale
        self.pid = ScalarPid(self.kp, self.ki, self.kd)

        motion_config = (config or {}).get("MOTION", {})
        default_max_angular = float(motion_config.get("max_angular_speed", 1.5))
        self.max_angular_z = abs(
            float(self.follow_config.get("max_angular_z", default_max_angular))
        )
        self.angular_sign = float(self.follow_config.get("angular_sign", 1.0))
        self.image_flip = bool(self.follow_config.get("image_flip", False))
        self.center_tolerance_px = float(
            self.follow_config.get("center_tolerance_px", 40)
        )
        self.line_error_divisor = max(
            1e-6, float(self.follow_config.get("line_error_divisor", 16.0))
        )
        self.min_line_radius = float(self.follow_config.get("min_line_radius", 10.0))
        self.lost_timeout_sec = max(
            0.1, float(self.follow_config.get("lost_timeout_sec", 10.0))
        )
        self.max_runtime_sec = max(
            0.0, float(self.follow_config.get("max_runtime_sec", 0.0))
        )
        self.lost_log_after_sec = max(
            0.0, float(self.follow_config.get("lost_log_after_sec", 5.0))
        )
        self.status_log_interval_sec = max(
            0.0, float(self.follow_config.get("status_log_interval_sec", 0.0))
        )
        self.control_interval_sec = max(
            0.01, float(self.follow_config.get("control_interval_sec", 0.025))
        )
        self.camera_start_timeout_sec = max(
            0.0, float(self.follow_config.get("camera_start_timeout_sec", 3.0))
        )
        self.show_debug_window = bool(
            self.follow_config.get("show_debug_window", False)
        )
        self.debug_window_name = str(
            self.follow_config.get("debug_window_name", "follow_line")
        )
        self._debug_window_created = False
        self._debug_window_unavailable_logged = False
        self._debug_window_condition = threading.Condition()
        self._debug_window_thread = None
        self._debug_pending_frame = None
        self._debug_close_requested = False
        self._debug_window_closed = threading.Event()
        self._debug_window_closed.set()

        self.enable_obstacle_guard = bool(
            self.follow_config.get("enable_obstacle_guard", True)
        )
        self.laser_angle_deg = float(self.follow_config.get("LaserAngle", 30))
        self.response_dist = float(self.follow_config.get("ResponseDist", 0.45))
        self.scan_sector = str(self.follow_config.get("scan_sector", "front")).lower()
        self.warning = 1

        self.enable_buzzer = bool(self.follow_config.get("enable_buzzer", True))
        self.joy_active = False
        self.active = False
        self._stop_requested = False
        self._finish_reason = ""
        self._state_lock = threading.Lock()
        self._finished_event = threading.Event()
        self._last_lost_countdown = None
        self._last_not_found_log = 0.0
        self.current_hsv_file = ""

        self.finished_pub = node.create_publisher(Bool, self.finished_topic, 5)
        self.finished_sub = node.create_subscription(
            Bool, self.finished_topic, self._finished_callback, 5
        )
        self.joy_sub = node.create_subscription(
            Bool, self.joy_state_topic, self._joy_state_callback, 1
        )

        self.buzzer_pub = None
        if self.enable_buzzer:
            self.buzzer_pub = node.create_publisher(UInt16, self.buzzer_topic, 1)

        self.scan_sub = None
        if self.enable_obstacle_guard:
            scan_qos = QoSProfile(depth=1)
            scan_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
            self.scan_sub = node.create_subscription(
                LaserScan, self.scan_topic, self._scan_callback, scan_qos
            )

        self.node.get_logger().info(
            f"[FOLLOW] scan_topic={self.scan_topic}, "
            f"obstacle_guard={self.enable_obstacle_guard}, "
            f"buzzer_topic={self.buzzer_topic}, buzzer={self.enable_buzzer}"
        )

    @property
    def finish_reason(self) -> str:
        return self._finish_reason

    def execute(self, color: str = "red") -> bool:
        """Follow the configured color line until finished or interrupted."""
        color_name = self.normalize_color(color)
        if color_name is None:
            self.node.get_logger().error(
                f"[FOLLOW] invalid color '{color}', expected r/g/b/y or names"
            )
            return False

        hsv_range = self.load_hsv_range(color_name)
        if hsv_range is None:
            return False

        self._begin_run()
        self.node.get_logger().info(
            f"[FOLLOW] start color={color_name}, hsv={hsv_range}, "
            f"hsv_file={self.current_hsv_file}"
        )

        try:
            if not self._wait_for_camera():
                self._finish_reason = "no_camera_frame"
                self.node.get_logger().error("[FOLLOW] no camera image available")
                return False

            run_started_time = time.monotonic()
            last_seen_time = run_started_time
            last_status_log_time = run_started_time
            while not self._finished_event.is_set():
                now = time.monotonic()
                if (
                    self.max_runtime_sec > 0.0
                    and now - run_started_time >= self.max_runtime_sec
                ):
                    self._finish_reason = "timeout"
                    self.node.get_logger().info(
                        f"[FOLLOW] max runtime reached: {self.max_runtime_sec:.1f}s"
                    )
                    self._finished_event.set()
                    break

                if self.node.should_interrupt():
                    self._stop_requested = True
                    self._finish_reason = "interrupted"
                    return False

                frame = self.node.get_latest_image_snapshot()
                if frame is None:
                    self._publish_zero_twist()
                    self._handle_missing_line(last_seen_time, "no image")
                    time.sleep(self.control_interval_sec)
                    continue

                point_x, color_radius, debug_frame, binary = self._detect_line(
                    frame, hsv_range
                )
                self._show_debug_window(debug_frame, binary)
                if color_radius < self.min_line_radius:
                    self._publish_zero_twist()
                    self._handle_missing_line(last_seen_time, "line not found")
                elif self._obstacle_detected():
                    self._publish_zero_twist()
                    self._set_buzzer(True)
                    self._handle_missing_line(last_seen_time, "obstacle ahead")
                else:
                    last_seen_time = time.monotonic()
                    self._last_lost_countdown = None
                    self._set_buzzer(False)
                    angular_z = self._publish_follow_twist(point_x)
                    if (
                        self.status_log_interval_sec > 0.0
                        and time.monotonic() - last_status_log_time
                        >= self.status_log_interval_sec
                    ):
                        self.node.get_logger().info(
                            "[FOLLOW] running: "
                            f"point_x={point_x:.1f}, radius={color_radius:.1f}, "
                            f"angular_z={angular_z:.3f}, joy_active={self.joy_active}"
                        )
                        last_status_log_time = time.monotonic()

                time.sleep(self.control_interval_sec)
        finally:
            self._publish_zero_twist()
            self._set_buzzer(False, repeats=3)
            self._close_debug_window()
            with self._state_lock:
                self.active = False

        if self._stop_requested:
            return False
        return self._finish_reason in (
            "line_lost",
            "external_finished",
            "finished",
            "timeout",
        )

    def stop(self) -> None:
        """Request the active line-follow action to stop."""
        with self._state_lock:
            if not self.active:
                return
            self._stop_requested = True
            self._finish_reason = "stop_requested"
            self._finished_event.set()
        self._publish_zero_twist()
        self._set_buzzer(False, repeats=3)

    def is_active(self) -> bool:
        with self._state_lock:
            return self.active

    def normalize_color(self, color) -> str | None:
        text = str(color or "red").strip().strip("'\"").lower()
        return self.COLOR_ALIASES.get(text)

    def load_hsv_range(self, color_name: str):
        hsv_path = self._resolve_hsv_file()
        if not hsv_path:
            self.node.get_logger().error(
                f"[FOLLOW] HSV file not found, configured={self.hsv_file}"
            )
            return None

        try:
            with open(hsv_path, "r", encoding="utf-8") as hsv_stream:
                lines = [
                    line.strip()
                    for line in hsv_stream
                    if line.strip() and not line.lstrip().startswith("#")
                ]
        except OSError as error:
            self.node.get_logger().error(f"[FOLLOW] failed to read HSV file: {error}")
            return None

        color_index = self.COLOR_ORDER.index(color_name)
        if color_index >= len(lines):
            self.node.get_logger().error(
                f"[FOLLOW] HSV file has {len(lines)} profiles, "
                f"missing {color_name} at line {color_index + 1}"
            )
            return None

        try:
            values = [int(part.strip()) for part in lines[color_index].split(",")]
        except ValueError as error:
            self.node.get_logger().error(
                f"[FOLLOW] invalid HSV value at line {color_index + 1}: {error}"
            )
            return None

        if len(values) != 6:
            self.node.get_logger().error(
                f"[FOLLOW] invalid HSV format at line {color_index + 1}: "
                f"expected 6 integers, got {len(values)}"
            )
            return None

        if not self._validate_hsv_values(values):
            self.node.get_logger().error(
                f"[FOLLOW] HSV values out of range at line {color_index + 1}: "
                f"{values}"
            )
            return None

        self.current_hsv_file = hsv_path
        return (tuple(values[:3]), tuple(values[3:]))

    def _resolve_hsv_file(self) -> str:
        hsv_path = os.path.expanduser(self.hsv_file)
        if os.path.exists(hsv_path):
            return hsv_path
        return ""

    def _validate_hsv_values(self, values: list[int]) -> bool:
        h_min, s_min, v_min, h_max, s_max, v_max = values
        return (
            0 <= h_min <= 179
            and 0 <= h_max <= 179
            and 0 <= s_min <= 255
            and 0 <= s_max <= 255
            and 0 <= v_min <= 255
            and 0 <= v_max <= 255
        )

    def _begin_run(self) -> None:
        self.pid.reset()
        self._finished_event.clear()
        self._last_lost_countdown = None
        self._last_not_found_log = 0.0
        with self._state_lock:
            self.active = True
            self._stop_requested = False
            self._finish_reason = "finished"

    def _wait_for_camera(self) -> bool:
        deadline = time.monotonic() + self.camera_start_timeout_sec
        while time.monotonic() <= deadline:
            if self.node.should_interrupt() or self._finished_event.is_set():
                return False
            if self.node.get_latest_image_snapshot() is not None:
                return True
            time.sleep(0.05)
        return self.node.get_latest_image_snapshot() is not None

    def _detect_line(self, frame, hsv_range):
        resized = cv.resize(frame, (self.image_width, self.image_height))
        if self.image_flip:
            resized = cv.flip(resized, 1)

        height, width = resized.shape[:2]
        debug_frame = resized.copy()
        detection_img = resized.copy()
        detection_img[0 : int(height / 2), 0:width] = 0

        hsv_img = cv.cvtColor(detection_img, cv.COLOR_BGR2HSV)
        mask = self._build_hsv_mask(hsv_img, hsv_range)
        color_mask = cv.bitwise_and(hsv_img, hsv_img, mask=mask)
        gray_img = cv.cvtColor(color_mask, cv.COLOR_RGB2GRAY)
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        gray_img = cv.morphologyEx(gray_img, cv.MORPH_CLOSE, kernel)
        _, binary = cv.threshold(gray_img, 10, 255, cv.THRESH_BINARY)

        found = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        contours = found[1] if len(found) == 3 else found[0]
        if not contours:
            return 0.0, 0.0, debug_frame, binary

        largest = max(contours, key=cv.contourArea)
        rect = cv.minAreaRect(largest)
        box = cv.boxPoints(rect)
        box = box.astype("int32")
        (center_x, center_y), radius = cv.minEnclosingCircle(box)
        cv.drawContours(debug_frame, [box], 0, (255, 0, 0), 2)
        cv.circle(debug_frame, (int(center_x), int(center_y)), 5, (255, 0, 255), -1)
        return float(center_x), float(radius), debug_frame, binary

    def _build_hsv_mask(self, hsv_img, hsv_range):
        lower, upper = hsv_range
        if lower[0] <= upper[0]:
            return cv.inRange(hsv_img, lower, upper)

        first = cv.inRange(hsv_img, lower, (179, upper[1], upper[2]))
        second = cv.inRange(hsv_img, (0, lower[1], lower[2]), upper)
        return cv.bitwise_or(first, second)

    def _publish_follow_twist(self, point_x: float) -> float:
        image_center_x = self.image_width / 2.0
        error = (image_center_x - point_x) / self.line_error_divisor
        angular_z = self.angular_sign * self.pid.update(error)
        if self.image_flip:
            angular_z = -angular_z

        if abs(point_x - image_center_x) < self.center_tolerance_px:
            angular_z = 0.0
        angular_z = max(-self.max_angular_z, min(self.max_angular_z, angular_z))

        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = angular_z
        if not self.joy_active:
            self.cmd_vel_pub.publish(twist)
        return angular_z

    def _show_debug_window(self, debug_frame, binary) -> None:
        if not self.show_debug_window:
            return
        if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            if not self._debug_window_unavailable_logged:
                self.node.get_logger().info(
                    "[FOLLOW] 无图形显示环境，跳过巡线调试窗口"
                )
                self._debug_window_unavailable_logged = True
            return

        try:
            binary_bgr = cv.cvtColor(binary, cv.COLOR_GRAY2BGR)
            if binary_bgr.shape[:2] != debug_frame.shape[:2]:
                binary_bgr = cv.resize(
                    binary_bgr,
                    (debug_frame.shape[1], debug_frame.shape[0]),
                )
            display = cv.hconcat([debug_frame, binary_bgr])
        except cv.error as error:
            self.show_debug_window = False
            self.node.get_logger().warn(f"[FOLLOW] 巡线调试窗口不可用: {error}")
            return

        self._ensure_debug_window_thread()
        with self._debug_window_condition:
            self._debug_pending_frame = display
            self._debug_window_closed.clear()
            self._debug_window_condition.notify()

    def _ensure_debug_window_thread(self) -> None:
        """Keep all OpenCV HighGUI calls on one thread across follow runs."""
        with self._debug_window_condition:
            thread = self._debug_window_thread
            if thread is not None and thread.is_alive():
                return
            thread = threading.Thread(
                target=self._debug_window_loop,
                name="follow-line-debug-window",
                daemon=True,
            )
            self._debug_window_thread = thread
            thread.start()

    def _debug_window_loop(self) -> None:
        while True:
            with self._debug_window_condition:
                while (
                    self._debug_pending_frame is None
                    and not self._debug_close_requested
                ):
                    self._debug_window_condition.wait()
                close_requested = self._debug_close_requested
                self._debug_close_requested = False
                if close_requested:
                    self._debug_pending_frame = None
                    display = None
                else:
                    display = self._debug_pending_frame
                    self._debug_pending_frame = None

            if close_requested:
                self._destroy_debug_window()
                self._debug_window_closed.set()
                continue

            try:
                if not self._debug_window_created:
                    cv.namedWindow(self.debug_window_name, cv.WINDOW_NORMAL)
                    self._debug_window_created = True
                cv.imshow(self.debug_window_name, display)
                cv.waitKey(1)
            except cv.error as error:
                self.show_debug_window = False
                self._destroy_debug_window()
                self._debug_window_closed.set()
                if not self._debug_window_unavailable_logged:
                    self.node.get_logger().warn(
                        f"[FOLLOW] 巡线调试窗口不可用: {error}"
                    )
                    self._debug_window_unavailable_logged = True

    def _destroy_debug_window(self) -> None:
        if not self._debug_window_created:
            return
        try:
            cv.destroyWindow(self.debug_window_name)
            cv.waitKey(1)
        except cv.error:
            pass
        self._debug_window_created = False

    def _close_debug_window(self) -> None:
        thread = self._debug_window_thread
        if thread is None or not thread.is_alive():
            self._debug_window_created = False
            self._debug_window_closed.set()
            return
        self._debug_window_closed.clear()
        with self._debug_window_condition:
            self._debug_pending_frame = None
            self._debug_close_requested = True
            self._debug_window_condition.notify()
        if not self._debug_window_closed.wait(timeout=1.0):
            self.node.get_logger().warn("[FOLLOW] 等待巡线调试窗口关闭超时")

    def _handle_missing_line(self, last_seen_time: float, reason: str) -> None:
        elapsed = time.monotonic() - last_seen_time
        now = time.monotonic()
        if now - self._last_not_found_log > 1.0:
            self.node.get_logger().info(f"[FOLLOW] {reason}")
            self._last_not_found_log = now

        if elapsed >= self.lost_timeout_sec:
            self._finish_reason = "line_lost"
            self._publish_finished()
            return

        if elapsed >= self.lost_log_after_sec:
            remaining = max(0, int(math.ceil(self.lost_timeout_sec - elapsed)))
            if remaining != self._last_lost_countdown:
                self.node.get_logger().info(f"[FOLLOW] finish in {remaining}s")
                self._last_lost_countdown = remaining

    def _publish_finished(self) -> None:
        msg = Bool()
        msg.data = True
        self.finished_pub.publish(msg)
        self._finished_event.set()

    def _publish_zero_twist(self) -> None:
        self.cmd_vel_pub.publish(Twist())

    def _set_buzzer(self, active: bool, repeats: int = 1) -> None:
        if self.buzzer_pub is None:
            return
        msg = UInt16()
        msg.data = 1 if active else 0
        for _ in range(max(1, repeats)):
            self.buzzer_pub.publish(msg)

    def _obstacle_detected(self) -> bool:
        return self.enable_obstacle_guard and self.warning > 10

    def _angle_in_obstacle_sector(self, angle_deg: float) -> bool:
        if self.scan_sector in ("rear", "back", "legacy"):
            return abs(angle_deg) > 180.0 - self.laser_angle_deg
        return abs(angle_deg) < self.laser_angle_deg

    def _finished_callback(self, msg: Bool) -> None:
        if not isinstance(msg, Bool) or not msg.data:
            return
        with self._state_lock:
            if not self.active:
                return
            if self._finish_reason == "finished":
                self._finish_reason = "external_finished"
            self._finished_event.set()

    def _joy_state_callback(self, msg: Bool) -> None:
        if isinstance(msg, Bool):
            self.joy_active = bool(msg.data)

    def _scan_callback(self, scan_data: LaserScan) -> None:
        if not isinstance(scan_data, LaserScan):
            return

        self.warning = 1
        for index, distance in enumerate(scan_data.ranges):
            if not math.isfinite(distance):
                continue
            angle = (
                scan_data.angle_min + scan_data.angle_increment * index
            ) * 180.0 / math.pi
            if self._angle_in_obstacle_sector(angle):
                if 0.2 < distance < self.response_dist:
                    self.warning += 1
