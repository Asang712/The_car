# -*- coding: utf-8 -*-
"""Nav2 navigation: goal dispatch, target loading, and the dynamic zero point.

Wraps the ``NavigateToPose`` action with interrupt-aware waiting so a newer
command can preempt an in-flight goal. Also owns the named-target table and
the runtime ``zero`` target recorded from the live ``map -> base_footprint``
transform.
"""

import os
import re
import threading

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


def normalize_nav_target_name(point_name: str, available_targets) -> str:
    """Normalize a model navigation target to an existing target key."""
    raw_name = str(point_name or "").strip().strip("'\"")
    target_names = [str(target) for target in available_targets]
    if raw_name in target_names:
        return raw_name

    for target in target_names:
        pattern = rf"(?<![A-Za-z0-9]){re.escape(target)}(?![A-Za-z0-9])"
        if re.search(pattern, raw_name):
            return target

    return raw_name


class NavigationController:
    """Drive Nav2 goal dispatch and own the target pose table."""

    def __init__(self, node, config):
        self.node = node
        nav_config = config.get("NAVIGATION", {})
        self.enabled = bool(nav_config.get("enable", True))

        self.nav_client = ActionClient(node, NavigateToPose, "/navigate_to_pose")
        self.navpose_dict = {}
        self.first_record = True
        self.nav_state_lock = threading.Lock()
        self.active_nav_state = None
        self.active_nav_goal_handle = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)

        self.load_nav_targets(config)

    def load_nav_targets(self, config) -> None:
        """Load named navigation targets from the configured map-mapping file."""
        nav_config = config.get("NAVIGATION", {})
        map_file = nav_config.get("map_mapping_file", "")

        if not map_file or not os.path.exists(map_file):
            self.node.get_logger().warn("Nav targets file not found")
            return

        with open(map_file, "r") as handle:
            targets = yaml.safe_load(handle)

        common_areas = targets.get("common_map_areas", {})
        for name, data in common_areas.items():
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = data["position"]["x"]
            pose.pose.position.y = data["position"]["y"]
            pose.pose.position.z = data["position"]["z"]
            pose.pose.orientation.x = data["orientation"]["x"]
            pose.pose.orientation.y = data["orientation"]["y"]
            pose.pose.orientation.z = data["orientation"]["z"]
            pose.pose.orientation.w = data["orientation"]["w"]
            self.navpose_dict[name] = pose

    def cancel_active_goal(self) -> None:
        """Cancel any in-flight Nav2 goal (called on command preemption)."""
        with self.nav_state_lock:
            nav_state = self.active_nav_state
            if nav_state is not None:
                nav_state["cancel_requested"].set()
                goal_handle = nav_state.get("goal_handle")
            else:
                goal_handle = self.active_nav_goal_handle
            self.active_nav_state = None
            self.active_nav_goal_handle = None

        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception as error:
                self.node.get_logger().warn(f"[ACTION] 取消导航失败: {error}")

    def _activate_nav_state(self, nav_state: dict) -> None:
        """Install a pending goal state and cancel any superseded state."""
        with self.nav_state_lock:
            previous_state = self.active_nav_state
            self.active_nav_state = nav_state
            self.active_nav_goal_handle = None
        if previous_state is not None:
            previous_state["cancel_requested"].set()
            previous_handle = previous_state.get("goal_handle")
            if previous_handle is not None:
                try:
                    previous_handle.cancel_goal_async()
                except Exception as error:
                    self.node.get_logger().warn(f"[NAV] 取消被替代目标失败: {error}")

    def _clear_nav_state(self, nav_state: dict) -> None:
        """Clear ``nav_state`` only when it is still the active request."""
        with self.nav_state_lock:
            if self.active_nav_state is nav_state:
                self.active_nav_state = None
                self.active_nav_goal_handle = None

    def _cancel_nav_state(self, nav_state: dict) -> None:
        """Mark a pending/accepted goal canceled and clear its active slot."""
        nav_state["cancel_requested"].set()
        goal_handle = nav_state.get("goal_handle")
        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception as error:
                self.node.get_logger().warn(f"[NAV] 取消导航失败: {error}")
        self._clear_nav_state(nav_state)

    def get_current_pose(self) -> bool:
        """Record the current map pose as the dynamic ``zero`` target."""
        if not self.enabled:
            self.node.get_logger().warn("[NAVIGATION] 导航功能在配置中已禁用")
            return False
        return self._record_current_pose(add_feedback=True)

    def _record_current_pose(self, add_feedback: bool) -> bool:
        """Store map -> base_footprint as the ``navigation(zero)`` target."""
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            )

            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.node.get_clock().now().to_msg()
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            self.navpose_dict["zero"] = pose
            self.first_record = False

            position = pose.pose.position
            orientation = pose.pose.orientation
            pose_text = (
                f"已记录当前位置为zero点: "
                f"position=({position.x:.3f}, {position.y:.3f}, {position.z:.3f}), "
                f"orientation=({orientation.x:.3f}, {orientation.y:.3f}, "
                f"{orientation.z:.3f}, {orientation.w:.3f})"
            )
            self.node.get_logger().info(f"[NAVIGATION] {pose_text}")
            if add_feedback:
                self.node.add_pending_action_feedback(pose_text)
            return True
        except Exception as error:
            self.node.get_logger().error(
                f"[NAVIGATION] 获取当前位置失败: {type(error).__name__}: {error}"
            )
            return False

    def execute_navigation(self, point_name: str) -> bool:
        """Use the normal Nav2 NavigateToPose action and wait for its result."""
        if not self.enabled:
            self.node.get_logger().warn("[NAVIGATION] 导航功能在配置中已禁用")
            return False

        point_name = normalize_nav_target_name(point_name, self.navpose_dict.keys())
        self.node.get_logger().info(f"[NAVIGATION] 开始普通导航: {point_name}")

        nav_accept_timeout = 30.0
        nav_result_timeout = 300.0

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.node.get_logger().error(
                "[NAV] Nav2 服务不可用: /navigate_to_pose 未响应"
            )
            return False

        target_pose = self.navpose_dict.get(point_name)
        if not target_pose:
            available = list(self.navpose_dict.keys())
            self.node.get_logger().error(
                f"[NAV] 目标点 '{point_name}' 不存在, 可用: {available}"
            )
            return False

        if self.first_record and point_name != "zero":
            if not self._record_current_pose(add_feedback=False):
                self.node.get_logger().error("[NAVIGATION] 无法记录出发位置，取消导航")
                return False

        pos = target_pose.pose.position
        ori = target_pose.pose.orientation
        self.node.get_logger().info(
            f"[NAV] 目标坐标: pos=({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}), "
            f"ori=({ori.x:.3f}, {ori.y:.3f}, {ori.z:.3f}, {ori.w:.3f})"
        )

        nav_state = {
            "goal_accepted": threading.Event(),
            "result_received": threading.Event(),
            "cancel_requested": threading.Event(),
            "goal_handle": None,
            "result_status": None,
        }
        self._activate_nav_state(nav_state)

        def goal_response_callback(future):
            try:
                goal_handle = future.result()
            except Exception as error:
                self.node.get_logger().error(
                    f"[NAV] 目标响应异常: {type(error).__name__}: {error}"
                )
                nav_state["goal_accepted"].set()
                return

            if goal_handle and goal_handle.accepted:
                nav_state["goal_handle"] = goal_handle
                with self.nav_state_lock:
                    if self.active_nav_state is nav_state:
                        self.active_nav_goal_handle = goal_handle
                nav_state["goal_accepted"].set()

                if nav_state["cancel_requested"].is_set():
                    goal_handle.cancel_goal_async()
                    self.node.get_logger().info(
                        f"[NAV] 已取消晚到的导航目标: '{point_name}'"
                    )
                    return

                self.node.get_logger().info(f"[NAV] 导航目标已接受: '{point_name}'")
                self.node.publish_progress(f"[导航] 正在前往{point_name}点...")

                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(result_callback)
            else:
                nav_state["goal_handle"] = None
                nav_state["goal_accepted"].set()
                self.node.get_logger().error(f"[NAV] 导航目标被拒绝: '{point_name}'")
                self.node.publish_progress(f"[导航] 目标{point_name}点被拒绝")

        def result_callback(future):
            try:
                wrapped_result = future.result()
                nav_state["result_status"] = wrapped_result.status
                self.node.get_logger().info(
                    f"[NAV] result_callback: status={wrapped_result.status}, "
                    f"result_type={type(wrapped_result.result).__name__}"
                )
            except Exception as error:
                self.node.get_logger().error(
                    f"[NAV] result 回调异常: {type(error).__name__}: {error}"
                )
                nav_state["result_status"] = -1
            finally:
                nav_state["result_received"].set()

        goal_msg = NavigateToPose.Goal()
        target_pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose = target_pose
        try:
            goal_handle_future = self.nav_client.send_goal_async(goal_msg)
        except Exception as error:
            self._cancel_nav_state(nav_state)
            self.node.get_logger().error(
                f"[NAV] 发送导航目标失败: {type(error).__name__}: {error}"
            )
            return False
        goal_handle_future.add_done_callback(goal_response_callback)

        if not self.node.wait_event_or_interrupt(
            nav_state["goal_accepted"], nav_accept_timeout
        ):
            self._cancel_nav_state(nav_state)
            if self.node.should_interrupt():
                self.node.get_logger().warn(
                    f"[NAV] 导航到 '{point_name}' 在目标接受前被中断"
                )
            else:
                self.node.get_logger().error(
                    f"[NAV] 导航目标接受超时 ({nav_accept_timeout}s)"
                )
            return False

        if self.node.should_interrupt():
            self._cancel_nav_state(nav_state)
            self.node.get_logger().warn(f"[NAV] 导航到 '{point_name}' 被中断")
            return False

        if nav_state["goal_handle"] is None:
            self._clear_nav_state(nav_state)
            self.node.get_logger().error(f"[NAV] 导航目标被拒绝: '{point_name}'")
            return False

        self.node.get_logger().info(
            f"[NAV] 等待到达 '{point_name}' (超时{nav_result_timeout}s)..."
        )

        if not self.node.wait_event_or_interrupt(
            nav_state["result_received"], nav_result_timeout
        ):
            if self.node.should_interrupt():
                self._cancel_nav_state(nav_state)
                self.node.execute_stop(recenter_camera=False)
                self.node.get_logger().warn(f"[NAV] 导航到 '{point_name}' 被新指令中断")
                return False

            self.node.get_logger().error(
                f"[NAV] 导航超时 ({nav_result_timeout}s)，取消导航"
            )
            self._cancel_nav_state(nav_state)
            self.node.execute_stop(recenter_camera=False)
            return False

        if self.node.should_interrupt():
            self._cancel_nav_state(nav_state)
            self.node.execute_stop(recenter_camera=False)
            self.node.get_logger().warn(f"[NAV] 导航到 '{point_name}' 被中断")
            return False

        self.node.execute_stop(recenter_camera=False)
        self._clear_nav_state(nav_state)

        result_status = nav_state["result_status"]
        self.node.get_logger().info(f"[NAV] 导航完成: status={result_status}")

        # GoalStatus: STATUS_SUCCEEDED=4, STATUS_CANCELED=5, STATUS_ABORTED=6
        if result_status == 4:
            self.node.get_logger().info(f"[NAV] 成功到达 '{point_name}' ✓")
            self.node.publish_progress(f"[导航] 已到达{point_name}点 ✓")
            return True

        self.node.get_logger().error(
            f"[NAV] 导航到 '{point_name}' 状态异常: status={result_status}"
        )
        self.node.publish_progress(
            f"[导航] 前往{point_name}点失败 (status={result_status})"
        )
        return False

