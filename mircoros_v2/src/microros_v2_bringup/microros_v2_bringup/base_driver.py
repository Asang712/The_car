#!/usr/bin/env python
# encoding: utf-8
# author: Yahboom Technology ROS Robotics Team
# version: 202607

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist, Pose,Vector3
from nav_msgs.msg import Odometry
from interfaces.action import MoveByOdom, PreciseAdjustPose
from interfaces.srv import GetRobotPose, GetPTZAngle
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from sensor_msgs.msg import JointState
class BaseDriverNode(Node):
    def __init__(self, name: str):
        super().__init__(name)

        # ======================== MoveByOdom Action Parameters ========================
        self.declare_parameter('odom_linear_speed', 0.1)
        self.odom_linear_speed = self.get_parameter('odom_linear_speed').get_parameter_value().double_value
        self.declare_parameter('odom_angular_speed', 0.5)
        self.odom_angular_speed = self.get_parameter('odom_angular_speed').get_parameter_value().double_value
        self.declare_parameter('odom_position_tolerance', 0.02)
        self.odom_position_tolerance = self.get_parameter('odom_position_tolerance').get_parameter_value().double_value
        self.declare_parameter('odom_angle_tolerance', 0.1)
        self.odom_angle_tolerance = self.get_parameter('odom_angle_tolerance').get_parameter_value().double_value
        self.declare_parameter('odom_timeout', 30.0)
        self.odom_timeout = self.get_parameter('odom_timeout').get_parameter_value().double_value

        # ======================== Shared Minimum Speed (anti-stall) ========================
        self.declare_parameter('min_linear_speed', 0.03)     # m/s - minimum for both move_by_odom & precise_adjust
        self.min_linear_speed = self.get_parameter('min_linear_speed').get_parameter_value().double_value
        self.declare_parameter('min_angular_speed', 0.05)    # rad/s - minimum for both move_by_odom & precise_adjust
        self.min_angular_speed = self.get_parameter('min_angular_speed').get_parameter_value().double_value

        # ======================== Precise Adjust PID Parameters ========================
        self.declare_parameter('precise_adjust_xy_pid', [0.5, 0.0, 0.1])         # [Kp, Ki, Kd]
        self.declare_parameter('precise_adjust_theta_pid', [1.0, 0.0, 0.2])       # [Kp, Ki, Kd]
        self.declare_parameter('precise_adjust_max_linear_vel', 0.15)              # m/s
        self.declare_parameter('precise_adjust_max_angular_vel', 0.5)             # rad/s
        self.declare_parameter('precise_adjust_pos_tolerance', 0.01)              # m
        self.declare_parameter('precise_adjust_angle_tolerance', 0.05)            # rad
        self.declare_parameter('precise_adjust_max_iteration', 500)

        self.precise_adjust_xy_pid = self.get_parameter('precise_adjust_xy_pid').get_parameter_value().double_array_value
        self.precise_adjust_theta_pid = self.get_parameter('precise_adjust_theta_pid').get_parameter_value().double_array_value
        self.precise_adjust_max_linear_vel = self.get_parameter('precise_adjust_max_linear_vel').get_parameter_value().double_value
        self.precise_adjust_max_angular_vel = self.get_parameter('precise_adjust_max_angular_vel').get_parameter_value().double_value
        self.precise_adjust_pos_tolerance = self.get_parameter('precise_adjust_pos_tolerance').get_parameter_value().double_value
        self.precise_adjust_angle_tolerance = self.get_parameter('precise_adjust_angle_tolerance').get_parameter_value().double_value
        self.precise_adjust_max_iteration = self.get_parameter('precise_adjust_max_iteration').get_parameter_value().integer_value

        self.declare_parameter('camera_type', 'none')
        self.camera_type = self.get_parameter('camera_type').get_parameter_value().string_value

        G = '\033[32m'
        R = '\033[0m'
        self.get_logger().info(f"{G}=== BaseDriverNode Parameters ==={R}")
        self.get_logger().info(f"{G}  odom_linear_speed       : {self.odom_linear_speed} m/s{R}")
        self.get_logger().info(f"{G}  odom_angular_speed      : {self.odom_angular_speed} rad/s{R}")
        self.get_logger().info(f"{G}  odom_position_tolerance : {self.odom_position_tolerance} m{R}")
        self.get_logger().info(f"{G}  odom_angle_tolerance    : {self.odom_angle_tolerance} rad{R}")
        self.get_logger().info(f"{G}  odom_timeout            : {self.odom_timeout} s{R}")
        self.get_logger().info(f"{G}  min_linear_speed        : {self.min_linear_speed} m/s{R}")
        self.get_logger().info(f"{G}  min_angular_speed       : {self.min_angular_speed} rad/s{R}")
        self.get_logger().info(f"{G}  precise_adjust_xy_pid   : {list(self.precise_adjust_xy_pid)}{R}")
        self.get_logger().info(f"{G}  precise_adjust_theta_pid: {list(self.precise_adjust_theta_pid)}{R}")
        self.get_logger().info(f"{G}  precise_adjust_max_lin  : {self.precise_adjust_max_linear_vel} m/s{R}")
        self.get_logger().info(f"{G}  precise_adjust_max_ang  : {self.precise_adjust_max_angular_vel} rad/s{R}")
        self.get_logger().info(f"{G}  precise_adjust_pos_tol  : {self.precise_adjust_pos_tolerance} m{R}")
        self.get_logger().info(f"{G}  precise_adjust_ang_tol  : {self.precise_adjust_angle_tolerance} rad{R}")
        self.get_logger().info(f"{G}==============================={R}")

        self._callback_group = ReentrantCallbackGroup()

        if self.camera_type == 'ptz':
            _ptz_qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
            )
            self.create_subscription(Vector3, '/ptz_angle', self._ptzAngleCallback, _ptz_qos)
            self.staPublisher = self.create_publisher(JointState, "/ptz_status", 10)
            self.ptz_x = 0.0
            self.ptz_y = 0.0
            self.get_ptz_angle_srv = self.create_service(
                GetPTZAngle,
                'get_ptz_angle',
                self._getPTZAngleCallback,
                callback_group=self._callback_group)

        self._move_by_odom_action_server = ActionServer(
            self,
            MoveByOdom,
            'move_by_odom',
            self._moveByOdomActionCallback,
            callback_group=self._callback_group)

        self._precise_adjust_action_server = ActionServer(
            self,
            PreciseAdjustPose,
            'precise_adjust_pose',
            self._preciseAdjustActionCallback,
            callback_group=self._callback_group)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self._odomCallback, 10)

        # TF buffer and listener for map -> base_footprint transform
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # GetRobotPose service
        self.get_robot_pose_srv = self.create_service(
            GetRobotPose,
            'get_robot_pose',
            self._getRobotPoseCallback,
            callback_group=self._callback_group)

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_th = 0.0

#-----------------------------Callbacks-----------------------------
    def _ptzAngleCallback(self, msg: Vector3):
        self.ptz_x = msg.x
        self.ptz_y = msg.y
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['cam1', 'cam2']
        joint_state.position = [math.radians(msg.x), math.radians(msg.y)]
        joint_state.velocity = []
        joint_state.effort = []
        self.staPublisher.publish(joint_state)

    def _getPTZAngleCallback(self, request: GetPTZAngle.Request, response: GetPTZAngle.Response):
        """Service callback: return latest PTZ servo angle (deg)."""
        response.x = float(self.ptz_x)
        response.y = float(self.ptz_y)
        response.success = True
        response.message = f"PTZ angle: x={response.x:.2f}, y={response.y:.2f}"
        return response

    def _odomCallback(self, msg: Odometry):
        """Update internal pose from external /odom source (e.g. EKF)."""
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.odom_th = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)

    def _getRobotPoseCallback(self, request: GetRobotPose.Request, response: GetRobotPose.Response):
        """Service callback: return current robot pose in map frame via TF lookup."""
        max_attempts = 10
        for attempt in range(max_attempts):
            try:
                transform = self.tf_buffer.lookup_transform(
                    "map",
                    "base_footprint",
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5))
                response.pose = Pose()
                response.pose.position.x = round(transform.transform.translation.x, 3)
                response.pose.position.y = round(transform.transform.translation.y, 3)
                response.pose.position.z = round(transform.transform.translation.z, 3)
                response.pose.orientation.x = round(transform.transform.rotation.x, 3)
                response.pose.orientation.y = round(transform.transform.rotation.y, 3)
                response.pose.orientation.z = round(transform.transform.rotation.z, 3)
                response.pose.orientation.w = round(transform.transform.rotation.w, 3)
                response.success = True
                response.message = f"Current pose: x={response.pose.position.x}, y={response.pose.position.y}"
                return response

            except Exception as e:
                self.get_logger().warn(f"Failed to lookup TF map->base_footprint: {e} (attempt {attempt + 1}/{max_attempts})")
                time.sleep(0.5)

        response.success = False
        response.message = f"Failed to get map->base_footprint transform after {max_attempts} attempts"
        self.get_logger().error(response.message)
        return response

    def _publishCmdVel(self, vx: float, angular_z: float):
        """Publish velocity command to /cmd_vel topic."""
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = 0.0
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)

    def _normalizeAngle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _getYawFromQuaternion(self, q) -> float:
        """Extract yaw angle from quaternion."""
        return math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)

    def _getMapPose(self):
        """Get current robot pose in map frame via TF. Returns (x, y, yaw) or None."""
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_footprint",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            yaw = self._getYawFromQuaternion(transform.transform.rotation)
            return x, y, yaw
        except Exception as e:
            self.get_logger().warn(f"Failed to get map->base_footprint TF: {e}")
            return None

    # Control phases for differential-drive precise adjustment
    _PHASE_AIM = 0         # rotate in place until base_link faces the target point
    _PHASE_DRIVE = 1       # drive toward the target point, correcting heading
    _PHASE_FINAL_YAW = 2   # rotate in place to the final target yaw

    def _preciseAdjustActionCallback(self, goal_handle):

        goal = goal_handle.request
        target_x = goal.target_pose.position.x
        target_y = goal.target_pose.position.y
        target_yaw = self._getYawFromQuaternion(goal.target_pose.orientation)

        self.get_logger().info(
            f"PreciseAdjust goal: x={target_x:.3f}, y={target_y:.3f}, yaw={math.degrees(target_yaw):.1f}°")

        # Gains
        xy_kp = self.precise_adjust_xy_pid[0]        # distance -> forward speed
        theta_kp = self.precise_adjust_theta_pid[0]  # heading error -> angular speed

        consecutive_success = 0
        required_success = 5

        # Heading thresholds (rad): must face the target within aim_tol before driving;
        # while driving, fall back to AIM if the bearing drifts beyond realign_tol.
        aim_tol = 0.10           # ~5.7 deg
        realign_tol = 0.35       # ~20 deg
        # Leave FINAL_YAW back to AIM only after clearly drifting out of tolerance.
        reenter_dist = self.precise_adjust_pos_tolerance * 1.5

        phase = self._PHASE_AIM
        pos_error = 0.0
        yaw_error = 0.0

        for _ in range(self.precise_adjust_max_iteration):
            # Cancel check
            if goal_handle.is_cancel_requested:
                self._publishCmdVel(0.0, 0.0)
                goal_handle.canceled()
                result = PreciseAdjustPose.Result()
                result.success = False
                result.message = "Precise adjustment cancelled"
                return result

            pose = self._getMapPose()
            if pose is None:
                self.get_logger().error("Failed to get current pose from TF")
                self._publishCmdVel(0.0, 0.0)
                goal_handle.abort()
                result = PreciseAdjustPose.Result()
                result.success = False
                result.message = "Failed to get current pose from TF"
                return result

            current_x, current_y, current_yaw = pose

            # Position/orientation error in the map frame
            dx = target_x - current_x
            dy = target_y - current_y
            pos_error = math.hypot(dx, dy)
            yaw_error = self._normalizeAngle(target_yaw - current_yaw)
            bearing_error = self._normalizeAngle(math.atan2(dy, dx) - current_yaw)

            pos_reached = pos_error < self.precise_adjust_pos_tolerance
            angle_reached = abs(yaw_error) < self.precise_adjust_angle_tolerance

            # Publish feedback
            feedback = PreciseAdjustPose.Feedback()
            feedback.position_error = float(pos_error)
            feedback.angle_error = float(math.degrees(yaw_error))
            goal_handle.publish_feedback(feedback)

            if pos_reached and angle_reached:
                consecutive_success += 1
                if consecutive_success >= required_success:
                    self._publishCmdVel(0.0, 0.0)
                    msg = (f"Precise adjustment completed: pos_error={pos_error:.3f}m, "
                           f"angle_error={math.degrees(yaw_error):.1f}°")
                    self.get_logger().info(msg)
                    goal_handle.succeed()
                    result = PreciseAdjustPose.Result()
                    result.success = True
                    result.message = msg
                    return result
            else:
                consecutive_success = 0

            # ---- Phase transitions (with hysteresis) ----
            if pos_reached:
                phase = self._PHASE_FINAL_YAW
            elif phase == self._PHASE_FINAL_YAW:
                # Was aligning yaw but drifted off the point: re-aim and drive back in.
                if pos_error > reenter_dist:
                    phase = self._PHASE_AIM
            elif phase == self._PHASE_AIM:
                if abs(bearing_error) < aim_tol:
                    phase = self._PHASE_DRIVE
            elif phase == self._PHASE_DRIVE:
                if abs(bearing_error) > realign_tol:
                    phase = self._PHASE_AIM

            # ---- Per-phase velocity command ----
            if phase == self._PHASE_DRIVE:
                # Drive forward toward the target; heading correction keeps it on bearing.
                vx = xy_kp * pos_error * max(0.0, math.cos(bearing_error))
                vx = max(0.0, min(self.precise_adjust_max_linear_vel, vx))
                if vx > 1e-6 and vx < self.min_linear_speed:
                    vx = self.min_linear_speed
                omega = theta_kp * bearing_error
                omega = max(-self.precise_adjust_max_angular_vel,
                            min(self.precise_adjust_max_angular_vel, omega))
            else:
                # In-place rotation: AIM faces the target point, FINAL_YAW the target yaw.
                if phase == self._PHASE_FINAL_YAW:
                    heading_error = yaw_error
                    reached = angle_reached
                else:
                    heading_error = bearing_error
                    reached = abs(bearing_error) < aim_tol
                vx = 0.0
                omega = theta_kp * heading_error
                omega = max(-self.precise_adjust_max_angular_vel,
                            min(self.precise_adjust_max_angular_vel, omega))
                if reached:
                    omega = 0.0
                elif abs(omega) < self.min_angular_speed:
                    # Overcome the chassis static-friction dead-zone so it actually turns.
                    omega = self.min_angular_speed if omega > 0 else -self.min_angular_speed

            # Publish command (linear.y = 0 for differential drive)
            self._publishCmdVel(vx, omega)
            time.sleep(0.1)  # 10Hz

        # Max iterations reached
        self._publishCmdVel(0.0, 0.0)
        msg = (f"Precise adjustment reached max iterations ({self.precise_adjust_max_iteration}). "
               f"pos_error={pos_error:.3f}m, angle_error={math.degrees(yaw_error):.1f}°")
        self.get_logger().warn(msg)
        goal_handle.abort()
        result = PreciseAdjustPose.Result()
        result.success = False
        result.message = msg
        return result


    def _moveByOdomActionCallback(self, goal_handle):
        """Action callback: route to linear or rotational movement handler."""
        goal = goal_handle.request
        direction = goal.direction
        self.get_logger().info(f"Received move_by_odom goal: distance={goal.distance:.3f}m, "f"direction={direction}, angle={goal.angle:.1f}°")

        try:
            if direction in ('forward', 'backward'):
                return self._odomLinearMove(goal_handle)
            elif direction in ('turn_left', 'turn_right'):
                return self._odomRotate(goal_handle)
            else:
                goal_handle.abort()
                result = MoveByOdom.Result()
                result.success = False
                result.message = (
                    f"Unknown direction: '{direction}'. "
                    f"Two-wheel differential chassis only supports: "
                    f"forward, backward, turn_left, turn_right")
                self.get_logger().error(result.message)
                return result
        finally:
            self._publishCmdVel(0.0, 0.0)

    def _odomLinearMove(self, goal_handle):
        goal = goal_handle.request
        distance = goal.distance
        direction = goal.direction

        linear_speed = abs(self.odom_linear_speed)
        start_x = self.odom_x

        CLOSE_THRESHOLD = 0.1
        RAMP_THRESHOLD = 0.3
        MIN_SPEED = self.min_linear_speed

        start_time = time.time()
        while rclpy.ok():
            # Timeout check
            if time.time() - start_time > self.odom_timeout:
                self.get_logger().warn(f"Linear movement timeout ({self.odom_timeout}s)")
                goal_handle.abort()
                result = MoveByOdom.Result()
                result.success = False
                result.message = f"Movement timeout after {self.odom_timeout}s"
                return result

            # Cancel check
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Linear movement cancelled")
                goal_handle.canceled()
                result = MoveByOdom.Result()
                result.success = False
                result.message = "Movement cancelled"
                return result

            moved = abs(self.odom_x - start_x)

            # Publish feedback
            feedback = MoveByOdom.Feedback()
            feedback.moved_distance = float(moved)
            feedback.rotated_angle = 0.0
            goal_handle.publish_feedback(feedback)

            # Check if target distance reached
            if moved >= abs(distance) - self.odom_position_tolerance:
                self.get_logger().info(f"Linear movement complete! Moved {moved:.3f}m")
                goal_handle.succeed()
                result = MoveByOdom.Result()
                result.success = True
                result.message = f"Moved {direction} {moved:.3f}m"
                return result

            # Ramp-down: gradual deceleration when approaching target
            distance_to_go = abs(distance) - moved
            if distance_to_go < CLOSE_THRESHOLD:
                ramp_speed = MIN_SPEED
            elif distance_to_go < RAMP_THRESHOLD:
                ramp_speed = MIN_SPEED + (linear_speed - MIN_SPEED) * (
                    (distance_to_go - CLOSE_THRESHOLD) / (RAMP_THRESHOLD - CLOSE_THRESHOLD))
            else:
                ramp_speed = linear_speed

            # Map direction to velocity (vy=0 for differential drive)
            if direction == 'forward':
                vx = ramp_speed
            else:  # backward
                vx = -ramp_speed

            self._publishCmdVel(vx, 0.0)
            time.sleep(0.1)

        # Guard: should not reach here
        goal_handle.abort()
        result = MoveByOdom.Result()
        result.success = False
        result.message = "Unexpected exit from linear control loop"
        return result

    def _odomRotate(self, goal_handle):
        goal = goal_handle.request
        angle_deg = goal.angle
        direction = goal.direction

        if direction == 'turn_left':
            sign = 1.0
        elif direction == 'turn_right':
            sign = -1.0
        else:
            self.get_logger().error(f"Unknown rotation direction: {direction}")
            goal_handle.abort()
            result = MoveByOdom.Result()
            result.success = False
            result.message = f"Unknown rotation direction: {direction}"
            return result

        target_angle_rad = math.radians(abs(angle_deg))
        angular_speed = abs(self.odom_angular_speed)

        # Ramp-down thresholds (radians)
        CLOSE_THRESHOLD = 0.1   # ~5.73°
        RAMP_THRESHOLD = 0.3    # ~17.19°
        MIN_SPEED = self.min_angular_speed

        start_th = self.odom_th

        start_time = time.time()
        while rclpy.ok():
            # Timeout check
            if time.time() - start_time > self.odom_timeout:
                self.get_logger().warn(f"Rotation timeout ({self.odom_timeout}s)")
                goal_handle.abort()
                result = MoveByOdom.Result()
                result.success = False
                result.message = f"Rotation timeout after {self.odom_timeout}s"
                return result

            # Cancel check
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Rotation cancelled")
                goal_handle.canceled()
                result = MoveByOdom.Result()
                result.success = False
                result.message = "Rotation cancelled"
                return result

            # Compute angle rotated so far from /odom pose
            rotated = self.odom_th - start_th

            # Publish feedback
            feedback = MoveByOdom.Feedback()
            feedback.moved_distance = 0.0
            feedback.rotated_angle = float(math.degrees(abs(rotated)))
            goal_handle.publish_feedback(feedback)

            # Check if target angle reached
            if abs(rotated) >= target_angle_rad - self.odom_angle_tolerance:
                rotated_deg = math.degrees(abs(rotated))
                self.get_logger().info(f"Rotation complete! Rotated {rotated_deg:.1f}°")
                goal_handle.succeed()
                result = MoveByOdom.Result()
                result.success = True
                result.message = f"Rotated {direction} {abs(angle_deg):.1f}°"
                return result

            # Ramp-down: gradual deceleration when approaching target
            angle_to_go = target_angle_rad - abs(rotated)
            if angle_to_go < CLOSE_THRESHOLD:
                ramp_angular = MIN_SPEED
            elif angle_to_go < RAMP_THRESHOLD:
                ramp_angular = MIN_SPEED + (angular_speed - MIN_SPEED) * (
                    (angle_to_go - CLOSE_THRESHOLD) / (RAMP_THRESHOLD - CLOSE_THRESHOLD))
            else:
                ramp_angular = angular_speed

            angular_cmd = sign * ramp_angular
            self._publishCmdVel(0.0, angular_cmd)
            time.sleep(0.1)

        # Guard: should not reach here
        goal_handle.abort()
        result = MoveByOdom.Result()
        result.success = False
        result.message = "Unexpected exit from rotation control loop"
        return result


def main():
    rclpy.init()
    node = BaseDriverNode('base_driver_node')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
