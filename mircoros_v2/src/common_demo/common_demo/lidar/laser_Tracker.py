#ros lib
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_point
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

#commom lib
import math
import numpy as np
import time
from time import sleep

import os
RAD2DEG = 180 / math.pi
class SinglePID:
    def __init__(self, P=0.1, I=0.0, D=0.1):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        print("init_pid: ", P, I, D)
        self.pid_reset()

    def Set_pid(self, P, I, D):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        print("set_pid: ", P, I, D)
        self.pid_reset()

    def pid_compute(self, target, current):
        self.error = target - current
        self.intergral += self.error
        self.derivative = self.error - self.prevError
        result = self.Kp * self.error + self.Ki * self.intergral + self.Kd * self.derivative
        self.prevError = self.error
        return result

    def pid_reset(self):
        self.error = 0
        self.intergral = 0
        self.derivative = 0
        self.prevError = 0

class laserTracker(Node):
    def __init__(self,name):
        super().__init__(name)
        #create a sub
        self.sub_laser = self.create_subscription(LaserScan,"/scan",self.registerScan,qos_profile_sensor_data)
        #create a pub
        self.pub_vel = self.create_publisher(Twist,'/cmd_vel',1)
        
        # TF2 components
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        #declareparam
        self.declare_parameter("linear", 0.15)
        self.linear = self.get_parameter('linear').get_parameter_value().double_value
        self.declare_parameter("angular", 1.0)
        self.angular = self.get_parameter('angular').get_parameter_value().double_value
        self.declare_parameter("LaserAngle", 45.0)
        self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().double_value
        self.declare_parameter("ResponseDist", 0.5)
        self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value
        self.declare_parameter("enable", False)  # Enable parameter for tracking control
        
        # PID parameters
        self.declare_parameter("lin_pid_P", 1.0)
        self.declare_parameter("lin_pid_I", 0.0)
        self.declare_parameter("lin_pid_D", 1.0)
        self.declare_parameter("ang_pid_P", 2.0)
        self.declare_parameter("ang_pid_I", 0.0)
        self.declare_parameter("ang_pid_D", 2.0)
        
        # Angular velocity deadzone and limits
        self.declare_parameter("angular_deadzone", 0.47)
        self.declare_parameter("max_angular", 2.0)
        
        self.laser_frame = ""
        
        # Initialize PID controllers
        lin_pid_P = self.get_parameter('lin_pid_P').get_parameter_value().double_value
        lin_pid_I = self.get_parameter('lin_pid_I').get_parameter_value().double_value
        lin_pid_D = self.get_parameter('lin_pid_D').get_parameter_value().double_value
        ang_pid_P = self.get_parameter('ang_pid_P').get_parameter_value().double_value
        ang_pid_I = self.get_parameter('ang_pid_I').get_parameter_value().double_value
        ang_pid_D = self.get_parameter('ang_pid_D').get_parameter_value().double_value
        
        self.lin_pid = SinglePID(lin_pid_P, lin_pid_I, lin_pid_D)
        self.ang_pid = SinglePID(ang_pid_P, ang_pid_I, ang_pid_D)
        
        # Get other parameters
        self.angular_deadzone = self.get_parameter('angular_deadzone').get_parameter_value().double_value
        self.max_angular = self.get_parameter('max_angular').get_parameter_value().double_value
        self.log_interval = 1.0
        
        self.tf_ready = False
        self.is_enabled = False  # Internal tracking state
        
        # Log throttling
        self.last_log_time = 0.0
        self.last_tf_log_time = 0.0
        self.tf_log_interval = 3.0  # TF waiting log at most once every 3 seconds
        self.last_state_log_time = 0.0
        
        # Add parameter callback for dynamic reconfiguration
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.get_logger().info('Tracker initialized. Use "ros2 param set /laser_Tracker enable true" to start tracking')
    
    def parameter_callback(self, params):
        """Callback for parameter changes"""
        for param in params:
            if param.name == "enable":
                new_state = param.value
                if new_state and not self.is_enabled:
                    # Enable tracking
                    self.is_enabled = True
                    self.lin_pid.pid_reset()
                    self.ang_pid.pid_reset()
                    current_time = self.get_clock().now().nanoseconds / 1e9
                    if (current_time - self.last_state_log_time) >= 1.0:
                        self.get_logger().info('✓ Tracking ENABLED')
                        self.last_state_log_time = current_time
                elif not new_state and self.is_enabled:
                    # Disable tracking
                    self.is_enabled = False
                    stop_cmd = Twist()
                    self.pub_vel.publish(stop_cmd)
                    current_time = self.get_clock().now().nanoseconds / 1e9
                    if (current_time - self.last_state_log_time) >= 1.0:
                        self.get_logger().info('✗ Tracking DISABLED - Robot stopped')
                        self.last_state_log_time = current_time
        
        result = SetParametersResult()
        result.successful = True
        return result
        
    def wait_for_tf(self):
        """Wait for TF transform to be available"""
        self.get_logger().info('Waiting for TF transform listener to be ready...')
        
        # Wait for TransformListener to cache TF data
        time.sleep(1.0)
        # If laser_frame parameter is empty, wait for first scan data to detect actual frame_id
        if not self.laser_frame:
            self.get_logger().info('laser_frame parameter not set, will detect from /scan topic')
            # Wait a short time for sub_laser callback to receive data
            wait_count = 0
            while not self.laser_frame and wait_count < 10:
                time.sleep(0.5)
                wait_count += 1
            
            if not self.laser_frame:
                self.get_logger().error('Failed to detect laser frame from /scan topic')
                return
        
        # Wait for TF transform to be available
        while rclpy.ok():
            try:
                self.tf_buffer.lookup_transform(
                    'base_link',
                    self.laser_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                self.get_logger().info(f'TF available: base_link <- {self.laser_frame}')
                self.tf_ready = True
                return
            except TransformException:
                # Throttle TF waiting logs
                current_time = self.get_clock().now().nanoseconds / 1e9
                if (current_time - self.last_tf_log_time) >= self.tf_log_interval:
                    self.get_logger().info(f'Waiting for TF: base_link <- {self.laser_frame}...')
                    self.last_tf_log_time = current_time
                time.sleep(0.5)
        

    def registerScan(self, scan_data):
        if not isinstance(scan_data, LaserScan): return
        
        # Skip processing if not enabled
        if not self.is_enabled:
            return
        
        # Auto-detect laser frame_id
        if not self.laser_frame:
            self.laser_frame = scan_data.header.frame_id
            self.get_logger().info(f'Detected laser frame: {self.laser_frame}')
        
        # If TF is not ready, try to establish connection
        if not self.tf_ready:
            try:
                self.tf_buffer.lookup_transform(
                    'base_link',
                    scan_data.header.frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                self.tf_ready = True
                self.get_logger().info(f'TF ready: base_link <- {scan_data.header.frame_id}')
            except TransformException:
                # TF not ready, skip this scan
                return
        
        # Get transform from laser to base_link
        try:
            trans = self.tf_buffer.lookup_transform(
                'base_link',
                scan_data.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except TransformException:
            # Throttle TF error logs
            current_time = self.get_clock().now().nanoseconds / 1e9
            if (current_time - self.last_tf_log_time) >= self.tf_log_interval:
                self.get_logger().warn('TF not available, skipping scan')
                self.last_tf_log_time = current_time
            return
        
        # Iterate through all laser points and find closest obstacle in front
        ranges = np.array(scan_data.ranges)
        minDistList = []
        minDistBaseList = []  # Store positions in base_link frame
        
        for i in range(len(ranges)):
            if ranges[i] == 0.0 or ranges[i] > scan_data.range_max:
                continue
                
            # Calculate laser point position in laser coordinate frame
            angle = scan_data.angle_min + scan_data.angle_increment * i
            point_in_laser = PointStamped()
            point_in_laser.header = scan_data.header
            point_in_laser.point.x = ranges[i] * math.cos(angle)
            point_in_laser.point.y = ranges[i] * math.sin(angle)
            point_in_laser.point.z = 0.0
            
            # Transform to base_link coordinate frame
            try:
                point_in_base = do_transform_point(point_in_laser, trans)
                x = point_in_base.point.x
                y = point_in_base.point.y
                
                # Only consider obstacles in front (within LaserAngle)
                dist = math.sqrt(x**2 + y**2)
                angle_to_base = math.atan2(y, x) * RAD2DEG
                
                if abs(angle_to_base) < self.LaserAngle:
                    minDistList.append(dist)
                    minDistBaseList.append((x, y, angle_to_base))
            except TransformException:
                continue
        
        if len(minDistList) == 0:
            return
        
        # Find the closest obstacle
        minDist = min(minDistList)
        minIdx = minDistList.index(minDist)
        min_x, min_y, min_angle = minDistBaseList[minIdx]
        
        # PID control for tracking
        velocity = Twist()
        
        # Distance deadzone
        if abs(minDist - self.ResponseDist) < 0.1:
            minDist = self.ResponseDist
        
        # Linear velocity PID (maintain target distance)
        velocity.linear.x = -self.lin_pid.pid_compute(self.ResponseDist, minDist)
        
        # Angular velocity PID (align with obstacle)
        # Use angle directly in radians for better sensitivity
        angle_rad = math.radians(min_angle)
        ang_pid_compute = self.ang_pid.pid_compute(0, angle_rad)  # Target is 0 (aligned), current is angle_rad
        
        velocity.angular.z = -ang_pid_compute
        
        # Reduced deadzone for better responsiveness
        if abs(ang_pid_compute) < self.angular_deadzone:
            velocity.angular.z = 0.0
        
        # Clamp angular velocity to safe limits
        if abs(velocity.angular.z) > self.max_angular:
            velocity.angular.z = self.max_angular * (1.0 if velocity.angular.z > 0 else -1.0)
        
        # Throttle logging
        current_time = self.get_clock().now().nanoseconds / 1e9
        if (current_time - self.last_log_time) >= self.log_interval:
            self.get_logger().info(
                f'Tracking: dist={minDist:.2f}m, angle={min_angle:.1f}deg, '
                f'linear={velocity.linear.x:.2f}, angular={velocity.angular.z:.2f}'
            )
            self.last_log_time = current_time
        
        self.pub_vel.publish(velocity)

def main():
    rclpy.init()
    laser_tracker = laserTracker("laser_Tracker")

    if laser_tracker.laser_frame:
        laser_tracker.wait_for_tf()
    
    try:
        rclpy.spin(laser_tracker)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot
        twist = Twist()
        laser_tracker.pub_vel.publish(twist)
        laser_tracker.destroy_node()
        rclpy.shutdown()
