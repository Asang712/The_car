# author: Yahboom Technology Co., Ltd.
# version: 202606

#ros lib
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_point

#commom lib
import math
import numpy as np
import time
from time import sleep
import os

RAD2DEG = 180 / math.pi

class laserAvoid(Node):
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
        self.declare_parameter("linear",0.15)
        self.linear = self.get_parameter('linear').get_parameter_value().double_value
        self.declare_parameter("angular",1.0)
        self.angular = self.get_parameter('angular').get_parameter_value().double_value
        self.declare_parameter("LaserAngle",30.0)
        self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().double_value
        self.declare_parameter("ResponseDist",0.3)
        self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value
        self.laser_frame =""
        
        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0
        self.conut = 10
        self.tf_ready = False

        self.turn_dir = 0
        
        # Log throttling
        self.last_log_time = 0.0
        self.log_interval = 2.0  # Log at most once every 2 seconds
        self.last_obstacle_state = -1  # Track obstacle state to log only on change
        
        # TF waiting log throttling
        self.last_tf_log_time = 0.0
        self.tf_log_interval = 3.0  # TF waiting log at most once every 3 seconds

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
        
        # Reset counters
        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0
        
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
        
        # Iterate through all laser points
        ranges = np.array(scan_data.ranges)
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
                
                # Determine obstacle position based on base_link coordinate frame
                dist = math.sqrt(x**2 + y**2)
                if dist <= self.ResponseDist * 1.5:
                    # Use atan2 to calculate angle relative to base_link x-axis
                    angle_to_base = math.atan2(y, x) * RAD2DEG

                    if abs(angle_to_base) < self.LaserAngle:
                        self.front_warning += 1
                    elif self.LaserAngle <= angle_to_base < 90.0:
                        self.Left_warning += 1
                    elif -90.0 < angle_to_base <= -self.LaserAngle:
                        self.Right_warning += 1
            except TransformException:
                continue
        twist = Twist()
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Determine current obstacle state
        front_blocked = self.front_warning > self.conut
        left_blocked = self.Left_warning > self.conut
        right_blocked = self.Right_warning > self.conut
        
        if not front_blocked:
            # Front is clear: side obstacles do not block forward motion,
            # just go straight and release the turn direction lock
            self.turn_dir = 0
            obstacle_state = 10
            twist.linear.x = self.linear
            twist.angular.z = 0.0
        else:
            # Front blocked: rotate in place. Pick a direction once and keep it
            # until the front is clear again, preventing left/right oscillation
            if self.turn_dir == 0:
                if left_blocked and not right_blocked:
                    self.turn_dir = -1   # obstacle on left, turn right
                elif right_blocked and not left_blocked:
                    self.turn_dir = 1    # obstacle on right, turn left
                else:
                    self.turn_dir = 1 if self.Left_warning <= self.Right_warning else -1
            obstacle_state = 6 if self.turn_dir > 0 else 4
            twist.linear.x = 0.0
            twist.angular.z = self.turn_dir * self.angular
        
        # Log only when state changes or after log_interval
        if obstacle_state != self.last_obstacle_state or (current_time - self.last_log_time) >= self.log_interval:
            log_messages = {
                4: 'Obstacle in front, turning right',
                6: 'Obstacle in front, turning left',
                10: 'Front clear, going forward'
            }
            if obstacle_state in log_messages:
                self.get_logger().info(log_messages[obstacle_state])
            self.last_log_time = current_time
            self.last_obstacle_state = obstacle_state
        
        self.pub_vel.publish(twist)


def main():
    rclpy.init()
    laser_avoid = laserAvoid("laserAvoid")

    if laser_avoid.laser_frame:
        laser_avoid.wait_for_tf()
    
    try:
        rclpy.spin(laser_avoid)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot
        twist = Twist()
        laser_avoid.pub_vel.publish(twist)
        laser_avoid.destroy_node()
        rclpy.shutdown()


