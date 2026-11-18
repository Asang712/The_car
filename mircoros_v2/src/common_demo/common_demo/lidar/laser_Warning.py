#ros lib
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt16
#commom lib
import math
import numpy as np
import os

# TF2 imports
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped

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

class laserWarning(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info('Initializing laser_Warning node with TF2 support...')
        
        #create a sub
        self.sub_laser = self.create_subscription(LaserScan,"/scan",self.registerScan,qos_profile_sensor_data)
        #create a pub
        self.pub_vel = self.create_publisher(Twist,'/cmd_vel',1)
        self.pub_Buzzer = self.create_publisher(UInt16,'/beep',1)

        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Wait a moment for TF listener to initialize
        import time
        time.sleep(1)
        
        #declareparam
        self.declare_parameter("linear",0.15)
        self.linear = self.get_parameter('linear').get_parameter_value().double_value
        self.declare_parameter("angular",1.0)
        self.angular = self.get_parameter('angular').get_parameter_value().double_value
        self.declare_parameter("LaserAngle",45.0)
        self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().double_value
        self.declare_parameter("ResponseDist",0.55)
        self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value
        
        # PID parameters
        self.declare_parameter("ang_pid_P", 3.0)
        self.declare_parameter("ang_pid_I", 0.0)
        self.declare_parameter("ang_pid_D", 5.0)
        
        # Angular velocity deadzone and limits
        self.declare_parameter("angular_deadzone", 0.5)
        self.declare_parameter("max_angular", 2.0)
        
        # Log interval
        self.log_interval = 1.0
        
        self.laser_frame = ""
        
        # Initialize PID controller
        ang_pid_P = self.get_parameter('ang_pid_P').get_parameter_value().double_value
        ang_pid_I = self.get_parameter('ang_pid_I').get_parameter_value().double_value
        ang_pid_D = self.get_parameter('ang_pid_D').get_parameter_value().double_value
        
        self.ang_pid = SinglePID(ang_pid_P, ang_pid_I, ang_pid_D)
        
        # Get other parameters
        self.angular_deadzone = self.get_parameter('angular_deadzone').get_parameter_value().double_value
        self.max_angular = self.get_parameter('max_angular').get_parameter_value().double_value
        
        self.tf_ready = False
        self.buzzer_active = False  # Track buzzer state to reduce logging
        self.last_log_time = 0
        self.last_state_log_time = 0
        
        # Start TF waiting task
        self.create_timer(0.5, self.wait_for_tf)
        
        self.get_logger().info('laser_Warning node initialized successfully, guard mode active')
    
    def wait_for_tf(self):
        """Wait for TF transform to be available"""
        if self.tf_ready:
            return
        
        # Auto-detect laser frame_id from scan data if not set
        if not self.laser_frame:
            # Try to get laser frame from a temporary subscription or use default
            # For now, we'll wait until first scan message
            return
        
        try:
            # Check if transform is available
            self.tf_buffer.lookup_transform('base_link', self.laser_frame, rclpy.time.Time())
            if not self.tf_ready:
                self.get_logger().info(f'✓ TF ready: {self.laser_frame} → base_link')
                self.tf_ready = True
        except TransformException as ex:
            current_time = self.get_clock().now().nanoseconds / 1e9
            if (current_time - self.last_log_time) >= 3.0:
                self.get_logger().warn(f'Waiting for TF: {self.laser_frame} → base_link ({ex})')
                self.last_log_time = current_time
            return

    def registerScan(self, scan_data):
        if not isinstance(scan_data, LaserScan): return
        
        # Auto-detect laser frame_id
        if not self.laser_frame and scan_data.header.frame_id:
            self.laser_frame = scan_data.header.frame_id
            self.get_logger().info(f'Detected laser frame: {self.laser_frame}')
        
        # Check if TF is ready
        if not self.tf_ready or not self.laser_frame:
            return
        
        # Check if transform is still available
        try:
            trans = self.tf_buffer.lookup_transform(
                'base_link',
                self.laser_frame,
                rclpy.time.Time()
            )
        except TransformException as ex:
            current_time = self.get_clock().now().nanoseconds / 1e9
            if (current_time - self.last_log_time) >= 3.0:
                self.get_logger().warn(f'TF transform error: {ex}')
                self.last_log_time = current_time
            return
        
        # Get laser data
        ranges = np.array(scan_data.ranges)
        
        # Find the nearest obstacle in front using TF2
        min_dist = float('inf')
        nearest_x = 0.0
        nearest_y = 0.0
        nearest_angle = 0.0
        found_obstacle = False
        
        for i in range(len(ranges)):
            # Skip invalid measurements
            if ranges[i] <= 0.0 or ranges[i] > scan_data.range_max:
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
                
                # Check if point is within LaserAngle in front
                dist = math.sqrt(x**2 + y**2)
                angle_to_base = math.atan2(y, x) * RAD2DEG
                
                if abs(angle_to_base) < self.LaserAngle:
                    # Find nearest obstacle
                    if dist < min_dist:
                        min_dist = dist
                        nearest_x = x
                        nearest_y = y
                        nearest_angle = angle_to_base
                        found_obstacle = True
            except Exception as e:
                continue
        
        if not found_obstacle:
            # No obstacle found, stop robot
            velocity = Twist()
            self.pub_vel.publish(velocity)
            # Turn off buzzer
            if self.buzzer_active:
                buzzer_msg = UInt16()
                buzzer_msg.data = 0
                self.pub_Buzzer.publish(buzzer_msg)
                self.buzzer_active = False
            return
        
        # Log nearest obstacle info (throttled)
        current_time = self.get_clock().now().nanoseconds / 1e9
        if (current_time - self.last_log_time) >= self.log_interval:
            self.get_logger().info(
                f'Nearest: dist={min_dist:.2f}m, '
                f'x={nearest_x:.2f}m, y={nearest_y:.2f}m, '
                f'angle={nearest_angle:.1f}°'
            )
            self.last_log_time = current_time
        
        # Control buzzer based on distance
        if min_dist <= self.ResponseDist:
            if not self.buzzer_active:
                buzzer_msg = UInt16()
                buzzer_msg.data = 1
                self.pub_Buzzer.publish(buzzer_msg)
                self.buzzer_active = True
        else:
            if self.buzzer_active:
                buzzer_msg = UInt16()
                buzzer_msg.data = 0
                self.pub_Buzzer.publish(buzzer_msg)
                self.buzzer_active = False
        
        # Calculate angular velocity to track the nearest obstacle
        # Target angle is 0 (aligned with x-axis)
        angle_rad = math.radians(nearest_angle)
        ang_pid_compute = self.ang_pid.pid_compute(0, angle_rad)
        
        velocity = Twist()
        
        # Reverse angular velocity direction
        velocity.angular.z = -ang_pid_compute
        
        # Apply deadzone
        if abs(ang_pid_compute) < self.angular_deadzone:
            velocity.angular.z = 0.0
        
        # Clamp angular velocity
        if abs(velocity.angular.z) > self.max_angular:
            velocity.angular.z = self.max_angular * (1.0 if velocity.angular.z > 0 else -1.0)
        
        # Publish velocity
        self.pub_vel.publish(velocity)

def main():
    rclpy.init()
    laser_warn = laserWarning("laser_Warnning")
    try:
        rclpy.spin(laser_warn)
    except KeyboardInterrupt:
        pass
    finally:
        laser_warn.destroy_node()
        rclpy.shutdown()
