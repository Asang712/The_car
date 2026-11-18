# author: Yahboom Technology Co., Ltd.
# version: 202606

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import atan2
import sys

# Conversion constant
DEG_TO_RAD = 3.14159265358979323846 / 180.0
RAD_TO_DEG = 180.0 / 3.14159265358979323846


class CalibrateAngularZ(Node):
    def __init__(self):
        super().__init__('calibrate_angular_z')
        
        # Declare parameters
        self.declare_parameter('test_angle_deg', 720.0)  # Test angle in degrees
        self.declare_parameter('angular_speed', 0.35)  # Angular speed in rad/s
        self.declare_parameter('tolerance_deg', 1.0)  # Tolerance in degrees
        tolerance_deg = self.get_parameter('tolerance_deg').get_parameter_value().double_value
        # Get parameters
        test_angle_deg = self.get_parameter('test_angle_deg').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value
        
        self.test_angle = test_angle_deg * DEG_TO_RAD
        self.tolerance = tolerance_deg * DEG_TO_RAD
        
        # Parameter validation
        if test_angle_deg <= 0:
            self.get_logger().error(f"Angle parameter error: test_angle_deg={test_angle_deg}, must be greater than 0")
            sys.exit(1)
        
        if self.angular_speed <= 0:
            self.get_logger().error(f"Angular speed parameter error: angular_speed={self.angular_speed}, must be greater than 0")
            sys.exit(1)
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        odom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom_raw',
            self.odom_callback,
            odom_qos
        )
        
        # State variables
        self.start_yaw = 0.0
        self.current_yaw = 0.0
        self.previous_yaw = 0.0
        self.odom_received = False
        self.test_started = False
        self.test_completed = False
        self.waiting_for_input = False
        self.odom_angle = 0.0
        self.accumulated_angle = 0.0  # Accumulated rotation angle
        
        # Create timer for control loop
        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz
        
        self.get_logger().info('='*60)
        self.get_logger().info('Odometry Angular Velocity Calibration Node Started')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Test angle: {test_angle_deg:.2f} degrees ({self.test_angle:.4f} radians)')
        self.get_logger().info(f'Angular speed: {self.angular_speed:.3f} rad/s')
        self.get_logger().info(f'Allowed error: {tolerance_deg:.2f} degrees ({self.tolerance:.4f} radians)')
        self.get_logger().info('Odometry topic: /odom_raw')
        self.get_logger().info('='*60)

    
    def odom_callback(self, msg):
        """Odometry data callback - extract yaw from quaternion"""
        # Convert quaternion to yaw angle
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.current_yaw = 2.0 * atan2(qz, qw)
        
        if not self.odom_received:
            self.odom_received = True
            self.start_yaw = self.current_yaw
            self.previous_yaw = self.current_yaw
            self.get_logger().info(f'Odometry data received, start yaw: {self.start_yaw:.4f} radians')
    
    def control_loop(self):
        """Main control loop for angular calibration"""
        if not self.odom_received:
            return
        
        if not self.test_started:
            self.start_yaw = self.current_yaw
            self.previous_yaw = self.current_yaw
            self.accumulated_angle = 0.0
            self.test_started = True
        
        delta_yaw = self.current_yaw - self.previous_yaw
    
        while delta_yaw > 3.14159:
            delta_yaw -= 2.0 * 3.14159
        while delta_yaw < -3.14159:
            delta_yaw += 2.0 * 3.14159
        
        # Accumulate the angle (always positive for counter-clockwise rotation)
        self.accumulated_angle += abs(delta_yaw)
        
        # Update previous yaw
        self.previous_yaw = self.current_yaw
        
        # Use accumulated angle for calibration
        rotated_angle = self.accumulated_angle
        
        # Calculate error
        error = self.test_angle - rotated_angle
        
        # Check if target reached
        if abs(error) < self.tolerance and not self.test_completed:
            self.test_completed = True
            self.get_logger().info('='*60)
            self.get_logger().info('Calibration test rotation completed!')
            self.get_logger().info('='*60)
            # Stop robot
            self.cmd_vel_pub.publish(Twist())
            self.timer.cancel()
            # Display odometry reading
            self.odom_angle = rotated_angle
            odom_angle_deg = self.odom_angle * RAD_TO_DEG
            self.get_logger().info(f'Odometry reading: {odom_angle_deg:.2f} degrees ({self.odom_angle:.4f} radians)')
            self.get_logger().info('='*60)
            self.get_logger().info('Enter actual rotation angle...')
            
            # Set flag to notify main function to wait for user input
            self.waiting_for_input = True
            return

        # Target not reached, continue rotating
        move_cmd = Twist()
        angle_to_go = abs(error)
        
        # Angular velocity ramping: reduce speed when approaching target to prevent overshoot
        angle_to_go_deg = abs(error) * RAD_TO_DEG
        
        if angle_to_go_deg < 5.73:  # Very close (< 5.73 degrees ≈ 0.1 rad), use minimum speed
            ramp_speed = 0.1
        elif angle_to_go_deg < 17.19:  # Approaching (< 17.19 degrees ≈ 0.3 rad), linear deceleration
            ramp_speed = 0.1 + (self.angular_speed - 0.1) * ((angle_to_go_deg - 5.73) / 11.46)
        else:  # Far away, full speed
            ramp_speed = self.angular_speed
        
        # Ensure minimum angular speed of 0.1 rad/s, otherwise the robot may not rotate
        ramp_speed = max(ramp_speed, 0.15)
        
        # Rotate counter-clockwise (positive angular.z)
        move_cmd.angular.z = ramp_speed
        self.cmd_vel_pub.publish(move_cmd)

def main():
    rclpy.init()
    node = CalibrateAngularZ()
    
    try:
        # Run node until test completes
        while rclpy.ok() and not node.waiting_for_input:
            rclpy.spin_once(node, timeout_sec=0.1)
        
        # Test completed, wait for user input
        if node.waiting_for_input:
            try:
                print('\n' + '='*60)
                actual_angle_deg_str = input('Please enter actual rotation angle (degrees): ')
                actual_angle_deg = float(actual_angle_deg_str)
                
                if actual_angle_deg <= 0:
                    node.get_logger().error('Actual angle must be greater than 0')
                else:
                    # Convert to radians for calculation
                    actual_angle = actual_angle_deg * DEG_TO_RAD
                    
                    # Calculate correction factor
                    if node.odom_angle > 0:
                        correction_factor = actual_angle / node.odom_angle
                        odom_angle_deg = node.odom_angle * RAD_TO_DEG
                        
                        node.get_logger().info('='*60)
                        node.get_logger().info('Calibration Results')
                        node.get_logger().info('='*60)
                        node.get_logger().info(f'Odometry reading: {odom_angle_deg:.2f} degrees ({node.odom_angle:.4f} radians)')
                        node.get_logger().info(f'Actual angle: {actual_angle_deg:.2f} degrees ({actual_angle:.4f} radians)')
                        node.get_logger().info(f'Correction factor: {correction_factor:.4f}')
                        node.get_logger().info('='*60)
                    else:
                        node.get_logger().warn('Odometry reading is 0, cannot calculate correction factor')
            except ValueError:
                node.get_logger().error('Invalid input, please enter a number')
            except KeyboardInterrupt:
                node.get_logger().info('User cancelled input')
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure robot stops
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
        

if __name__ == '__main__':
    main()
