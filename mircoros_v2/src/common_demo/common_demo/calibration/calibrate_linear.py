# author: Yahboom Technology Co., Ltd.
# version: 202607

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import copysign
import sys

class CalibrateLinear(Node):
    def __init__(self):
        super().__init__('calibrate_linear')
        
        # Declare parameters with explicit types
        self.declare_parameter('test_distance', 1.5)
        self.declare_parameter('speed',  0.2)
        self.declare_parameter('tolerance',  0.005)
        
        # Get parameters
        self.test_distance = self.get_parameter('test_distance').get_parameter_value().double_value
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        self.tolerance = self.get_parameter('tolerance').get_parameter_value().double_value
        
        # Parameter validation
        if self.test_distance <= 0:
            self.get_logger().error(f"Distance parameter error: test_distance={self.test_distance}, must be greater than 0")
            sys.exit(1)
        
        if self.speed <= 0:
            self.get_logger().error(f"Speed parameter error: speed={self.speed}, must be greater than 0")
            sys.exit(1)
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # State variables
        self.start_x = 0.0
        self.current_x = 0.0
        self.odom_received = False
        self.test_started = False
        self.test_completed = False
        self.waiting_for_input = False
        self.odom_distance = 0.0
        
        # Create timer for control loop
        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz
        
        self.get_logger().info('='*60)
        self.get_logger().info('Odometer Linear Velocity Calibration Node Started')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Test distance: {self.test_distance:.2f} meters')
        self.get_logger().info(f'Movement speed: {self.speed:.2f} m/s')
        self.get_logger().info(f'Allowed error: {self.tolerance:.3f} meters')
        self.get_logger().info('='*60)

    def odom_callback(self, msg):
        """Odometry data callback"""
        self.current_x = msg.pose.pose.position.x
        
        if not self.odom_received:
            self.odom_received = True
            self.start_x = self.current_x
            self.get_logger().info(f'Odometry data received, start position: x={self.start_x:.4f}')
    
    def control_loop(self):
        if not self.odom_received:
            return
        
        if not self.test_started:
            self.start_x = self.current_x
            self.test_started = True
        
        # Calculate moved distance along x axis
        moved_distance = self.current_x - self.start_x
        # Calculate error
        error = self.test_distance - abs(moved_distance)
        
        # Check if target reached
        if abs(error) < self.tolerance and not self.test_completed:
            self.test_completed = True
            self.get_logger().info('='*60)
            self.get_logger().info('Calibration test movement completed!')
            self.get_logger().info('='*60)
            # Stop robot
            self.cmd_vel_pub.publish(Twist())
            self.timer.cancel()
            # Display odometry reading
            self.odom_distance = abs(moved_distance)
            self.get_logger().info(f'Odometry reading: {self.odom_distance:.4f} meters')
            self.get_logger().info('='*60)
            self.get_logger().info('Enter actual movement distance...')
            
            # Set flag to notify main function to wait for user input
            self.waiting_for_input = True
            return

        # Target not reached, continue moving
        move_cmd = Twist()
        distance_to_go = abs(error)
        
        # Velocity ramping: reduce speed when approaching target to prevent overshoot
        if distance_to_go < 0.1:  # Very close, use minimum speed
            ramp_speed = 0.05
        elif distance_to_go < 0.3:  # Approaching, linear deceleration
            ramp_speed = 0.05 + (self.speed - 0.05) * ((distance_to_go - 0.1) / 0.2)
        else:  # Far away, full speed
            ramp_speed = self.speed
        
        move_cmd.linear.x = copysign(ramp_speed, error)
        
        self.cmd_vel_pub.publish(move_cmd)


def main():
    rclpy.init()
    node = CalibrateLinear()
    
    try:
        # Run node until test completes
        while rclpy.ok() and not node.waiting_for_input:
            rclpy.spin_once(node, timeout_sec=0.1)
        
        # Test completed, wait for user input
        if node.waiting_for_input:
            try:
                print('\n' + '='*60)
                actual_distance_str = input('Please enter actual movement distance (meters): ')
                actual_distance = float(actual_distance_str)
                
                if actual_distance <= 0:
                    node.get_logger().error('Actual distance must be greater than 0')
                else:
                    # Calculate correction factor
                    if node.odom_distance > 0:
                        correction_factor = actual_distance / node.odom_distance
                        
                        node.get_logger().info('='*60)
                        node.get_logger().info('Calibration Results')
                        node.get_logger().info('='*60)
                        node.get_logger().info(f'Odometry reading: {node.odom_distance:.4f} meters')
                        node.get_logger().info(f'Actual distance: {actual_distance:.4f} meters')
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
