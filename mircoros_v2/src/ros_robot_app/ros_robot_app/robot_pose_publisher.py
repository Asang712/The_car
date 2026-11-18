#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException
import math


class RobotPosePublisher(Node):
    def __init__(self):
        super().__init__('robot_pose_publisher')
        
        # Create TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Declare parameters
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('is_stamped', False)
        
        # Get parameters
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.is_stamped = self.get_parameter('is_stamped').get_parameter_value().bool_value
        
        # Create publisher based on is_stamped parameter
        if self.is_stamped:
            self.publisher = self.create_publisher(PoseStamped, 'robot_pose', 1)
        else:
            self.publisher = self.create_publisher(Pose, 'robot_pose', 1)
        
        # Create timer (50ms = 20Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        self.get_logger().info(f'Robot Pose Publisher started')
        self.get_logger().info(f'  map_frame: {self.map_frame}')
        self.get_logger().info(f'  base_frame: {self.base_frame}')
        self.get_logger().info(f'  is_stamped: {self.is_stamped}')
    
    def timer_callback(self):
        try:
            # Lookup transform from map to base_link
            # Use latest available transform (time=None or rclpy.time.Time())
            transform_stamped = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time()
            )
            
            # Create pose from transform
            if self.is_stamped:
                pose_msg = PoseStamped()
                pose_msg.header.frame_id = self.map_frame
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                
                pose_msg.pose.position.x = transform_stamped.transform.translation.x
                pose_msg.pose.position.y = transform_stamped.transform.translation.y
                pose_msg.pose.position.z = transform_stamped.transform.translation.z
                
                pose_msg.pose.orientation.x = transform_stamped.transform.rotation.x
                pose_msg.pose.orientation.y = transform_stamped.transform.rotation.y
                pose_msg.pose.orientation.z = transform_stamped.transform.rotation.z
                pose_msg.pose.orientation.w = transform_stamped.transform.rotation.w
                
                self.publisher.publish(pose_msg)
            else:
                pose_msg = Pose()
                pose_msg.position.x = transform_stamped.transform.translation.x
                pose_msg.position.y = transform_stamped.transform.translation.y
                pose_msg.position.z = transform_stamped.transform.translation.z
                
                pose_msg.orientation.x = transform_stamped.transform.rotation.x
                pose_msg.orientation.y = transform_stamped.transform.rotation.y
                pose_msg.orientation.z = transform_stamped.transform.rotation.z
                pose_msg.orientation.w = transform_stamped.transform.rotation.w
                
                self.publisher.publish(pose_msg)
                
        except LookupException as e:
            # TF not available yet, silently skip
            pass
        except Exception as e:
            self.get_logger().warn(f'Error looking up transform: {e}')


def main(args=None):
    rclpy.init(args=args)
    robot_pose_publisher = RobotPosePublisher()
    
    try:
        rclpy.spin(robot_pose_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        robot_pose_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
