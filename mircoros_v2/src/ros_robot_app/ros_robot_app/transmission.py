#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan, Image, CompressedImage
from cv_bridge import CvBridge
import tf2_ros
import math
import os
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class TransmissionNode(Node):
    """Integrated node for LaserScan to Point conversion and RGB Image compression"""

    def __init__(self):
        super().__init__('transmission_node')
        
        self.get_logger().info('Initializing LaserScan to Point conversion...')
        
        # Subscribe to LaserScan data (sensor data uses BEST_EFFORT QoS)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.laser_subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.laserscan_callback,
            sensor_qos
        )
        
        # Publish laser point path
        self.scan_point_publisher = self.create_publisher(Path, '/scan_points', 10)
        
        # Declare and get flip parameter
        self.declare_parameter('flip_laser_points', False)
        self.flip_laser_points = self.get_parameter('flip_laser_points').get_parameter_value().bool_value
        
        self.get_logger().info(f'LaserScan to Point conversion initialized (flip: {self.flip_laser_points})')
        
        # ========== RGB Image Compression ==========
        self.get_logger().info('Initializing RGB Image compression...')
                
        # Subscribe to raw RGB image
        self.rgb_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.rgb_image_callback,
            sensor_qos
        )
        
        # CvBridge for image format conversion
        self.rgb_bridge = CvBridge()
        
        # Publish compressed image
        self.rgb_img_pub = self.create_publisher(
            CompressedImage,
            "/camera/rgb/image_raw/compressed",
            10
        )
        # JPEG compression quality (0-100)
        self.jpeg_quality = 80
        
    # ========== LaserScan to Point Conversion ==========
    
    def laserscan_callback(self, msg):
        """Process LaserScan data"""
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        laserscan = msg.ranges
        
        laser_points = self.laserscan_to_points(laserscan, angle_min, angle_increment, msg.header)
        self.scan_point_publisher.publish(laser_points)
        
    def laserscan_to_points(self, laserscan, angle_min, angle_increment, header):
        """Convert LaserScan data to point cloud path"""
        points = []
        angle = angle_min
        laser_points = Path()
        
        laser_points.header = header
        laser_points.header.frame_id = 'laser_frame'
        
        # Add 180° offset if flip is enabled to compensate for laser_link to laser TF rotation
        angle_offset = math.pi if self.flip_laser_points else 0.0

        for distance in laserscan:
            # Filter invalid distances (infinity, NaN, out of range)
            if math.isinf(distance) or math.isnan(distance) or distance <= 0:
                angle += angle_increment
                continue
                
            # Convert polar coordinates to Cartesian coordinates with optional 180° offset
            adjusted_angle = angle + angle_offset
            x = distance * math.cos(adjusted_angle)
            y = distance * math.sin(adjusted_angle)
            
            pose = PoseStamped()
            pose.header = laser_points.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            points.append(pose)
            
            angle += angle_increment

        laser_points.poses = points
        return laser_points
     
    # ========== RGB Image Compression ==========
    
    def rgb_image_callback(self, msg):
        """Process RGB image and publish compressed version"""
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.rgb_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Set JPEG compression parameters
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            
            # Compress image
            result, encoded_img = cv2.imencode('.jpg', cv_image, encode_param)
            
            if not result:
                self.get_logger().error("Failed to compress image")
                return
            
            # Create compressed image message
            compressed_msg = CompressedImage()
            compressed_msg.header = msg.header
            compressed_msg.format = 'jpeg'
            compressed_msg.data = encoded_img.tobytes()
            
            # Publish compressed image
            self.rgb_img_pub.publish(compressed_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error in RGB image callback: {e}")


def main(args=None):
    rclpy.init(args=args)

    transmission_node = TransmissionNode()

    try:
        rclpy.spin(transmission_node)
    except KeyboardInterrupt:
        pass
    finally:
        transmission_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
