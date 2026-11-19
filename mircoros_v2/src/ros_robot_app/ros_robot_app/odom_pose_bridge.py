#!/usr/bin/env python3
"""把 /odom_raw 的里程计转成手机 App 需要的 /robot_pose，同时补齐 TF 树。

厂家原本的节点里缺少 odom -> base_footprint 这一环，导致 App 建图模块
拿不到位姿，地图区一直是空白。
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class OdomPoseBridge(Node):
    def __init__(self):
        super().__init__('odom_pose_bridge')

        self.declare_parameter('odom_topic', '/odom_raw')
        self.declare_parameter('pose_topic', '/robot_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('laser_frame', 'laser_frame')
        # 运行 SLAM 时须为 False，把 map->odom 交给 slam_toolbox 发布，避免 TF 冲突
        self.declare_parameter('publish_map_odom', True)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.laser_frame = self.get_parameter('laser_frame').value
        self.publish_map_odom = self.get_parameter('publish_map_odom').value

        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.tf_pub = TransformBroadcaster(self)
        self.static_pub = StaticTransformBroadcaster(self)

        # map -> odom：没有建图算法时按重合处理，保证 TF 树连通
        if self.publish_map_odom:
            self._publish_static(self.map_frame, self.odom_frame)
        # base_footprint -> laser_frame：雷达装在车体中心，按重合处理
        self._publish_static(self.base_frame, self.laser_frame)

        # micro-ROS 发布端是 BEST_EFFORT，订阅端必须一致，否则收不到数据
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, odom_qos)

        self.get_logger().info(
            f'odom_pose_bridge started: {self.odom_topic} -> {self.pose_topic}'
        )

    def _publish_static(self, parent, child):
        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = parent
        ts.child_frame_id = child
        ts.transform.rotation.w = 1.0
        self.static_pub.sendTransform(ts)

    def odom_callback(self, msg: Odometry):
        stamp = msg.header.stamp

        # 1) 动态 TF: odom -> base_footprint
        ts = TransformStamped()
        ts.header.stamp = stamp
        ts.header.frame_id = self.odom_frame
        ts.child_frame_id = self.base_frame
        ts.transform.translation.x = msg.pose.pose.position.x
        ts.transform.translation.y = msg.pose.pose.position.y
        ts.transform.translation.z = msg.pose.pose.position.z
        ts.transform.rotation = msg.pose.pose.orientation
        self.tf_pub.sendTransform(ts)

        # 2) /robot_pose: App 建图模块靠它确定机器人在地图上的位置
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.map_frame
        pose.pose = msg.pose.pose
        self.pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPoseBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        # 收到 SIGTERM/SIGINT 时 rclpy 可能已关闭上下文，重复调用会抛 RCLError
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
