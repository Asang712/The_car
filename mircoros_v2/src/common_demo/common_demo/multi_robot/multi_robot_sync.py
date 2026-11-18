#!/usr/bin/env python3
# encoding: utf-8

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import UInt16

SYNC_TOPICS = {
    'cmd_vel': Twist,
    'beep': UInt16,
    'cmd_ptz_angle': Vector3,
}


class MultiRobotSync(Node):
    def __init__(self):
        super().__init__('multi_robot_sync')

        self.declare_parameter('robot_names', ['robot1', 'robot2'])
        robot_names = [name.strip().strip('/') for name in
                       self.get_parameter('robot_names').value if name.strip()]
        if not robot_names:
            self.get_logger().error('robot_names is empty, nothing to sync')


        self.publishers_map = {}
        for topic, msg_type in SYNC_TOPICS.items():
            self.publishers_map[topic] = [
                self.create_publisher(msg_type, f'/{name}/{topic}', 10)
                for name in robot_names
            ]
            self.create_subscription(
                msg_type, topic,
                lambda msg, t=topic: self.sync_callback(t, msg), 10)

        self.get_logger().info(
            f'multi_robot_sync started, robots: {robot_names}, '
            f'topics: {list(SYNC_TOPICS.keys())}')

    def sync_callback(self, topic, msg):
        for pub in self.publishers_map[topic]:
            pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotSync()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
