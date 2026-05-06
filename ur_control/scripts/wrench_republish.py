#!/usr/bin/env python3
# Copyright (c) 2018-2021 Cristian Beltran — ROS2 Humble port

import rclpy
from rclpy.node import Node
import numpy as np
from ur_control import conversions
from geometry_msgs.msg import WrenchStamped


class FTRepublisher(Node):
    def __init__(self):
        super().__init__('ft_republisher')
        self.in_topic = '/wrench'
        self.out_topic = '/test/wrench'
        self.get_logger().info("Publishing FT to %s" % self.out_topic)
        self.pub = self.create_publisher(WrenchStamped, self.out_topic, 10)
        self.create_subscription(WrenchStamped, self.in_topic, self.cb_raw, 1)
        self.get_logger().info('FT republisher initialized')
        # NOTE: arm init removed — add back when needed with proper node arg

    def cb_raw(self, msg):
        # Passthrough republish; transform to base frame if arm is available
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FTRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
