#!/usr/bin/env python3
# Copyright (c) 2018-2021 Cristian Beltran — ROS2 Humble port

import argparse
import rclpy
from rclpy.node import Node
import numpy as np
from ur_control import conversions
from sensor_msgs.msg import Imu


class ImuFakeNode(Node):
    def __init__(self, namespace="", frequency=500):
        super().__init__('imu_fake')
        prefix = "" if not namespace else namespace + "_"
        topic = (namespace.rstrip('/') + '/imu') if namespace else 'imu'
        self.pub = self.create_publisher(Imu, topic, 10)
        gravity = np.array([0, 0, 9.81])
        self.timer = self.create_timer(1.0 / frequency, lambda: self._publish(prefix, gravity))

    def _publish(self, prefix, gravity):
        msg = Imu()
        msg.header.frame_id = prefix + 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.linear_acceleration = conversions.to_vector3(gravity)
        self.pub.publish(msg)


def main(args=None):
    parser = argparse.ArgumentParser(description='Fake IMU publisher')
    parser.add_argument('-ns', '--namespace', type=str, default="")
    parsed, remaining = parser.parse_known_args()
    rclpy.init(args=remaining)
    node = ImuFakeNode(namespace=parsed.namespace)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
