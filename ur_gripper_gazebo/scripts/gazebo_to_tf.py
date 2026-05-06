#!/usr/bin/env python3
"""Broadcast Gazebo model poses as TF transforms — ROS2 Humble"""
import rclpy
from rclpy.node import Node

from gazebo_msgs.msg import ModelStates
import tf2_ros
from geometry_msgs.msg import TransformStamped

from ur_control import conversions


class GazeboToTf(Node):
    """Republish Gazebo model poses to TF."""

    def __init__(self):
        super().__init__('gazebo_to_tf')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(ModelStates, "/gazebo/model_states", self.callback, 10)
        self._last_time = None

    def callback(self, data):
        now = self.get_clock().now()
        if self._last_time is not None and now == self._last_time:
            return
        for i in range(len(data.name)):
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = "world"
            t.child_frame_id = data.name[i]
            pos = data.pose[i].position
            t.transform.translation.x = pos.x
            t.transform.translation.y = pos.y
            t.transform.translation.z = pos.z
            t.transform.rotation = data.pose[i].orientation
            self.tf_broadcaster.sendTransform(t)
        self._last_time = now


def main(args=None):
    rclpy.init(args=args)
    node = GazeboToTf()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
