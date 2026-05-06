#!/usr/bin/env python3
"""Publish Gazebo world/robot TF transforms — ROS2 Humble"""
import rclpy
from rclpy.node import Node
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped
from gazebo_msgs.msg import ModelStates
from ur_control import conversions, transformations
from pyquaternion import Quaternion


class WorldPublisher(Node):
    def __init__(self):
        super().__init__('gazebo_models_tf')
        self.br = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(ModelStates, "/gazebo/model_states", self.gazebo_callback, 10)
        self.robot = []

    def gazebo_callback(self, msg):
        self.robot = []
        world = np.zeros(7)
        world[6] = 1
        for i, obj_name in enumerate(msg.name):
            if obj_name.endswith("robot"):
                pose = msg.pose[i]
                robot_pose = np.concatenate([
                    conversions.from_point(pose.position),
                    conversions.from_quaternion(pose.orientation)
                ])
                self.robot = robot_pose
                world[:3] -= robot_pose[:3]
                world[:3] = Quaternion(np.roll(robot_pose[3:], 1)).inverse.rotate(world[:3])
                world[3:] = transformations.vector_from_pyquaternion(
                    transformations.vector_to_pyquaternion([0.5, -0.5, -0.5, 0.5]).inverse)

                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = "base_link"
                t.child_frame_id = "sim_world"
                t.transform.translation.x = world[0]
                t.transform.translation.y = world[1]
                t.transform.translation.z = world[2]
                t.transform.rotation.x = world[3]
                t.transform.rotation.y = world[4]
                t.transform.rotation.z = world[5]
                t.transform.rotation.w = world[6]
                self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = WorldPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
