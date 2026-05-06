"""Launch gazebo_to_tf node — ROS2 Humble"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur_gripper_gazebo',
            executable='gazebo_to_tf',
            name='gazebo_to_tf',
            output='screen',
        ),
    ])
