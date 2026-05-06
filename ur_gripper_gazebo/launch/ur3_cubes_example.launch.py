"""UR3 + Robotiq 85 cubes example — ROS2 Humble"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ur_gripper_gazebo'), 'launch', 'ur_gripper_85_cubes.launch.py'
                ])
            ]),
            launch_arguments={'ur_robot': 'ur3'}.items(),
        ),
    ])
