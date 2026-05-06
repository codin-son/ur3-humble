"""MoveIt2 RViz launch for UR + Robotiq 85 — ROS2 Humble"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    pkg_moveit = get_package_share_directory('ur_gripper_85_moveit_config')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_moveit, 'launch', 'moveit.rviz')],
            parameters=[
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])
