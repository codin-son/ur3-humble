"""Start MoveIt2 move_group + RViz for UR + Robotiq 85 — ROS2 Humble"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ur_robot = LaunchConfiguration('ur_robot')
    use_sim_time = LaunchConfiguration('use_sim_time')
    pkg_moveit = get_package_share_directory('ur_gripper_85_moveit_config')

    return LaunchDescription([
        DeclareLaunchArgument('ur_robot', default_value='ur3'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_moveit, 'launch', 'move_group.launch.py')
            ),
            launch_arguments={
                'ur_robot': ur_robot,
                'use_sim_time': use_sim_time,
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_moveit, 'launch', 'moveit_rviz.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
