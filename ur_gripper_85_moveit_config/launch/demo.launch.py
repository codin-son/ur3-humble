"""MoveIt2 demo (fake joint state publisher) for UR + Robotiq 85 — ROS2 Humble"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ur_robot = LaunchConfiguration('ur_robot')
    pkg_moveit = get_package_share_directory('ur_gripper_85_moveit_config')
    pkg_desc = get_package_share_directory('ur_gripper_description')

    robot_description_content = Command([
        'xacro ',
        os.path.join(pkg_desc, 'urdf', 'ur_gripper_85.xacro'),
        ' ur_type:=', ur_robot,
    ])

    return LaunchDescription([
        DeclareLaunchArgument('ur_robot', default_value='ur3'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content, 'use_sim_time': False}],
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_moveit, 'launch', 'move_group.launch.py')
            ),
            launch_arguments={'ur_robot': ur_robot, 'use_sim_time': 'false'}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_moveit, 'launch', 'moveit_rviz.launch.py')
            ),
            launch_arguments={'use_sim_time': 'false'}.items(),
        ),
    ])
