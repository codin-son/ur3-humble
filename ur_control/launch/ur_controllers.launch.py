"""Spawn ROS2 controllers for UR + optional Robotiq gripper — ROS2 Humble"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gripper_robotiq_85 = LaunchConfiguration('gripper_robotiq_85')
    gripper_robotiq_hande = LaunchConfiguration('gripper_robotiq_hande')

    return LaunchDescription([
        DeclareLaunchArgument('gripper_robotiq_85', default_value='false'),
        DeclareLaunchArgument('gripper_robotiq_hande', default_value='false'),

        # TF publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'publish_frequency': 50.0}],
        ),

        # Controller spawners
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['scaled_joint_trajectory_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
        # Gripper controller (spawned always; depends on YAML defining it)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
    ])
