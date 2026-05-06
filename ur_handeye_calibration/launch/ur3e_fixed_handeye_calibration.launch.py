"""UR3e fixed hand-eye calibration launch — ROS2 Humble"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ur_calibration_ns', default_value='/'),
        DeclareLaunchArgument('tracking_base_frame', default_value='camera_link'),
        DeclareLaunchArgument('tracking_marker_frame', default_value='marker_link'),
        DeclareLaunchArgument('robot_base_frame', default_value='base_link'),
        DeclareLaunchArgument('robot_effector_frame', default_value='ee_link'),
        DeclareLaunchArgument('camera_setup', default_value='Fixed'),
        DeclareLaunchArgument('solver', default_value='Daniilidis1999'),

        Node(
            package='ur_handeye_calibration',
            executable='calibrator.py',
            name='ur_handeye_calibration_capture',
            output='screen',
            parameters=[{
                'ur_calibration_ns': LaunchConfiguration('ur_calibration_ns'),
                'tracking_base_frame': LaunchConfiguration('tracking_base_frame'),
                'tracking_marker_frame': LaunchConfiguration('tracking_marker_frame'),
                'robot_base_frame': LaunchConfiguration('robot_base_frame'),
                'robot_effector_frame': LaunchConfiguration('robot_effector_frame'),
                'camera_setup': LaunchConfiguration('camera_setup'),
                'solver': LaunchConfiguration('solver'),
            }],
        ),
    ])
