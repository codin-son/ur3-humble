"""MoveIt2 move_group for UR + Robotiq Hand-e — ROS2 Humble"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ur_robot = LaunchConfiguration('ur_robot')
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_moveit = get_package_share_directory('ur_hande_moveit_config')
    pkg_desc = get_package_share_directory('ur_gripper_description')

    robot_description_content = Command([
        'xacro ',
        os.path.join(pkg_desc, 'urdf', 'ur_gripper_hande.xacro'),
        ' ur_type:=', ur_robot,
    ])

    robot_description = {'robot_description': robot_description_content}
    robot_description_semantic_content = Command([
        'cat ', os.path.join(pkg_moveit, 'config', 'ur_robot_gazebo.srdf'),
    ])
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_content}

    return LaunchDescription([
        DeclareLaunchArgument('ur_robot', default_value='ur3e'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            output='screen',
            parameters=[
                robot_description,
                robot_description_semantic,
                {'use_sim_time': use_sim_time},
                os.path.join(pkg_moveit, 'config', 'kinematics.yaml'),
                os.path.join(pkg_moveit, 'config', 'ompl_planning.yaml'),
                os.path.join(pkg_moveit, 'config', 'moveit_controllers.yaml'),
                os.path.join(pkg_moveit, 'config', 'joint_limits.yaml'),
            ],
        ),
    ])
