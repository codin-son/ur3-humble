"""MoveIt2 move_group launch for UR + Robotiq 85 — ROS2 Humble"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ur_robot = LaunchConfiguration('ur_robot')
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_moveit = get_package_share_directory('ur_gripper_85_moveit_config')
    pkg_desc = get_package_share_directory('ur_gripper_description')

    robot_description_content = Command([
        'xacro ',
        os.path.join(pkg_desc, 'urdf', 'ur_gripper_85.xacro'),
        ' ur_type:=', ur_robot,
    ])

    robot_description = {'robot_description': robot_description_content}

    robot_description_semantic_content = Command([
        'cat ',
        os.path.join(pkg_moveit, 'config', 'ur_robot_gazebo.srdf'),
    ])
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_content}

    kinematics_yaml = os.path.join(pkg_moveit, 'config', 'kinematics.yaml')
    ompl_planning_yaml = os.path.join(pkg_moveit, 'config', 'ompl_planning.yaml')
    moveit_controllers_yaml = os.path.join(pkg_moveit, 'config', 'moveit_controllers.yaml')
    joint_limits_yaml = os.path.join(pkg_moveit, 'config', 'joint_limits.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('ur_robot', default_value='ur3'),
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
                kinematics_yaml,
                ompl_planning_yaml,
                moveit_controllers_yaml,
                joint_limits_yaml,
                {
                    'planning_scene_monitor_options': {
                        'name': 'planning_scene_monitor',
                        'robot_description': 'robot_description',
                        'joint_state_topic': '/joint_states',
                        'attached_collision_object_topic': '/move_group/planning_scene_monitor',
                        'publish_planning_scene_topic': '/move_group/publish_planning_scene',
                        'monitored_planning_scene_topic': '/move_group/monitored_planning_scene',
                        'wait_for_initial_state_timeout': 10.0,
                    },
                },
            ],
        ),
    ])
