"""Real-robot bringup: UR + Robotiq 85 gripper using ur_robot_driver (ROS2)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_robot = LaunchConfiguration('ur_robot')
    robot_ip = LaunchConfiguration('robot_ip')
    tf_prefix = LaunchConfiguration('tf_prefix')
    use_tool_communication = LaunchConfiguration('use_tool_communication')
    kinematics_config = LaunchConfiguration('kinematics_config')

    pkg_share = FindPackageShare('ur_gripper_description')
    ur_driver_share = FindPackageShare('ur_robot_driver')

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([pkg_share, 'urdf', 'ur_gripper_85.xacro']),
        ' ur_type:=', ur_robot,
        ' tf_prefix:=', tf_prefix,
    ])

    return LaunchDescription([
        DeclareLaunchArgument('ur_robot', default_value='ur3'),
        DeclareLaunchArgument('robot_ip', description='IP address of the UR robot'),
        DeclareLaunchArgument('tf_prefix', default_value=''),
        DeclareLaunchArgument('use_tool_communication', default_value='false'),
        DeclareLaunchArgument(
            'kinematics_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('ur_description'), 'config', 'ur3', 'default_kinematics.yaml'
            ])),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}],
        ),

        # UR robot driver bringup (ROS2 ur_robot_driver)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([ur_driver_share, 'launch', 'ur_control.launch.py'])
            ]),
            launch_arguments={
                'ur_type': ur_robot,
                'robot_ip': robot_ip,
                'tf_prefix': tf_prefix,
                'use_tool_communication': use_tool_communication,
                'kinematics_params_file': kinematics_config,
                'initial_joint_controller': 'scaled_joint_trajectory_controller',
            }.items(),
        ),

        # Robotiq gripper controller via URCap
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('robotiq_control'), 'launch',
                    'urcap_cmodel_action_controller.launch.py'
                ])
            ]),
            launch_arguments={
                'address': robot_ip,
                'config': 'cmodel_action_controller_85',
            }.items(),
        ),
    ])
