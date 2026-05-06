"""Launch UR + Robotiq 85 in Gazebo Classic (cubes task) — ROS2 Humble"""
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    ExecuteProcess,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_gazebo_share = get_package_share_directory('ur_gripper_gazebo')
    pkg_desc_share = get_package_share_directory('ur_gripper_description')

    paused = LaunchConfiguration('paused')
    gui = LaunchConfiguration('gui')
    ur_robot = LaunchConfiguration('ur_robot')
    world_name = LaunchConfiguration('world_name')
    grasp_plugin = LaunchConfiguration('grasp_plugin')

    robot_description_content = Command([
        'xacro ',
        os.path.join(pkg_desc_share, 'urdf', 'ur_gripper_85.xacro'),
        ' ur_type:=', ur_robot,
        ' grasp_plugin:=', grasp_plugin,
    ])

    gazebo_model_path = os.path.join(pkg_gazebo_share, 'models')

    return LaunchDescription([
        DeclareLaunchArgument('paused', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('ur_robot', default_value='ur3'),
        DeclareLaunchArgument('grasp_plugin', default_value='false'),
        DeclareLaunchArgument(
            'world_name',
            default_value=os.path.join(pkg_gazebo_share, 'worlds', 'cubes_task.world')),

        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            gazebo_model_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', '/'),

        # Start Gazebo Classic
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
            ]),
            launch_arguments={
                'world': world_name,
                'paused': paused,
                'gui': gui,
            }.items(),
        ),

        # Robot description + state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}],
        ),

        # Spawn robot in Gazebo (delayed to let Gazebo start)
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    name='spawn_gazebo_model',
                    arguments=[
                        '-entity', 'robot',
                        '-topic', 'robot_description',
                        '-x', '0.11',
                        '-z', '0.69',
                        '-Y', '-1.5707',
                        '-unpause',
                    ],
                    output='screen',
                ),
            ],
        ),

        # ros2_control spawner for joint controllers
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['scaled_joint_trajectory_controller', '--controller-manager', '/controller_manager'],
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
                ),
            ],
        ),
    ])
