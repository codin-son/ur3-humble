"""Display UR + Robotiq 85 in RViz with joint_state_publisher_gui."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_robot = LaunchConfiguration('ur_robot')
    ur_robot_arg = DeclareLaunchArgument('ur_robot', default_value='ur3')

    pkg_share = FindPackageShare('ur_gripper_description')
    robotiq_share = FindPackageShare('robotiq_description')

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([pkg_share, 'urdf', 'ur_gripper_85.xacro']),
        ' ur_type:=', ur_robot,
    ])

    return LaunchDescription([
        ur_robot_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([pkg_share, 'config', 'config.rviz'])],
            required=True,
        ),
    ])
