"""Standalone BAR Lite URDF inspector."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    description_pkg = FindPackageShare('lite_urdf')
    xacro_file = PathJoinSubstitution([description_pkg, 'urdf', 'lite.urdf.xacro'])

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                FindExecutable(name='xacro'), ' ', xacro_file,
                ' emit_ros2_control:=false',
                ' use_fake_hardware:=true',
            ]),
            value_type=str,
        ),
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                description_pkg,
                'config',
                'view_lite.rviz',
            ]),
            description='Path to an RViz config file.',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
        ),
    ])
