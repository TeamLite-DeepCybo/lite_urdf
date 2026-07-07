"""Live RViz visualization for the production Lite arms+grippers model.

This is intended to run beside bar_bringup_lite/real.launch.py. The real
bringup owns CAN, controllers, and /lite/joint_states; this launch only
    publishes a robot_description + TF tree and starts RViz.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    description_pkg = FindPackageShare('lite_urdf')
    model = LaunchConfiguration('model')
    joint_state_topic = LaunchConfiguration('joint_state_topic')
    robot_description_topic = LaunchConfiguration('robot_description_topic')

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                FindExecutable(name='xacro'), ' ', model,
                ' emit_ros2_control:=false',
            ]),
            value_type=str,
        ),
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=PathJoinSubstitution([
                description_pkg,
                'urdf',
                'lite.urdf.xacro',
            ]),
            description='Lite arms+grippers xacro model to visualize.',
        ),
        DeclareLaunchArgument(
            'joint_state_topic',
            default_value='/lite/joint_states',
            description='Joint-state topic from the real robot bringup.',
        ),
        DeclareLaunchArgument(
            'robot_description_topic',
            default_value='/lite_urdf/robot_description',
            description='Robot description topic used by this RViz instance.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                description_pkg,
                'config',
                'live_lite_urdf.rviz',
            ]),
            description='RViz config path.',
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='lite_urdf_state_publisher',
            output='screen',
            parameters=[robot_description],
            remappings=[
                ('joint_states', joint_state_topic),
                ('robot_description', robot_description_topic),
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='lite_urdf_rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
        ),
    ])
