import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_name = 'arm_description'
    pkg_share = get_package_share_directory(pkg_name)

    xacro_file = os.path.join(pkg_share, 'urdf', 'ASSY.xacro')
    controller_params_file = os.path.join(pkg_share, 'config', 'controller.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # Standalone ROS2 Control node for physical hardware (when not using Gazebo)
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description, 'use_sim_time': use_sim_time},
            controller_params_file
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Start controller manager, then broadcasters and controllers
    delayed_jsb_spawner = TimerAction(
        period=2.0,
        actions=[joint_state_broadcaster_spawner]
    )

    delayed_arm_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    return LaunchDescription([
        declare_use_sim_time_arg,
        ros2_control_node,
        delayed_jsb_spawner,
        delayed_arm_spawner
    ])
