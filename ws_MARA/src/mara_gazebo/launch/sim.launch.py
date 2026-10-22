"""
Gazebo Harmonic simulation of MARA with the stereo cameras.

Usage:
    ros2 launch mara_gazebo sim.launch.py
    ros2 launch mara_gazebo sim.launch.py gui:=false rviz:=false   #for headless
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

#pose (rad) for [J1, J2, J4, J6, J9]
SCAN_POSE = '[0.0, 0.0, 0.698, 1.920, 0.0]'


def generate_launch_description():
    description_dir = get_package_share_directory('marav3best_description')
    gazebo_dir = get_package_share_directory('mara_gazebo')

    xacro_file = os.path.join(description_dir, 'urdf', 'marav3best.urdf.xacro')
    world_file = os.path.join(gazebo_dir, 'worlds', 'mara_tabletop.sdf')
    bridge_file = os.path.join(gazebo_dir, 'config', 'gz_bridge.yaml')
    rviz_file = os.path.join(gazebo_dir, 'rviz', 'sim.rviz')

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    world = LaunchConfiguration('world')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_sim:=true']),
        value_type=str
    )

    gz_args = PythonExpression([
        "'-r -v 1 ' + ('' if '", gui, "' == 'true' else '-s ') + '", world, "'"
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'mara', '-z', '0.0'],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_file, 'use_sim_time': True}],
        output='screen',
    )

    def spawner(*names_and_flags):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[*names_and_flags, '--controller-manager', 'controller_manager',
                       '--switch-timeout', '30'],
            parameters=[{'use_sim_time': True}],
            output='screen',
        )

    joint_state_broadcaster = spawner('joint_state_broadcaster')
    trajectory_controller = spawner('trajectory_controller')
    inactive_controllers = spawner('position_controller', 'velocity_controller', '--inactive')

    move_to_scan_pose = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/trajectory_controller/joint_trajectory',
             'trajectory_msgs/msg/JointTrajectory',
             '{joint_names: [J1_yaw_base_shoulderLink, J2_pitch_shoulder, J4_pitch_elbow, '
             'J6_pitch_wrist, J9_horn_drive], points: [{positions: ' + SCAN_POSE + ', '
             'time_from_start: {sec: 3}}]}'],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Show the Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz with camera views'),
        DeclareLaunchArgument('world', default_value=world_file, description='SDF world file'),

        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(description_dir)),

        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,

        RegisterEventHandler(OnProcessExit(target_action=spawn_robot,
                                           on_exit=[joint_state_broadcaster])),
        RegisterEventHandler(OnProcessExit(target_action=joint_state_broadcaster,
                                           on_exit=[trajectory_controller, inactive_controllers])),
        RegisterEventHandler(OnProcessExit(target_action=trajectory_controller,
                                           on_exit=[move_to_scan_pose])),

        rviz_node,
    ])
