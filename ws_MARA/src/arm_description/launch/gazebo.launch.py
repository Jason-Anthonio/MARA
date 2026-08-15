import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_name = 'arm_description'
    pkg_share = get_package_share_directory(pkg_name)

    # Paths
    xacro_file = os.path.join(pkg_share, 'urdf', 'ASSY.xacro')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')

    # Declare arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='Gazebo world file'
    )

    # Robot Description
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_ros2_control:=true sim_mode:=true']),
        value_type=str
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # Include Gazebo Sim (ros_gz_sim for ROS 2 Jazzy / Harmonic)
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', world]}.items()
    )

    # Spawn Robot Entity in Gazebo
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_arm',
        arguments=[
            '-name', 'arm',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen'
    )

    # ROS <-> Gazebo Bridge for clock and joint states
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Controller Spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen'
    )

    # Delay start of arm_controller until joint_state_broadcaster is ready
    delay_arm_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    return LaunchDescription([
        declare_use_sim_time_arg,
        declare_world_arg,
        robot_state_publisher_node,
        gazebo_sim,
        spawn_entity_node,
        bridge_node,
        joint_state_broadcaster_spawner,
        delay_arm_controller_spawner
    ])
