import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_name = 'arm_description'
    pkg_share = get_package_share_directory(pkg_name)

    # Default paths
    default_model_path = os.path.join(pkg_share, 'urdf', 'ASSY.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'launch', 'urdf.rviz')

    # Launch configuration variables
    gui = LaunchConfiguration('gui')
    model = LaunchConfiguration('model')
    rviz_config = LaunchConfiguration('rvizconfig')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare launch arguments
    declare_model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot urdf/xacro file'
    )

    declare_rviz_config_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file'
    )

    declare_gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Flag to enable joint_state_publisher_gui'
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Robot State Publisher
    robot_description = ParameterValue(
        Command(['xacro ', model]),
        value_type=str
    )

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

    # Joint State Publisher (GUI / standard)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(gui)
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=UnlessCondition(gui)
    )

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_model_arg,
        declare_rviz_config_arg,
        declare_gui_arg,
        declare_use_sim_time_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
