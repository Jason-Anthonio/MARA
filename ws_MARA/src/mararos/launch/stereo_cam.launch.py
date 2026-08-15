from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Top Camera Instance
        Node(
            package='mararos', 
            executable='camera_pub', 
            name='camera_top_node',
            parameters=[
                # Use your specific hardware ID here
                {'video_device': '/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_B438AE50-video-index0'}, 
                {'camera_name': 'camera_top'}
            ]
        ),
        
        # Bottom Camera Instance
        Node(
            package='mararos',
            executable='camera_pub',
            name='camera_bottom_node',
            parameters=[
                # Replace this with the ID or index for your second camera!
                {'video_device': '/dev/v4l/by-id/usb-046d_0825_D22AEE10-video-index0'}, 
                {'camera_name': 'camera_bottom'}
            ]
        )
    ])