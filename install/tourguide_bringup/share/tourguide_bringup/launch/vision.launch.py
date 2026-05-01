import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('tourguide_bringup'),
        'config',
        'apriltags.yaml'
    )

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        remappings=[
            ('image_rect', '/oak/left/image_raw'), 
            ('camera_info', '/oak/left/camera_info'),
            ('detections', '/detections'),
        ],
        parameters=[
            config_file,                  
            {'image_transport': 'raw'}    
        ]
    )

    return LaunchDescription([
        apriltag_node
    ])