import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('tourguide_bringup'),
        'config',
        'apriltags.yaml'
    )

    # Camera name argument — set to match whatever camera node is running.
    # Defaults to 'tourguide_oak' to match localization.launch.py and
    # mapping.launch.py. Override with:
    #   ros2 launch tourguide_bringup vision.launch.py camera_name:=oak
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='tourguide_oak',
        description=(
            'Prefix for camera topics. Must match the camera node name '
            '(tourguide_oak when launched via localization.launch.py; '
            'oak when running the camera driver standalone).'
        )
    )
    camera_name = LaunchConfiguration('camera_name')

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        remappings=[
            # apriltag_ros expects 'image_rect' and 'camera_info' as inputs.
            # We remap them to the OAK-D RGB stream.
            # The leading '/' is omitted so LaunchConfiguration substitution works.
            ('image_rect', [camera_name, '/rgb/image_raw']),
            ('camera_info', [camera_name, '/rgb/camera_info']),
            ('detections', '/detections'),
        ],
        parameters=[
            config_file,
            {'image_transport': 'raw'}
        ]
    )

    return LaunchDescription([
        camera_name_arg,
        apriltag_node,
    ])