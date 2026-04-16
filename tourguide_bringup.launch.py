import os 
from launch import LaunchDescription 
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Specifying RTABMAP Localization Mode Launch
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('rtabmap_launch'), '/launch/rtabmap.launch.py'
        ]),
        launch_arguments={
            'map_topic': '/map',
            'rtabmap_args': '--Mem/IncrementalMemory false --Mem/InitWMWithAllNodes true --Grid/FromDepth true',
            'database_path': '~/.ros/rtabmap.db',
            'frame_id': 'base_link',
            'subscribe_scan': 'true',
            'scan_topic': '/scan_filtered',
            'approx_sync': 'true',
            'rgb_topic': '/oak/rgb/image_raw',
            'camera_info_topic': '/oak/rgb/camera_info',
            'depth_topic': '/oak/stereo/image_raw',
            'visual_odometry': 'false',
            'odom_topic': '/odometry/filtered',
            'qos_image': '2',
            'qos_depth': '2',
            'wait_for_transform': '1.4',
            'tf_delay': '0.30'
        }.items()
    )

    # Specifying NAV2 Launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('nav2_bringup'), '/launch/navigation_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': os.path.join(os.getcwd(), 
                                        'tourguide', 
                                        'nav2_params_4.yaml'
                                        ), 
            'autostart': 'true'
        }.items()
    )
    

    return LaunchDescription([rtabmap_launch, 
                                nav2_launch ])








