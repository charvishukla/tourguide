import os 
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Path to your existing database
    # Adjust this path to where your map file is saved
    database_path = os.path.expanduser('~/.ros/rtabmap.db')

    # 1. OAK-D V3 (Same as mapping)
    depthai_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('depthai_ros_driver_v3'), 'launch', 'driver.launch.py'
        )]),
        launch_arguments={
            'name': 'tourguide_oak',
            'parent_frame': 'camera_mount_link', 
            'pipeline_gen.i_pipeline_type': 'RGBD',
            'rgb.i_fps': '10.0', 
            'rgb.i_width': '640',
            'rgb.i_height': '400'
        }.items()
    )

    # 2. Semantic Bridge (Keep running for localization)
    bridge_node = Node(
        package='tourguide_bringup',
        executable='semantic_bridge',
        name='semantic_bridge',
        output='screen'
    )

    # 3. RTAB-Map in Localization Mode
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py'
        )]),
        launch_arguments={
            # localization=true is the critical change
            'localization': 'true',
            'database_path': database_path,
            'rtabmap_args': '--RGBD/LinearUpdate 0.25 --RGBD/AngularUpdate 0.25 --Mem/ImagePreDecimation 2',
            'rtabmap_viz': 'false',
            'visual_odometry': 'false',
            'frame_id': 'base_link',
            'odom_topic': '/odometry/filtered',
            'rgb_topic': '/tourguide_oak/rgb/image_raw',
            'camera_info_topic': '/tourguide_oak/rgb/camera_info',
            'depth_topic': '/tourguide_oak/stereo/image_raw',
            'subscribe_scan': 'true',
            'scan_topic': '/scan_filtered',
            'approx_sync': 'true',
            'queue_size': '50'
        }.items()
    )

    return LaunchDescription([
        depthai_launch,
        bridge_node,
        rtabmap_launch
    ])
