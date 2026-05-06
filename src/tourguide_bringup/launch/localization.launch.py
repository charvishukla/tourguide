import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. OAK-D with YOLO
    depthai_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('depthai_ros_driver'), 'launch', 'yolov4_spatial.launch.py'
        )])
    )

    # 2. AprilTags
    apriltag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('tourguide_bringup'), 'launch', 'vision.launch.py'
        )])
    )

    # 3. Semantic Bridge
    bridge_node = Node(
        package='tourguide_bringup',
        executable='semantic_bridge',
        name='semantic_bridge',
        output='screen'
    )

    # 4. Running rtabmap in localization mode:
    # --Mem/IncrementalMemory false 
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py'
        )]),
        launch_arguments={
            'map_topic': '/map',
            'rtabmap_args': '--Mem/IncrementalMemory false --Mem/InitWMWithAllNodes true --Grid/FromDepth true --Marker/VarianceLinear 0.0001 --Marker/VarianceAngular 9999.0 --landmark_linear_variance 0.05 --landmark_angular_variance 9999.0',
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
            'tf_delay': '0.30',
            'tag_topic': '/detections'
        }.items()
    )

    return LaunchDescription([
        depthai_launch,
        apriltag_launch,
        bridge_node,
        rtabmap_launch
    ])