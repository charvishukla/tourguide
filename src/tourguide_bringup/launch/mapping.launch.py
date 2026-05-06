import os 

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch OAK-D with YOLO Spatial Detections
    depthai_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('depthai_ros_driver'), 'launch', 'yolov4_spatial.launch.py'
        )])
    )

    # apriltags
    # Consumes shared RGB
    apriltag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('tourguide_bringup'), 'launch', 'vision.launch.py'
        )])
    )

    # bridge
    # Consumes YOLO data, publishes to RTAB-Map
    bridge_node = Node(
        package='tourguide_bringup',
        executable='semantic_bridge',
        name='semantic_bridge',
        output='screen'
    )

    # if you wanna restart the mapping from scratch (instead of just updating the map across sessions), 
    # add --delete_db_on_start to 'rtabmap_args'
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py'
        )]),
        launch_arguments={
            'rtabmap_args': ' --RGBD/LinearUpdate 0.25 --RGBD/AngularUpdate 0.25 --Rtabmap/DetectionRate 0.5 --Kp/MaxFeatures 400 --Mem/ImagePreDecimation 2 --Marker/VarianceLinear 0.0001 --Marker/VarianceAngular 9999.0 --landmark_linear_variance 0.05 --landmark_angular_variance 9999.0',
            'rtabmap_viz': 'false',
            'visual_odometry': 'false',
            'frame_id': 'base_link',
            'odom_topic': '/odometry/filtered',
            'subscribe_rgb': 'true',
            'rgb_topic': '/oak/rgb/image_raw',
            'camera_info_topic': '/oak/rgb/camera_info',
            'subscribe_depth': 'true',
            'depth_topic': '/oak/stereo/image_raw',
            'subscribe_scan': 'true',
            'scan_topic': '/scan_filtered',
            'approx_sync': 'true',
            'qos': '2',
            'tag_topic': '/detections',
            'queue_size': '20'
        }.items()
    )

    return LaunchDescription([
        depthai_launch,
        apriltag_launch,
        bridge_node,
        rtabmap_node
    ])


