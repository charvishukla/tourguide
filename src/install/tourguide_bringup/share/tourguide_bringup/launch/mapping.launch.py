import os 
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # 1. OAK-D V3 
    depthai_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('depthai_ros_driver_v3'), 'launch', 'driver.launch.py'
        )]),
        launch_arguments={
            'name': 'tourguide_oak', # FIX 1: Renamed to avoid URDF clash!
            'parent_frame': 'camera_mount_link', 
            'cam_pos_x': '0.0', 
            'cam_pos_y': '0.0',
            'cam_pos_z': '0.0',
            'nn.i_nn_config_path': 'depthai_ros_driver/yolo',
            'pipeline_gen.i_pipeline_type': 'RGBD',
            'i_use_system_time': 'true',  
            'rgb.i_fps': '10.0', 
            'rgb.i_width': '640',
            'rgb.i_height': '400'
        }.items()
    )
    
    # 2. AprilTags
    # apriltag_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('tourguide_bringup'), 'launch', 'vision.launch.py'
    #     )])
    # )

    # 3. Semantic Bridge
    bridge_node = Node(
        package='tourguide_bringup',
        executable='semantic_bridge',
        name='semantic_bridge',
        output='screen'
    )

    # 4. RTAB-Map
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py'
        )]),
        launch_arguments={
            'rtabmap_args': '--delete_db_on_start --RGBD/LinearUpdate 0.25 --RGBD/AngularUpdate 0.25 --Rtabmap/DetectionRate 0.5 --Kp/MaxFeatures 800 --Mem/ImagePreDecimation 2 --Marker/VarianceLinear 0.0001 --Marker/VarianceAngular 0.01 --landmark_linear_variance 0.05 --landmark_angular_variance 9999.0',
            'rtabmap_viz': 'false',
            'visual_odometry': 'false',
            'frame_id': 'base_link',
            'odom_topic': '/odometry/filtered',
            # FIX 2: Updated the topics to match the new camera name
            'rgb_topic': '/tourguide_oak/rgb/image_raw',
            'camera_info_topic': '/tourguide_oak/rgb/camera_info',
            'depth_topic': '/tourguide_oak/stereo/image_raw',
            'subscribe_scan': 'true',
            'scan_topic': '/scan_filtered',
            'approx_sync': 'true',
            'qos': '2',
            'apriltag': '/detections', 
            'queue_size': '50', 
            'wait_for_transform': '1.0' 
        }.items()
    )

    return LaunchDescription([
        depthai_launch,
        # apriltag_launch,
        bridge_node,
        rtabmap_launch
    ])