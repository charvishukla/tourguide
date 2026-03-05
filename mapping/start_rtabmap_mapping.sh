#!/bin/bash
trap "echo 'Stopping mapping...'; kill 0" EXIT

source /opt/ros/jazzy/setup.bash

echo "=========================================="
echo "   STARTING RTAB-MAP (Background)         "
echo "=========================================="

# Launch RTAB-Map
ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:="--delete_db_on_start" \
    rtabmap_viz:=false \
    visual_odometry:=false \
    frame_id:=base_link \
    odom_topic:=/odometry/filtered \
    subscribe_rgb:=true \
    rgb_topic:=/oak/rgb/image_raw \
    camera_info_topic:=/oak/rgb/camera_info \
    subscribe_depth:=true \
    depth_topic:=/oak/stereo/image_raw \
    subscribe_scan:=true \
    scan_topic:=/scan_filtered \
    approx_sync:=true \
    qos:=2 \
    queue_size:=20 > /dev/null 2>&1 &

MAPPING_PID=$!
echo "Mapping running (PID $MAPPING_PID)."
echo "You can now drive using WebUI or a separate terminal."
echo "Press Ctrl+C here to save the map and stop."

# Wait forever until user presses Ctrl+C
wait $MAPPING_PID
