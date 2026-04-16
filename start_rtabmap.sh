#!/bin/bash

echo "--------------------------------------------------------"
echo "🗺️  Starting RTAB-Map in Localization Mode..."
echo "Loading map from: ~/.ros/rtabmap.db"
echo "--------------------------------------------------------"

ros2 launch rtabmap_launch rtabmap.launch.py \
  map_topic:="/map" \
  rtabmap_args:="--Mem/IncrementalMemory false --Mem/InitWMWithAllNodes true --Grid/FromDepth true" \
  database_path:="~/.ros/rtabmap.db" \
  frame_id:="base_link" \
  subscribe_scan:=true \
  scan_topic:="/scan_filtered" \
  approx_sync:=true \
  rgb_topic:="/oak/rgb/image_raw" \
  camera_info_topic:="/oak/rgb/camera_info" \
  depth_topic:="/oak/stereo/image_raw" \
  visual_odometry:=false \
  odom_topic:="/odometry/filtered" \
  qos_image:=2 \
  qos_depth:=2 \
  wait_for_transform:=1.4 \
  tf_delay:=0.30
