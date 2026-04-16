#!/bin/bash

echo "Starting Nav2... Waiting for RTAB-Map TF tree..."

# The 'until' loop runs the command. 
# If it crashes (non-zero exit code), the loop triggers, waits 3 seconds, and tries again.
# If it runs successfully or you manually Ctrl+C, the loop ends.

until ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false params_file:=$(pwd)/nav2_params_4.yaml autostart:=true; do
  echo "--------------------------------------------------------"
  echo "⚠️  Nav2 encountered a fatal ERROR and crashed."
  echo "🔄 Retrying in 3 seconds..."
  echo "--------------------------------------------------------"
  sleep 3
done

echo "Nav2 was terminated by the user."
