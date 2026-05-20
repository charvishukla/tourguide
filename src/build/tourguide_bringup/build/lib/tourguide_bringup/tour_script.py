#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time
import yaml 
import os
from ament_index_python.packages import get_package_share_directory
import math


def get_offset_pose(x, y, z, qx, qy, qz, qw, offset_distance=1.0):
    # z-axis direction vector 
    z_axis_x = 2.0 * (qx * qz + qy * qw)
    z_axis_y = 2.0 * (qy * qz - qx * qw)

    # stepping back in the z-direction by offset distance amount 
    safe_x = x - (offset_distance * z_axis_x)
    safe_y = y - (offset_distance * z_axis_y)

    # compute new yaw
    yaw = math.atan2(y - safe_y, x - safe_x)

    # yaw to quaternion
    safe_qx = 0.0
    safe_qy = 0.0
    safe_qz = math.sin(yaw / 2.0)
    safe_qw = math.cos(yaw / 2.0)

    # return new pose
    return safe_x, safe_y, safe_qx, safe_qy, safe_qz, safe_qw


def create_pose(navigator, waypoints):

    # get offsetted verison of the waypointts
    safe_x, safe_y, safe_qx, safe_qy, safe_qz, safe_qw = get_offset_pose(float(waypoints['x']), 
                                                                                float(waypoints['y']), 
                                                                                float(waypoints['z']), 
                                                                                float(waypoints['qx']),
                                                                                float(waypoints['qy']), 
                                                                                float(waypoints['qz']), 
                                                                                float(waypoints['qw']),
                                                                                offset_distance=-1.0 
    )

    pose = PoseStamped()                    # initialize a pose stamped message 
    pose.header.frame_id = 'map'            # the poses are in the map frame
    pose.header.stamp = navigator.get_clock().now().to_msg()        # we will need poses to be timestamped 

    # xyz position coords in the map 
    pose.pose.position.x = safe_x   
    pose.pose.position.y = safe_y
    pose.pose.position.z = 0.0                      # this should be 0 since 2D

    # orienatation
    pose.pose.orientation.x = safe_qx
    pose.pose.orientation.y = safe_qy
    pose.pose.orientation.z = safe_qz
    pose.pose.orientation.w = safe_qw

    return pose


def get_pose_sequence_from_route(navigator, waypoints_db, tour_sequence):
    tour_stops = []
    for point_id in tour_sequence:
        if point_id in waypoints_db:
            wp = waypoints_db[point_id]
            pose_msg = create_pose(navigator, wp)
            tour_stops.append((wp['name'], pose_msg))
            print(f"SUCCESS: Loaded Waypoints from file!")
        else:
            print(f" WARNING: '{point_id}' was in tour_route.yaml but not found in waypoints.yaml! Skipping...")
        
    return tour_stops 

def main():
    # initializing navigator
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active(localizer='robot_localization') 

    pkg_share_dir = get_package_share_directory('tourguide_bringup')
    config_dir = os.path.join(pkg_share_dir, 'config')

    # load apriltags positions
    print("Loading Apriltag Pose DB...")
    with open(os.path.join(config_dir, 'tour_waypoints.yaml'), 'r') as file:
        waypoints = yaml.safe_load(file)['tour_waypoints']
    
    # load route ordering 
    print("Loading Route...")
    with open(os.path.join(config_dir, 'tour_route.yaml'), 'r') as file:
        route = yaml.safe_load(file)['route']

    
    # get route array containing pose stamped objects 
    tour_stop_poses = get_pose_sequence_from_route(navigator, waypoints, route)

    for i, (stop_name, goal_pose) in enumerate(tour_stop_poses):
        print(f"\nCurrent index: {i}  Current Goal: {stop_name}")

        navigator.goToPose(goal_pose)                   # tell the navigator to go to a specific pose
        while not navigator.isTaskComplete():           # keep navigating until the goal is complete
            feedback = navigator.getFeedback()          # get current feedback                                          
            if feedback:
                print(f"Navigating to {stop_name}... Distance remaining: {feedback.distance_remaining:.2f} m", end='\r')
        

        print(" ")
        result = navigator.getResult()                  # Once the robot reaches the goal, we get the result/statuses                         

        if result == TaskResult.SUCCEEDED:
            print(f"[SUCCESS] Arrived at {stop_name} successfully!")
            time.sleep(5.0)                             # Robot pauses to give its "tour"
        elif result == TaskResult.CANCELED:
            print(f"[GOAL CANCELLED] Waypoint task to {stop_name} was canceled.")
        elif result == TaskResult.FAILED:
            print(f"[ERROR] Waypoint task failed for {stop_name}!")
            print
        else:
            print("Unknown result.")

    print('Tour Complete')
    rclpy.shutdown()



# CALL MAIN
if __name__ == '__main__':
    main()


