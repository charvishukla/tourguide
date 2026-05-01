#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time
import yaml 
import os
from ament_index_python.packages import get_package_share_directory


def create_pose(navigator, waypoints):
    pose = PoseStamped()                    # initialize a pose stamped message 
    pose.header.frame_id = 'map'            # the poses are in the map frame
    pose.header.stamp = navigator.get_clock().now().to_msg()        # we will need poses to be timestamped 

    pose.pose.position.x = float(wp['x'])
    pose.pose.position.y = float(wp['y'])
    pose.pose.position.z = float(wp['z'])
    pose.pose.orientation.x = float(wp['qx'])
    pose.pose.orientation.y = float(wp['qy'])
    pose.pose.orientation.z = float(wp['qz'])
    pose.pose.orientation.w = float(wp['qw'])


def get_pose_sequence_from_route(waypoints_db, tour_sequence):
    tour_stops = []
    for point_id in tour_sequence:
        if point_id in waypoints_db:
            wp = waypoints_db[point_id]
            pose_msg = create_pose(navigator, 
                                    wp['x'], 
                                    wp['y'], 
                                    wp['z'], 
                                    wp['qx'], 
                                    wp['qy'], 
                                    wp['qz'],
                                    wp['qw'], 
                                    'map')
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


    # load apriltags positions
    print("Loading Apriltag Pose DB...")
    with open(os.path.join(config_dir, 'tour_waypoints.yaml'), 'r') as file:
        waypoints = yaml.safe_load(file)['tour_waypoints']
    
    # load route ordering 
    print("Loading Route...")
    with open(os.path.join(config_dir, 'tour_route.yaml'), 'r') as file:
        route = yaml.safe_load(file)['route']

    
    # get route array containing pose stamped objects 
    tour_stop_poses = get_pose_sequence_from_route(waypoints, route)

    for i, (stop_name, goal_pose) in enumerate(tour_stops):
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


