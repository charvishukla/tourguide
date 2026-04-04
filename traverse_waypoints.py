#!/usr/bin/env python3


import math
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


FILE_PATH = 'waypoints.yaml'

def make_pose(navigator, x, y, yaw_deg=0.0, frame_id='odom'):
    '''
    This function can be used to specify an orientation and position for the robot. 
    Inputs:
        - navigator: Nav2 Navigator object (ex. BasicNavigator)
        - x, y: cartesian coordinates for the robot's destination.
        - yaw_deg: bearing
        - frame_id: coordinate system (map, odom, base_link). The default is the 'map' frame
    Output:
        - pose: a PoseStamped object
    '''
    pose = PoseStamped()                                            # time-stamped pose obect
    pose.header.frame_id = frame_id                                 # set the frame_id
    pose.header.stamp = navigator.get_clock().now().to_msg()        # get and set current timestamp

    # Assigning the 3D coordinates
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0                                      # robot stays on the ground so this is obviously 0

    # Using a quaternion to represent the orientation to prvent gimbal lock 
    # (x, y, z, w)
    yaw = math.radians(yaw_deg)                                     # deg --> rad
    # converting from axis-angle to quaternion (axis is the z-axis and angle is yaw)
    # updating the orientation
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)

    return pose



def load_waypoints_from_file(navigator, file_path, frame_id='map'):
    '''
    Loads a list of waypoints from a specififed yaml file
    Inputs:
        - navigator:  Nav2 Navigator object (ex. BasicNavigator)
        - file_path: waypoints.yaml
        - frame_id: coordinate system (map, odom, base_link). The default is the 'odom' frame
    '''
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)
        
    # for each waypoint defined, create a PoseStamped Object
    # Put them in an array and return
    return [
        make_pose(navigator, p['x'], p['y'], p['yaw'], frame_id) 
        for p in data['waypoints']
    ]


def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active(localizer='robot_localization')                               # Wait until Nav2 is active
    print("Loading Waypoints")
    waypoints = load_waypoints_from_file(navigator, FILE_PATH, frame_id='map')                 # Load waypoints
    print("Loaded Waypoints:   ", waypoints)

    print("----- Starting Waypoint traversal -----")
    for i, goal_pose in enumerate(waypoints):
        print("Current index: ", i, "               Current Goal: ", goal_pose )
        navigator.goToPose(goal_pose)
        # navigator.followWaypoints(waypoints)                                                        # use Nav2's followWaypoints module to navigate through the specified waypts
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()                                                  
            if feedback:
                print(f"Navigating to Goal {i}... Distance remaining: {feedback.distance_remaining:.2f} m", end='\r')
        
        print("")
        result = navigator.getResult()                                                              # grab result status 


        if result == TaskResult.SUCCEEDED:
            print("Waypoints completed successfully.")
        elif result == TaskResult.CANCELED:
            print("Waypoint task was canceled.")
        elif result == TaskResult.FAILED:
            print("Waypoint task failed. Waypoint index: ", i, "Current waypoinr goal: ", goal_pose)
            break
        else:
            print("Unknown result.")

    rclpy.shutdown()                                                                            # shut the nodes


if __name__ == '__main__':
    main()
