#!/usr/bin/env python3

import math
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def make_pose(navigator, x, y, yaw_deg=0.0, frame_id='map'):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = navigator.get_clock().now().to_msg()

    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0

    yaw = math.radians(yaw_deg)
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)

    return pose


def square(navigator, side=1.0, frame_id='odom'):
    negative_side = -1 * side
    return [
        make_pose(navigator, 0.0, 0.0, 180, frame_id),
        make_pose(navigator, negative_side, 0.0, 180, frame_id),
        make_pose(navigator, negative_side, side, 90, frame_id),
        make_pose(navigator, 0.0, side, 0, frame_id),
        make_pose(navigator, 0.0, 0.0, -90, frame_id),
    ]


def triangle(navigator, side=1.0, frame_id='odom'):
  #  h = math.sqrt(3) / 2.0 * side
    return [
        make_pose(navigator, 0.0, 0.0, 0, frame_id),
        make_pose(navigator, -2.0, 0.0, 180, frame_id),
        make_pose(navigator, -1.0, 1, -135, frame_id),
        make_pose(navigator, 0.0, 0.0, 0.0, frame_id),
    ]


def zigzag(navigator, step=0.75, frame_id='odom'):
    return [
        make_pose(navigator, 0.0, 0.0, 0, frame_id),
        make_pose(navigator, step, step, 0, frame_id),
        make_pose(navigator, 2 * step, 0.0, 0, frame_id),
        make_pose(navigator, 3 * step, step, 0, frame_id),
        make_pose(navigator, 4 * step, 0.0, 0, frame_id),
    ]


def line(navigator, frame_id='odom'):
    waypoints = []

    waypoints.append(make_pose(navigator, 0.0, 0.0, 0, frame_id))
    waypoints.append(make_pose(navigator, 1.0, 0.0, 0, frame_id))
    waypoints.append(make_pose(navigator, 2.0, 0.0, 0, frame_id))
    return waypoints 


def circle(navigator, radius=1.0, num_points=12, frame_id='odom'):
    poses = []
    for i in range(num_points):
        theta = 2.0 * math.pi * i / num_points
        x = radius * math.cos(theta)
        y = radius * math.sin(theta)
        yaw_deg = math.degrees(theta + math.pi / 2.0)  # tangent-ish heading
        poses.append(make_pose(navigator, x, y, yaw_deg, frame_id))
    poses.append(make_pose(navigator, radius, 0.0, 90, frame_id))  # close loop
    return poses


def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Wait until Nav2 is active
    navigator.waitUntilNav2Active(localizer='robot_localization')
    
    # Pick one:
#    waypoints = square(navigator, side=1.0, frame_id='odom')
    waypoints = triangle(navigator, side=1.0, frame_id='odom')
    # waypoints = zigzag(navigator, step=0.75, frame_id='map')
    # waypoints = circle(navigator, radius=1.0, num_points=16, frame_id='map')
   # waypoints = line(navigator, frame_id='odom')
    navigator.followWaypoints(waypoints)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print("Following waypoints...")

    result = navigator.getResult()

    if result == TaskResult.SUCCEEDED:
        print("Waypoints completed successfully.")
    elif result == TaskResult.CANCELED:
        print("Waypoint task was canceled.")
    elif result == TaskResult.FAILED:
        print("Waypoint task failed.")
    else:
        print("Unknown result.")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
