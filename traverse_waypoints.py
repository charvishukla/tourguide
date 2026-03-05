import rclpy
import math 
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped

def create_pose(nav, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0

    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    
    return pose

def main():
    rclpy.init()
    print("Initializing Basic Navigator...")

    nav = BasicNavigator()
    nav.waitUntilNav2Active(localizer='bt_navigator')
    
    # REMOVED: nav.setInitialPose(initial_pose)
    # RTAB-Map is already providing localization. You do not need to set this.

    print("Waiting for Nav2 lifecycle nodes to become active...")
    # FIXED: Tell the navigator NOT to wait for AMCL, since RTAB-Map is used instead
    nav.waitUntilNav2Active(localizer='') 
    print("Nav2 is active and ready!")

    waypoints = [
        create_pose(nav, 0.5, 0.0, 0.0),  # Point 1
        create_pose(nav, 1.0, 0.0, 0.0),  # Point 2
    ]

    print(f"Starting traversal with {len(waypoints)} waypoints...")
    nav.followWaypoints(waypoints)

    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback:
            # Note: Waypoint array indices start at 0
            print(f"Executing waypoint {feedback.current_waypoint}...", end='\r')

    result = nav.getResult()

    # FIXED: Comparing against the imported TaskResult enum
    if result == TaskResult.SUCCEEDED:
        print('\nGoal succeeded!')
    elif result == TaskResult.CANCELED:
        print('\nGoal was canceled!')
    elif result == TaskResult.FAILED:
        print('\nGoal failed!')
        
    rclpy.shutdown()

if __name__ == '__main__':
    main()