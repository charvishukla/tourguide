import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


from quaternion_helpers.utils import euler_yaw_to_quaternion
import math 
import time 


class WayPointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')
        
        # configs 
        self.distance_threshold = 0.10  # for now, consider 10 cms from the goal to be arrived
        self.waypoints = [] 
        self.current_waypoint_index = 0 

        self.state = 'IDLE'         # set the initial state to IDLE 
        self.wait_start_time = 0.0
        self.current_robot_x = 0.0
        self.current_robot_y = 0.0

        # Publisher to send goals to RTAB-MAP
        self.goal_pub = self.create_publisher(PoseStamped, 
                                              '/rtabmap/goal', 
                                              10)

        # subscribe to know where the robot is on the map 
        self.pose_sub = self.create_subscription(
            PoseStamped, 
            '/rtabmap/global_pose',  
            self.pose_callback, 
            10
        )

        # Running the main control loop at 5 Hz. 
        self.timer = self.create_timer(0.2, self.control_loop)
        self.start_next_waypoint()

    
    def pose_callback(self, msg): 
        self.correct_robot_x = msg.pose.position.x
        self.correct_robot_y = msg.pose.position.y

    def start_next_waypoint(self):
        if self.current_wp_index >= len(self.waypoints):
            self.get_logger().info("Tour complete! Returning to IDLE.")
            self.state = 'IDLE'
            return

        current_waypoint = self.waypoints[self.current_wp_index]
        self.get_logger().info(f"Navigating to: {current_waypoint['name']}")
        
    
        goal_msg = PoseStamped()                                                                        # create a pose stamped message 
        goal_msg.header.frame_id = 'map'    
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = current_waypoint['x']                                               # set the x and y coordinates in the goal message 
        goal_msg.pose.position.y = current_waypoint['y']
        
        
        # setting orientation
        qx, qy, qz, qw = euler_yaw_to_quaternion(current_waypoint['theta'])
        goal_msg.pose.orientation.x = qx
        goal_msg.pose.orientation.y = qy
        goal_msg.pose.orientation.z = qz
        goal_msg.pose.orientation.w = qw
        
        self.goal_pub.publish(goal_msg)
        self.state = 'NAVIGATING'

    def control_loop(self):
        if self.state == 'IDLE':
            return
            
        elif self.state == 'NAVIGATING':
            curr_waypoint = self.waypoints[self.current_wp_index]
            distance_from_waypoint = math.sqrt((curr_waypoint['x'] - self.current_robot_x)**2 + 
                                               (curr_waypoint['y'] - self.current_robot_y)**2)
            
            if distance_from_waypoint < self.distance_threshold:
                self.get_logger().info(f"Arrived at {curr_waypoint['name']}! Waiting for {curr_waypoint['wait_time']} seconds.")
                self.state = 'WAITING'
                self.wait_start_time = time.time()
                
        elif self.state == 'WAITING':
            curr_waypoint = self.waypoints[self.current_wp_index]
            if (time.time() - self.wait_start_time) >= curr_waypoint['wait_time']:
                self.current_wp_index += 1
                self.start_next_waypoint()

    
def main(args=None):
    rclpy.init(args=args)
    node = WayPointManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


