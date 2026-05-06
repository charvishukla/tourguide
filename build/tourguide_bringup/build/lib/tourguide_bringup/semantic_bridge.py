#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

# FIX 1: Use the _v3 suffix for Jazzy!
from depthai_ros_msgs_v3.msg import SpatialDetectionArray 

class SemanticLandmarkBridge(Node):
    def __init__(self):
        super().__init__('semantic_bridge')

        self.target_classes = {60, 62, 68, 71, 72, 74}
        self.min_confidence = 0.75

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self) 

        # FIX 2: Create the Subscriber to listen to OAK-D spatial detections
        # Note: Update '/oak/nn/spatial_detections' if your launch file remaps this differently
        self.subscription = self.create_subscription(
            SpatialDetectionArray,
            '/oak/nn/spatial_detections',
            self.detection_callback,
            10
        )

        # FIX 3: Create the Publisher to output the transformed poses
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/semantic_landmarks',
            10
        )

    def detection_callback(self, msg):
        for detection in msg.detections:
            class_id = detection.results[0].id
            score = detection.results[0].score

            if class_id in self.target_classes and score >= self.min_confidence:
                self.process_landmark(detection, class_id)

    def process_landmark(self, detection, class_id):
        try:
            # 1. Get the transform from camera optical frame to robot base_link
            transform = self.tf_buffer.lookup_transform(
                'base_link', 
                detection.header.frame_id, 
                rclpy.time.Time()
            )

            # 2. Prepare the pose in the camera frame
            camera_pose = PoseStamped()
            camera_pose.header = detection.header
            camera_pose.pose.position = detection.position
            camera_pose.pose.orientation.w = 1.0 

            # 3. Transform pose to base_link
            base_link_pose = tf2_geometry_msgs.do_transform_pose(camera_pose, transform)

            # 4. Construct the outgoing message
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'base_link'
            
            pose_msg.pose.pose.position = base_link_pose.pose.position
            pose_msg.pose.pose.orientation = base_link_pose.pose.orientation

            # 5. Initialize Covariance Matrix
            # We use 9999.0 (high uncertainty) for all 36 elements by default
            cov = [9999.0] * 36
            
            # Specifically 'unlock' the XYZ translational axes with high confidence
            cov[0] = 0.05   # X variance
            cov[7] = 0.05   # Y variance
            cov[14] = 0.05  # Z variance
            
            # Assign the full array to the message
            pose_msg.pose.covariance = cov

            # 6. Publish and Log
            self.publisher.publish(pose_msg)
            
            self.get_logger().info(f"Detected {class_id} at {pose_msg.pose.pose.position.x:.2f}, {pose_msg.pose.pose.position.y:.2f}")

        except Exception as e:
            self.get_logger().warn(f"Landmark processing failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SemanticLandmarkBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()