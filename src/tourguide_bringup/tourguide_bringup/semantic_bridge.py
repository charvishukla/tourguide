#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from depthai_ros_msgs.msg import SpatialDetectionArray
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

class SemanticLandmarkBridge(Node):
    def __init__(self):
        super().__init__('semantic_bridge')

        self.target_classes = {58, 62} 
        self.min_confidence = 0.75

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/landmark_poses', 
            10
        )

        self.subscription = self.create_subscription(
            SpatialDetectionArray,
            '/color/yolov4_Spatial_detections', # Check your depthai topic name
            self.detection_callback,
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
            transform = self.tf_buffer.lookup_transform(
                'base_link', 
                detection.header.frame_id, 
                rclpy.time.Time()
            )

            landmark_id = int(class_id * 1000) 
            
            # --- APPLYING THE TF2 TRANSFORM ---
            
            # 1. Put the raw camera coordinate into a PoseStamped message
            camera_pose = PoseStamped()
            camera_pose.header = detection.header
            camera_pose.pose.position = detection.position
            camera_pose.pose.orientation.w = 1.0 # Default orientation

            # 2. Apply the TF matrix to move it to the base_link frame
            base_link_pose = tf2_geometry_msgs.do_transform_pose(camera_pose, transform)

            # 3. Create the final pose message for RTAB-Map
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'base_link'
            
            # Feed the corrected base_link coordinates
            pose_msg.pose.pose.position = base_link_pose.pose.position

            # Set high angular variance (we only trust the XYZ position, not rotation)
            pose_msg.pose.covariance[0] = 0.05  # X variance
            pose_msg.pose.covariance[7] = 0.05  # Y variance
            pose_msg.pose.covariance[14] = 0.05 # Z variance
            pose_msg.pose.covariance[35] = 9999.0 # Yaw variance

            self.publisher.publish(pose_msg)
            self.get_logger().info(f"Published Landmark ID: {landmark_id}")

        except Exception as e:
            self.get_logger().warn(f"TF Transform failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SemanticLandmarkBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()