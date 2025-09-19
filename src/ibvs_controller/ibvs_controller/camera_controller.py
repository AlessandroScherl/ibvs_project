#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraController(Node):
    def __init__(self):
        super().__init__('camera_controller')

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/ibvs/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/ibvs/depth/image_raw', self.depth_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/ibvs/camera_info', self.camera_info_callback, 10)

        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/ibvs/cmd_vel', 10)

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Camera parameters
        self.camera_matrix = None
        self.image = None
        self.depth = None

        # Depth handling for Gazebo bug workaround
        self.default_depth = 1.5  # Default depth in meters (matching initial height)
        self.last_valid_depth = self.default_depth  # Store last valid depth

        # Control timer (10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Camera Controller Node Started')
        self.get_logger().warn('Note: Using depth workaround for Gazebo Sim 8.x depth sensor limitations')

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.get_logger().info('Camera matrix received')

    def image_callback(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')

    def depth_callback(self, msg):
        try:
            depth_raw = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

            # Handle Gazebo depth sensor bug: replace -inf/inf with last valid or default depth
            depth_processed = np.copy(depth_raw)

            # Find valid depth values (not inf or -inf)
            valid_mask = np.isfinite(depth_raw)
            valid_depths = depth_raw[valid_mask]

            if len(valid_depths) > 0:
                # Update last valid depth with median of valid values
                self.last_valid_depth = np.median(valid_depths)

                # Replace invalid values with last valid depth
                depth_processed[~valid_mask] = self.last_valid_depth

                # Log occasionally
                invalid_percent = (1 - np.sum(valid_mask) / valid_mask.size) * 100
                if invalid_percent > 10:  # Only log if significant invalid values
                    self.get_logger().debug(f'Depth: {invalid_percent:.1f}% invalid, using median {self.last_valid_depth:.2f}m')
            else:
                # All values invalid, use last valid or default
                depth_processed[:] = self.last_valid_depth
                self.get_logger().debug(f'All depth values invalid, using {self.last_valid_depth:.2f}m')

            self.depth = depth_processed

        except Exception as e:
            self.get_logger().error(f'Failed to convert depth: {e}')

    def control_loop(self):
        """Main control loop - students will implement IBVS here"""
        if self.image is None:
            return

        # Example: Simple velocity command (students will replace this)
        cmd = Twist()

        # TODO: Students implement feature extraction here
        # features = self.extract_features(self.image)

        # TODO: Students implement visual servoing control law
        # cmd.linear.x, cmd.linear.y, cmd.angular.z = self.compute_control(features)

        # Example movement (remove in student implementation)
        cmd.linear.x = 0.0  # Forward/backward
        cmd.linear.y = 0.0  # Left/right
        cmd.angular.z = 0.0  # Rotation

        self.cmd_pub.publish(cmd)

    def get_depth_at_pixel(self, x, y):
        """Get depth value at specific pixel with fallback to default"""
        if self.depth is None:
            return self.default_depth

        try:
            # Ensure coordinates are within bounds
            h, w = self.depth.shape
            x = max(0, min(x, w - 1))
            y = max(0, min(y, h - 1))

            depth_value = self.depth[int(y), int(x)]

            # Return valid depth or fallback
            return depth_value if np.isfinite(depth_value) else self.last_valid_depth

        except Exception:
            return self.last_valid_depth

    def extract_features(self, image):
        """Students implement feature extraction (ORB, SIFT, etc.)"""
        # TODO: Implement feature extraction
        pass

    def compute_control(self, features, desired_features):
        """Students implement IBVS control law"""
        # TODO: Implement control law
        pass

def main(args=None):
    rclpy.init(args=args)
    controller = CameraController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()