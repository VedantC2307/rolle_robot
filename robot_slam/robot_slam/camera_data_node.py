#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import base64

class CameraDataNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.get_logger().info("Camera Data Node initialized!")

        # Publisher for camera data (base64 string)
        self.camera_publisher = self.create_publisher(String, '/base64_image', 5)
        
        # Subscribe to compressed image topic
        self.image_subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.image_callback,
            5)

    def image_callback(self, msg):
        """Convert compressed image to base64 and publish"""
        try:
            # Convert compressed image data to base64 string
            base64_str = base64.b64encode(msg.data).decode('utf-8')
            self.publish_camera_data(base64_str)
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

    def publish_camera_data(self, camera_frame):
        """Publish the camera frame as a base64 string."""
        msg = String()
        msg.data = camera_frame
        self.camera_publisher.publish(msg)
        self.get_logger().debug("Camera frame published to '/base64_image' topic")

def main(args=None):
    rclpy.init(args=args)
    node = CameraDataNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Camera Data Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
