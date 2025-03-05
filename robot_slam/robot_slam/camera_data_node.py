#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import zmq
import json
import time

class CameraDataNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.get_logger().info("Camera Data Node initialized!")

        # Publisher for camera data (base64 string)
        self.camera_publisher = self.create_publisher(String, '/base64_image', 5)

        # Initialize ZeroMQ context and socket
        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.SUB)
        self.zmq_socket.connect("tcp://localhost:5556")
        self.zmq_socket.subscribe("sensor_data")
        
        # Create a timer to check for new data
        self.create_timer(0.01, self.receive_zmq_data)  # 10Hz timer

    def receive_zmq_data(self):
        """Receives ZeroMQ data and extracts camera frame."""
        try:
            topic = self.zmq_socket.recv_string(flags=zmq.NOBLOCK)
            message = self.zmq_socket.recv_string(flags=zmq.NOBLOCK)
            data = json.loads(message)
            
            # Extract and publish camera data if available
            if data and 'camera' in data:
                camera_frame = data['camera']
                if camera_frame:
                    self.publish_camera_data(camera_frame)
                    
        except zmq.Again:
            # No message available
            pass
        except Exception as e:
            self.get_logger().error(f"Error receiving ZMQ data: {str(e)}")

    def publish_camera_data(self, camera_frame):
        """Publish the camera frame as a base64 string."""
        msg = String()
        msg.data = camera_frame
        self.camera_publisher.publish(msg)
        self.get_logger().debug("Camera frame published to '/camera_data' topic")

    def __del__(self):
        """Cleanup method to ensure proper shutdown"""
        if hasattr(self, 'zmq_socket'):
            self.zmq_socket.close()
        if hasattr(self, 'zmq_context'):
            self.zmq_context.term()

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
