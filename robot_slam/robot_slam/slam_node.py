#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from dotenv import load_dotenv
import os
import math
from robot_controller import config
import zmq
import json
import time

class SLAMNode(Node):  # Renamed from WebSocketListenerNode
    def __init__(self):
        # Load environment variables
        env_path = os.path.join(os.path.dirname(__file__), ".env")
        if os.path.exists(env_path):
           load_dotenv(dotenv_path=env_path)
        
        super().__init__('slam_node')
        self.get_logger().info("SLAM Node initialized!")

        # Publisher for pose data
        self.pose_publisher = self.create_publisher(Vector3, '/pose_data', 5)

        # Initialize ZeroMQ context and socket
        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.SUB)
        self.zmq_socket.connect("tcp://localhost:5556")
        self.zmq_socket.subscribe("sensor_data")
        
        # Create a timer to check for new data
        self.create_timer(0.033, self.receive_zmq_data)  # 100Hz timer

    def receive_zmq_data(self):
        """Receives ZeroMQ data containing position, orientation and camera frame."""
        try:
            topic = self.zmq_socket.recv_string(flags=zmq.NOBLOCK)
            message = self.zmq_socket.recv_string(flags=zmq.NOBLOCK)
            data = json.loads(message)
            
            # Check for the exact structure
            if data and 'pose' in data:
                pose_data = self.extract_pose(data)
                if pose_data:
                    self.publish_pose(pose_data)
                    self.get_logger().debug(f"Timestamp: {data['timestamp']}")
                    
        except zmq.Again:
            # No message available
            pass
        except Exception as e:
            self.get_logger().error(f"Error receiving ZMQ data: {str(e)}")

    def extract_pose(self, data):
        """Extract pose from the ZMQ message with exact JSON structure."""
        try:
            # Extract position and orientation from the exact structure
            pos = data['pose']['position']
            ori = data['pose']['orientation']
            
            # Get values with exact keys
            x = float(pos['x'])
            y = float(pos['y'])
            z = float(pos['z'])
            
            qx = float(ori['x'])
            qy = float(ori['y'])
            qz = float(ori['z'])
            qw = float(ori['w'])

            roll = self.quaternion_to_roll(qx, qy, qz, qw)
            
            return {"x": z, "y": x, "z": roll}

        except Exception as e:
            self.get_logger().error(f"Error extracting pose: {str(e)}")
            return None

    def quaternion_to_roll(self, qx, qy, qz, qw):
        """Convert quaternion to roll (rotation around X-axis) in degrees"""
        try:
            qx = qx
            qy = qy
            qz = qz
            qw = qw

            t0 = 2.0 * (qw * qx + qy * qz)
            t1 = 1.0 - 2.0 * (qx * qx + qy * qy)
            roll = math.degrees(math.atan2(t0, t1))
            return roll
        except Exception as e:
            self.get_logger().error(f"Error in quaternion conversion: {e}")
            return 0.0

    def publish_pose(self, pose_data):
        """
        Publish the pose data as a ROS 2 Pose message.

        Parameters:
            pose_data (dict): Pose data extracted from the WebSocket message.
        """
        msg = Vector3()
        msg.x = pose_data["x"]
        msg.y = pose_data["y"]
        msg.z = pose_data["z"]
        self.pose_publisher.publish(msg)
        self.get_logger().debug("Pose data published to '/pose_data' topic.")

    def __del__(self):
        """Cleanup method to ensure proper shutdown"""
        if hasattr(self, 'zmq_socket'):
            self.zmq_socket.close()
        if hasattr(self, 'zmq_context'):
            self.zmq_context.term()

def main(args=None):
    rclpy.init(args=args)
    node = SLAMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down SLAM Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
