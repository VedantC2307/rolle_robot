import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, PoseStamped
from nav_msgs.msg import Odometry
from dotenv import load_dotenv
import os
import math
from robot_controller import config
import zmq
import json
import time

class SLAMNode(Node):  
    def __init__(self):
        # Load environment variables
        env_path = os.path.join(os.path.dirname(__file__), ".env")
        if os.path.exists(env_path):
           load_dotenv(dotenv_path=env_path)
        
        super().__init__('slam_node')
        self.get_logger().info("SLAM Node initialized!")

        # Publisher for pose data
        self.pose_publisher = self.create_publisher(PoseStamped, '/pose_data', 5)

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
            # print(data)
            
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
            # x = float(pos['x'])
            # y = float(pos['y'])
            # z = float(pos['z'])
            
            # qx = float(ori['x'])
            # qy = float(ori['y'])
            # qz = float(ori['z'])
            # qw = float(ori['w'])
            
            return {
                "position": {
                    "x": float(pos['z']),
                    "y": float(pos['x']),
                    "z": float(pos['y'])
                },
                "orientation": {
                    "x": float(ori['x']),
                    "y": float(ori['y']),
                    "z": float(ori['z']),
                    "w": float(ori['w'])
                }
            }

        except Exception as e:
            self.get_logger().error(f"Error extracting pose: {str(e)}")
            return None


    def publish_pose(self, pose_data):
        """
        Publish the pose data as a ROS 2 PoseStamped message.

        Parameters:
            pose_data (dict): Pose data extracted from the ZMQ message.
        """
        pose_msg = PoseStamped()
        # Fill in the header
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "odom"

        # Fill in the pose with position & orientation
        pose_msg.pose.position.x = - pose_data["position"]["x"]
        pose_msg.pose.position.y = - pose_data["position"]["y"]
        pose_msg.pose.position.z = pose_data["position"]["z"]

        pose_msg.pose.orientation.x = pose_data["orientation"]["x"]
        pose_msg.pose.orientation.y = pose_data["orientation"]["y"]
        pose_msg.pose.orientation.z = pose_data["orientation"]["z"]
        pose_msg.pose.orientation.w = pose_data["orientation"]["w"]

        self.pose_publisher.publish(pose_msg)
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
