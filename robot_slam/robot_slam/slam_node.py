#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import asyncio
import ssl
import websockets
import json
from threading import Thread
from geometry_msgs.msg import Vector3
from dotenv import load_dotenv
import os
import math
from robot_controller import config

class WebSocketListenerNode(Node):
    def __init__(self):
         # Load environment variables
        env_path = os.path.join(os.path.dirname(__file__), ".env")
        if os.path.exists(env_path):
           load_dotenv(dotenv_path=env_path)
        
        super().__init__('websocket_listener_node')
        self.get_logger().info("WebSocket Position Listener Node initialized!")

        # Use the IP address from the environment or default
        ip_address =  config.IP_ADDRESS # os.getenv("IP_ADDRESS")

        # WebSocket URL
        self.ws_slam_url = f"wss://{ip_address}:8888/xr-slam-client"

        # Publisher for pose data
        self.pose_publisher = self.create_publisher(Vector3, '/pose_data', 5)

        # Start the asyncio WebSocket listener in a separate thread
        self.loop = asyncio.new_event_loop()

        # Create and start a thread for running the asyncio loop
        self.websocket_thread = Thread(target=self.run_async_loop, daemon=True)
        self.websocket_thread.start()

    def run_async_loop(self):
        """Set the event loop for this thread and run the WebSocket listener"""
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.listen_to_websocket())

    async def listen_to_websocket(self):
        """
        Connect to the WebSocket server and continuously listen for pose updates.
        Logs the received pose data and connection status for debugging.
        """
        ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        ssl_context.check_hostname = False
        ssl_context.verify_mode = ssl.CERT_NONE

        while True:
            try:
                self.get_logger().info(f"Attempting to connect to WebSocket server at {self.ws_slam_url}...")

                # Connect to WebSocket
                async with websockets.connect(self.ws_slam_url, ssl=ssl_context) as websocket:
                    self.get_logger().info("Connected to WebSocket server successfully.")

                    async for message in websocket:
                        try:
                            self.get_logger().debug(f"Raw message received: {message}")

                            # Parse incoming JSON messages
                            data = json.loads(message)

                            # Log the parsed JSON structure
                            self.get_logger().debug(f"Parsed JSON data: {data}")

                            # Extract the pose or relevant data
                            pose_data = self.extract_pose(data)

                            if pose_data:
                                # self.publish_pose(pose_data)
                                # Log the pose data
                                self.get_logger().debug(f"Extracted Pose Data: {pose_data}")
                                self.publish_pose(pose_data)
                            else:
                                self.get_logger().warning("Pose data extraction failed or incomplete.")

                        except json.JSONDecodeError:
                            self.get_logger().error(f"Invalid JSON received: {message}")
                        except Exception as e:
                            self.get_logger().error(f"Error processing pose data: {str(e)}")

            except websockets.exceptions.ConnectionClosed as e:
                self.get_logger().warning(f"WebSocket connection closed unexpectedly: {str(e)}. Retrying...")
                await asyncio.sleep(1)
            except Exception as e:
                self.get_logger().error(f"WebSocket error occurred: {str(e)}. Retrying...")
                await asyncio.sleep(1)

    def extract_pose(self, message):
        """
        Extract the robot's pose (x, y, yaw) from a filtered WebSocket message.

        Parameters:
            message (dict): JSON object containing robot pose data.

        Returns:
            dict: Pose data as a dictionary.
        """
        try:
            # Log extraction process
            self.get_logger().debug(f"Attempting to extract pose from message: {message}")

            x = float(message.get("x", 0))
            y = float(message.get("y", 0))
            z = float(message.get("z", 0))
            qx = float(message.get("qx", 0))
            qy = float(message.get("qy", 0))
            qz = float(message.get("qz", 0))
            qw = float(message.get("qw", 0))

            roll = self.quaternion_to_roll(qx,qy,qz,qw)

            # Log successful extraction
            self.get_logger().debug(f"Successfully extracted pose: x={x}, y={y}, z={z}, qx={qx}, qy={qy}, qz={qz}, qw={qw}")
            return {"x": z, "y": x, "z": roll,}

        except Exception as e:
            self.get_logger().error(f"Error extracting pose: {str(e)}")
            return None
        
    def quaternion_to_roll(self, qx,qy,qz,qw):
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

def main(args=None):
    rclpy.init(args=args)
    # Create the WebSocket listener node
    node = WebSocketListenerNode()
    try:
        # Spin the node to keep it running
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down WebSocket Listener Node.")
    finally:
        # Shutdown and cleanup
        node.loop.call_soon_threadsafe(node.loop.stop) 
        # node.loop.stop()
        node.websocket_thread.join()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
