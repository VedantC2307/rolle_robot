import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, PoseStamped, Pose
from nav_msgs.msg import Odometry
from dotenv import load_dotenv
import os
import math
from robot_controller import config

class SLAMNode(Node):  
    def __init__(self):
        # Load environment variables
        env_path = os.path.join(os.path.dirname(__file__), ".env")
        if os.path.exists(env_path):
           load_dotenv(dotenv_path=env_path)
        
        super().__init__('slam_node')
        self.get_logger().info("SLAM Node initialized!")

        # Publisher for pose data
        self.posestamped_publisher = self.create_publisher(PoseStamped, '/rolle/posestamped', 5)
        self.pose_publisher = self.create_publisher(Vector3, '/rolle/pose', 5)

        # Subscribe to pose topic
        self.pose_subscription = self.create_subscription(
            Pose,
            '/mobile_sensor/pose',
            self.pose_callback,
            5
        )

    def pose_callback(self, msg):
        """Callback for receiving pose data"""
        try:
            pose_data = {
                "position": {
                    "x": float(msg.position.x),
                    "y": float(msg.position.z),
                    "z": float(msg.position.y)
                },
                "orientation": {
                    "x": float(msg.orientation.x),
                    "y": float(msg.orientation.y),
                    "z": float(msg.orientation.z),
                    "w": float(msg.orientation.w)
                },
                "roll": {
                    "roll": self.quaternion_to_roll(msg.orientation)
                }
            }
            self.publish_pose(pose_data)
        except Exception as e:
            self.get_logger().error(f"Error processing pose data: {str(e)}")

    def quaternion_to_roll(self, orientation):
        """Convert quaternion to roll (rotation around X-axis) in degrees"""
        try:
            qx = float(orientation.x)
            qy = float(orientation.y)
            qz = float(orientation.z)
            qw = float(orientation.w)

            t0 = 2.0 * (qw * qx + qy * qz)
            t1 = 1.0 - 2.0 * (qx * qx + qy * qy)
            roll = math.degrees(math.atan2(t0, t1))
            return roll
        except Exception as e:
            self.get_logger().error(f"Error in quaternion conversion: {e}")
            return 0.0

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
        pose_msg.pose.position.x = - pose_data["position"]["y"]
        pose_msg.pose.position.y = - pose_data["position"]["x"]
        pose_msg.pose.position.z = pose_data["position"]["z"]

        pose_msg.pose.orientation.x = pose_data["orientation"]["x"]
        pose_msg.pose.orientation.y = pose_data["orientation"]["y"]
        pose_msg.pose.orientation.z = pose_data["orientation"]["z"]
        pose_msg.pose.orientation.w = pose_data["orientation"]["w"]

        self.posestamped_publisher.publish(pose_msg)
        self.get_logger().debug("Pose data published to '/rolle/posestamped' topic.")

        msg = Vector3()
        msg.x = pose_data["position"]["x"]
        msg.y = pose_data["position"]["y"]
        msg.z = pose_data["roll"]["roll"]
        self.pose_publisher.publish(msg)
        self.get_logger().debug("Pose data published to '/rolle/pose' topic.")

    def __del__(self):
        """Cleanup method to ensure proper shutdown"""
        pass

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
