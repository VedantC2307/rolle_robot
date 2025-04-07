import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, PoseStamped, Pose, Twist
from nav_msgs.msg import Odometry
from dotenv import load_dotenv
import os
import math
from robot_controller import config
import numpy as np
import tf_transformations
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
        self.posestamped_publisher = self.create_publisher(PoseStamped, '/rolle/posestamped', 5)
        
        # Publisher for robot velocity
        self.twist_publisher = self.create_publisher(Twist, '/robot_vel', 5)

        # Subscribe to pose topic
        self.pose_subscription = self.create_subscription(
            Pose,
            '/mobile_sensor/pose',
            self.pose_callback,
            5
        )

        # Define the transformation matrix:
        # New X = Old Z, New Y = -Old X, New Z = Old Y.
        self.T = np.eye(4)
        # Fix the transformation matrix - correct the definition
        self.T[:3, :3] = np.array([[0, 0, 1],
                                   [-1, 0, 0],
                                   [0, 1, 0]])
        
        # Store previous pose data for velocity calculation
        self.previous_pose = None
        self.previous_time = None

        self.pose_buffer = []

        # EMA smoothing parameters
        self.ema_alpha = 0.1  # Smoothing factor
        self.linear_vel_ema = None
        self.angular_vel_ema = None

        
        # Timer for computing and publishing twist at regular intervals
        self.create_timer(0.04, self.compute_and_publish_twist)  # 25Hz (changed from 10Hz)

    def pose_callback(self, msg):
        """Callback for receiving pose data"""
        try:
            pose_data = {
                "position": {
                    "x": float(msg.position.x),
                    "y": float(msg.position.y),
                    "z": float(msg.position.z)
                },
                "orientation": {
                    "x": float(msg.orientation.x),
                    "y": float(msg.orientation.y),
                    "z": float(msg.orientation.z),
                    "w": float(msg.orientation.w)
                },
            }
            self.publish_pose(pose_data)
        except Exception as e:
            self.get_logger().error(f"Error processing pose data: {str(e)}")

    def publish_pose(self, pose_data):
        """
        Publish the pose data as a ROS 2 PoseStamped message.

        Parameters:
            pose_data (dict): Pose data extracted from the ZMQ message.
        """
        pos_vec = np.array([
            pose_data["position"]["x"],
            pose_data["position"]["y"],
            pose_data["position"]["z"],
            1.0
        ])
        rotated_pos = np.dot(self.T, pos_vec)

        # --- Transform the orientation ---
        # Get the original quaternion
        q = pose_data["orientation"]
        quat_orig = [q["x"], q["y"], q["z"], q["w"]]

        # 1. Convert transformation matrix to a quaternion (T: sensor frame → robot frame)
        q_T = tf_transformations.quaternion_from_matrix(self.T)

        # 2. Get inverse of the frame rotation
        q_T_inv = tf_transformations.quaternion_inverse(q_T)

        # 3. Apply frame change to the orientation:
        #    new_quat = T * sensor_quat * T⁻¹
        new_quat = tf_transformations.quaternion_multiply(
            tf_transformations.quaternion_multiply(q_T, quat_orig),
            q_T_inv
        )


        pose_msg = PoseStamped()
        # Fill in the header
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "position_sensor_frame"

        # Fill in the pose with position & orientation
        pose_msg.pose.position.x = rotated_pos[0]
        pose_msg.pose.position.y = rotated_pos[1]
        pose_msg.pose.position.z = rotated_pos[2]

        pose_msg.pose.orientation.x = new_quat[0]
        pose_msg.pose.orientation.y = new_quat[1]
        pose_msg.pose.orientation.z = new_quat[2]
        pose_msg.pose.orientation.w = new_quat[3]

        self.posestamped_publisher.publish(pose_msg)
        self.get_logger().debug("Pose data published to '/rolle/posestamped' topic.")
        # Extract yaw from quaternion (assuming planar motion)
        raw_yaw, degrees = self.quaternion_to_yaw(new_quat)
        self.get_logger().debug(f"Current theta: {degrees}")

        # Use current time (in seconds) for the timestamp
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9

        # Append the new sample (timestamp, x, y, raw_yaw) to the pose buffer
        self.pose_buffer.append((t, rotated_pos[0], rotated_pos[1], raw_yaw))
        # Keep only the latest 10 samples
        if len(self.pose_buffer) > 10:
            self.pose_buffer.pop(0)

    def quaternion_to_yaw(self, orientation):
        """Convert quaternion to yaw (rotation around Z-axis) in radians"""
        try:
            if isinstance(orientation, dict):
                qx = float(orientation["x"])
                qy = float(orientation["y"])
                qz = float(orientation["z"])
                qw = float(orientation["w"])
            elif isinstance(orientation, (list, tuple)):
                qx, qy, qz, qw = [float(x) for x in orientation]
            else:
                qx = float(orientation.x)
                qy = float(orientation.y)
                qz = float(orientation.z)
                qw = float(orientation.w)

            t0 = 2.0 * (qw * qz + qx * qy)
            t1 = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw = math.atan2(t0, t1)
            return yaw, math.degrees(yaw)
        except Exception as e:
            self.get_logger().error(f"Error in quaternion conversion: {e}")
            return 0.0, 0.0

    def compute_and_publish_twist(self):
        """
        Compute the robot's twist (linear and angular velocity) using the 5-point backward
        difference method and publish it as a Twist message.
        """
        if len(self.pose_buffer) < 5:
            self.get_logger().debug("Insufficient pose samples for velocity estimation.")
            return
        
        # Use the last 5 samples from the buffer
        samples = self.pose_buffer[-5:]
        times = [s[0] for s in samples]
        xs = [s[1] for s in samples]
        thetas = [s[3] for s in samples]
        
        # Compute average time step h (assuming uniform sampling over these samples)
        h = (times[-1] - times[0]) / 4.0
        if h <= 0:
            self.get_logger().error("Invalid time difference computed.")
            return
        
        # 5-point backward difference formula for the derivatives:
        # f'(t_n) ≈ [25*f(t_n) - 48*f(t_{n-1}) + 36*f(t_{n-2}) - 16*f(t_{n-3}) + 3*f(t_{n-4})] / (12*h)
        dx_dt = (25*xs[4] - 48*xs[3] + 36*xs[2] - 16*xs[1] + 3*xs[0]) / (12.0 * h)
        dtheta_dt = (25*thetas[4] - 48*thetas[3] + 36*thetas[2] - 16*thetas[1] + 3*thetas[0]) / (12.0 * h)
        
        # Transform world-frame velocity (dx_dt) into the robot's frame
        # using the most recent orientation (theta = samples[-1][3])
        theta_current = samples[4][3]
        v_forward = math.cos(theta_current) * dx_dt
        
        # EMA smoothing
        if self.linear_vel_ema is None:
            self.linear_vel_ema = v_forward
            self.angular_vel_ema = dtheta_dt
        else:
            self.linear_vel_ema = self.ema_alpha * v_forward + (1 - self.ema_alpha) * self.linear_vel_ema
            self.angular_vel_ema = self.ema_alpha * dtheta_dt + (1 - self.ema_alpha) * self.angular_vel_ema

        # Create and publish the Twist message with smoothed values
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_vel_ema
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = self.angular_vel_ema
        
        self.twist_publisher.publish(twist_msg)
        self.get_logger().debug(f"Published twist: linear.x = {self.linear_vel_ema:.2f}, angular.z = {self.angular_vel_ema:.2f}")

    def __del__(self):
        """Cleanup if necessary."""
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
