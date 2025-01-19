#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from geometry_msgs.msg import Pose
from robot_messages.action import MotorControl
from sensors.robot_control_motor import move_motors, stop_motors, MEC_STRAIGHT_FORWARD
import time
from std_msgs.msg import Float32


class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        # ROS 2 Action Server
        self.action_server = ActionServer(
            self,
            MotorControl,  # Replace with your custom action
            'motor_control',
            self.execute_callback,
            cancel_callback=self.cancel_callback
        )

        # Subscriber to /pose_data
        self.subscription = self.create_subscription(
            Pose,
            '/pose_data',
            self.pose_callback,
            10,
        )

        self.subscription_ultrasonic = self.create_subscription(
            Float32,
            '/ultrasonic_distance',
            self.ultrasonic_callback,
            10,
        )

        # Constants for safety
        self.SAFE_DISTANCE_THRESHOLD = 0.35  # 35 cm minimum safe distance
        self.ULTRASONIC_TIMEOUT = 1.0  # 1 second timeout for ultrasonic readings
        self.last_ultrasonic_reading_time = time.time()

        # Pose tracking
        self.current_pose = None  # Updated with /pose_data
        self.start_pose = None
        self.goal_distance = None
        self.goal_handle = None
        self.object_distance = None
        self.is_obstacle_detected = False

        self.get_logger().info("Motor Control Node initialized!")

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        stop_motors()
        self.reset_action_state()
        return CancelResponse.ACCEPT


    def ultrasonic_callback(self, msg):
        try:
            self.object_distance = msg.data
            self.last_ultrasonic_reading_time = time.time()
            
            # Update obstacle detection status
            self.is_obstacle_detected = self.object_distance < self.SAFE_DISTANCE_THRESHOLD
            
            if self.is_obstacle_detected:
                self.get_logger().warn(f"Obstacle detected at {self.object_distance:.2f} meters")
                stop_motors()
                
            self.get_logger().debug(f"Ultrasonic distance: {self.object_distance:.2f} meters")
            
        except Exception as e:
            self.get_logger().error(f"Error in ultrasonic callback: {str(e)}")
            self.is_obstacle_detected = True  # Fail safe: assume obstacle present on error

    def check_ultrasonic_health(self):
        """
        Check if ultrasonic sensor data is fresh and valid.
        """
        if self.object_distance is None:
            return False
            
        current_time = time.time()
        if current_time - self.last_ultrasonic_reading_time > self.ULTRASONIC_TIMEOUT:
            self.get_logger().warn("Ultrasonic sensor data timeout")
            return False
            
        return True

    def pose_callback(self, msg):
        """
        Callback to update the current pose.
        """
        self.current_pose = msg
        # self.get_logger().info(f"Received pose update - Position Z: {msg.position.z}")

        # If there's an active action goal, process movement logic
        if self.goal_handle is not None and self.start_pose is not None:
            self.process_movement()
        else:
            self.get_logger().debug("No active goal or start pose not set")

    def execute_callback(self, goal_handle):
        """
        Callback for the action server to process motor control commands.
        """
        self.get_logger().info("Received action goal.")
        command = goal_handle.request.command
        distance_to_travel = goal_handle.request.distance

        if command == "MOVE_FORWARD":
            self.get_logger().info(f"Command: MOVE_FORWARD, Distance: {distance_to_travel} m")

            # Wait for initial pose data if not available
            timeout = 5.0
            start_time = time.time()
            while self.current_pose is None:
                self.get_logger().warn("Waiting for initial pose data...")
                time.sleep(0.1)
                if time.time() - start_time > timeout:
                    self.get_logger().error("Timeout waiting for pose data")
                    goal_handle.abort()
                    return MotorControl.Result(success=False)

            # Initial safety check
            if not self.check_ultrasonic_health() or self.is_obstacle_detected:
                self.get_logger().error("Cannot start movement: Ultrasonic sensor not ready or obstacle detected")
                goal_handle.abort()
                return MotorControl.Result(success=False)

            # Initialize goal tracking
            self.start_pose = self.current_pose
            self.goal_distance = distance_to_travel
            self.goal_handle = goal_handle

            self.get_logger().info("Starting forward motion...")
            move_motors(90, 90, 90, 90, MEC_STRAIGHT_FORWARD)

            # Monitor the movement
            movement_timeout = 10.0
            start_time = time.time()
            while time.time() - start_time < movement_timeout:
                if goal_handle.is_cancel_requested:
                    self.get_logger().info("Goal cancelled")
                    stop_motors()
                    goal_handle.canceled()
                    self.reset_action_state()
                    return MotorControl.Result(success=False)

                if self.process_movement():  # Check if goal is completed
                    goal_handle.succeed()
                    self.get_logger().info("Action completed.")
                    return MotorControl.Result(success=True)

                time.sleep(0.1)  # Prevent CPU overload

            # Movement timed out
            self.get_logger().error("Movement timed out")
            stop_motors()
            goal_handle.abort()
            self.reset_action_state()
            return MotorControl.Result(success=False)

        else:
            self.get_logger().warning(f"Unknown command: {command}")
            goal_handle.abort()
            return MotorControl.Result(success=False)

    def process_movement(self):
        """
        Process movement logic based on pose and distance traveled.
        Returns True if the goal is completed, False otherwise.
        """
        try:
            if self.current_pose is None or self.start_pose is None:
                self.get_logger().warning("Pose data unavailable. Waiting for updates...")
                return False

            if not self.check_ultrasonic_health() or self.is_obstacle_detected:
                raise Exception("Ultrasonic sensor data invalid or obstacle detected")

            traveled_distance = self.calculate_distance(self.start_pose, self.current_pose)
            self.get_logger().info(f"Current distance traveled: {traveled_distance:.3f} meters")
            self.get_logger().info(f"Ultrasonic sensor distance: {self.object_distance:.2f} meters")

            feedback = MotorControl.Feedback()
            feedback.status = f"Traveled: {traveled_distance:.2f} meters"
            self.goal_handle.publish_feedback(feedback)
            self.get_logger().info(feedback.status)

            safe_distance = traveled_distance + 0.3
            if safe_distance >= self.goal_distance:
                self.get_logger().info(f"Target distance {self.goal_distance:.2f} meters reached.")
                stop_motors()
                self.reset_action_state()
                return True  # Goal completed

            return False  # Goal not yet completed

        except Exception as e:
            self.get_logger().error(f"Error during movement processing: {str(e)}")
            stop_motors()
            if self.goal_handle:
                try:
                    self.goal_handle.abort()
                except Exception as abort_error:
                    self.get_logger().error(f"Error aborting the goal: {str(abort_error)}")
            self.reset_action_state()
            return False  # Indicate failure

    def calculate_distance(self, start_pose, current_pose):
        """
        Calculate the Euclidean distance traveled along the z-axis.
        """
        z_start = start_pose.position.z
        z_current = current_pose.position.z
        distance = abs(z_current - z_start)
        self.get_logger().info(f"Distance calculation - Start Z: {z_start}, Current Z: {z_current}, Distance: {distance}")
        return distance
        # return abs(z_current - z_start)

    def reset_action_state(self):
        """
        Reset action-related variables after completion or failure.
        """
        self.start_pose = None
        self.goal_distance = None
        self.goal_handle = None


def main(args=None):
    rclpy.init(args=args)

    # Create the Motor Control Node
    node = MotorControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Motor Control Node.")
    finally:
        stop_motors()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
