#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
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
        distance_to_travel = goal_handle.request.distance  # Target distance in cm

        if command == "MOVE_FORWARD":
            self.get_logger().info(f"Command: MOVE_FORWARD, Distance: {distance_to_travel} cm")

            # Wait for initial pose data if not available
            timeout = 5.0  # 5 seconds timeout
            start_time = time.time()
            while self.current_pose is None:
                self.get_logger().warn("Waiting for initial pose data...")
                time.sleep(0.1)
                if time.time() - start_time > timeout:
                    self.get_logger().error("Timeout waiting for pose data")
                    goal_handle.abort()
                    return MotorControl.Result(success=False)
                
            
            # Initial safety check
            if not self.check_ultrasonic_health():
                self.get_logger().error("Cannot start movement: Ultrasonic sensor not ready")
                goal_handle.abort()
                return MotorControl.Result(success=False)

            if self.is_obstacle_detected:
                stop_motors()
                self.get_logger().error("Cannot start movement: Obstacle detected")
                goal_handle.abort()
                return MotorControl.Result(success=False)

            # Initialize goal tracking
            self.start_pose = self.current_pose
            self.goal_distance = distance_to_travel
            self.goal_handle = goal_handle

            self.get_logger().info("Starting forward motion...")
            move_motors(90, 90, 90, 90, MEC_STRAIGHT_FORWARD)

            # Monitor the movement until completion or timeout
            timeout = 30.0  # 30 seconds timeout for movement
            start_time = time.time()
            
            while not goal_handle.is_cancel_requested():
                # Check for timeout
                if time.time() - start_time > timeout:
                    self.get_logger().error("Movement timed out")
                    stop_motors()
                    goal_handle.abort()
                    self.reset_action_state()
                    return MotorControl.Result(success=False)
                    
                # Sleep briefly to prevent CPU overload
                time.sleep(0.1)
                
                # Check if goal was completed by process_movement
                if self.goal_handle is None:
                    break

            # Handle cancellation
            if goal_handle.is_cancel_requested():
                self.get_logger().info("Goal cancelled")
                stop_motors()
                goal_handle.canceled()
                self.reset_action_state()
                return MotorControl.Result(success=False)

            self.get_logger().info("Action completed.")
            return MotorControl.Result(success=True)
        else:
            self.get_logger().warning(f"Unknown command: {command}")
            goal_handle.abort()
            return MotorControl.Result(success=False)

    def process_movement(self):
        """
        Process movement logic based on pose and distance traveled.
        """
        try:
            # Check if pose data is available
            if self.current_pose is None or self.start_pose is None:
                self.get_logger().warning("Pose data unavailable. Waiting for updates...")
                return
            
            if not self.check_ultrasonic_health():
                raise Exception("Ultrasonic sensor data invalid")

            if self.is_obstacle_detected:
                raise Exception(f"Obstacle detected at {self.object_distance:.2f} meters")

            # Calculate distance traveled
            traveled_distance = self.calculate_distance(self.start_pose, self.current_pose)
            self.get_logger().info(f"Current distance traveled: {traveled_distance:.3f} meters")

            self.get_logger().info(f"Ultrasonic sesnor distance: {self.object_distance:.2f} meters")

            # if self.object_distance is None:
            #     self.get_logger().warn("Ultrasonic distance unavailable. Assuming clear path.")
            # else:
            #     if self.object_distance < 0.35:  # 35 cm safety margin
            #         self.get_logger().error("Obstacle detected. Stopping motors.")
            #         stop_motors()
            #         self.goal_handle.abort()
            #         self.reset_action_state()
            #         return

            # Publish feedback to the client
            feedback = MotorControl.Feedback()
            feedback.status = f"Traveled: {traveled_distance:.2f} meters"
            self.goal_handle.publish_feedback(feedback)
            self.get_logger().info(feedback.status)

            safe_distance = traveled_distance + 0.3
            # Stop if the target distance is reached
            if safe_distance >= self.goal_distance:
                self.get_logger().info(f"Target distance {self.goal_distance:.2f} meters reached.")
                stop_motors()
                if not self.goal_handle.is_cancel_requested():
                    self.goal_handle.succeed()
                self.reset_action_state()

        except Exception as e:
            self.get_logger().error(f"Error during movement processing: {str(e)}")
            stop_motors()
            if self.goal_handle:
                try:
                    self.goal_handle.abort()
                except Exception as abort_error:
                    self.get_logger().error(f"Error aborting the goal: {str(abort_error)}")
            self.reset_action_state()

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
