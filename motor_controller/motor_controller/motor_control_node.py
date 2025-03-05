#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from motor_controller.robot_control_motor import ramped_move_motors, ramped_stop_motors, MEC_STRAIGHT_FORWARD, MEC_STRAIGHT_BACKWARD, MEC_ROTATE_CLOCKWISE, MEC_ROTATE_COUNTERCLOCKWISE
from threading import Lock
import math

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        # Subscribers
        self.command_subscription = self.create_subscription(
            String,
            '/motor_command',
            self.command_callback,
            10
        )

        # State variables
        self.is_moving = False

        # Command mapping
        self.command_map = {
            'FORWARD': MEC_STRAIGHT_FORWARD,
            'BACKWARD': MEC_STRAIGHT_BACKWARD,
            'CLOCKWISE': MEC_ROTATE_CLOCKWISE,
            'COUNTERCLOCKWISE': MEC_ROTATE_COUNTERCLOCKWISE,
            'STOP': None,
            'WAIT': None
        }

        self.get_logger().info("Motor Control Node initialized!")

    def command_callback(self, msg):
        """Handle incoming motor commands"""
        command = msg.data.upper()
        
        if command not in self.command_map:
            self.get_logger().error(f"Unknown command: {command}")
            return

        try:
            if command == 'STOP' or command == 'WAIT':
                self.stop_movement()
            else:
                self.execute_movement(command)
        except Exception as e:
            self.get_logger().error(f"Error executing command: {str(e)}")
            self.stop_movement()

    def execute_movement(self, command):
        """Execute the movement command"""
        self.get_logger().info(f"Executing command: {command}")
        self.is_moving = True
        ramped_move_motors(self.command_map[command])

    def stop_movement(self):
        """Safely stop movement"""
        self.is_moving = False
        ramped_stop_motors()
        self.get_logger().info("Motors stopped")

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Motor Control Node")
    finally:
        node.stop_movement()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
