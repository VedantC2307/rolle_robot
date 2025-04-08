#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import signal

class SimpleVelocityPublisher(Node):
    def __init__(self):
        super().__init__('simple_velocity_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel_rolle', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        self.get_logger().info('Simple Velocity Publisher started')
        
        # Set up a shutdown handler
        self.is_shutting_down = False

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.3  # No rotation
        self.publisher.publish(msg)
    
    def stop_robot(self):
        """Send a zero velocity command to stop the robot"""
        if not self.is_shutting_down:
            self.is_shutting_down = True
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            self.publisher.publish(stop_msg)
            self.get_logger().info('Stopping robot')
        
def main():
    rclpy.init()
    node = SimpleVelocityPublisher()
    
    # Set up signal handler for graceful shutdown
    def signal_handler(sig, frame):
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Only do this if we haven't already shut down through the signal handler
        if not node.is_shutting_down:
            node.stop_robot()
            node.destroy_node()
            rclpy.shutdown()
    
    return 0

if __name__ == '__main__':
    main()
