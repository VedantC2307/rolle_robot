import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import zmq
import sys


class RobotSpeechNode(Node):
    def __init__(self):
        super().__init__('robot_speech_node')
        
        # Initialize instance variables first
        self._context = None
        self._publisher = None
        
        # Then set up ZMQ
        self._context = zmq.Context()
        self._publisher = self._context.socket(zmq.PUB)
        self._publisher.bind("tcp://*:5557")
        self.get_logger().info("ZMQ Publisher bound to port 5557")

        # Create ROS2 subscription
        self.subscription = self.create_subscription(
            String,
            '/robot_speech',
            self.speech_callback,
            3
        )
        self.get_logger().info("Subscribed to /robot_speech topic")

    def speech_callback(self, msg):
        """Handle incoming speech messages"""
        try:
            # Forward the message via ZMQ
            self._publisher.send_multipart([b"tts", msg.data.encode()])
            self.get_logger().debug(f"Forwarded speech message: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error sending message via ZMQ: {e}")

    def destroy_node(self):
        """Cleanup ZMQ resources"""
        if self._publisher:
            self._publisher.close()
        if self._context:
            self._context.term()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotSpeechNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
