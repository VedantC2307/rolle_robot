import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RobotSpeechNode(Node):
    def __init__(self):
        super().__init__('robot_speech_node')
        
        # Create publishers
        self.tts_publisher = self.create_publisher(
            String,
            '/mobile_sensor/tts',
            3
        )

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
            # Publish to TTS topic
            tts_msg = String()
            tts_msg.data = msg.data
            self.tts_publisher.publish(tts_msg)
            self.get_logger().info(f"Published TTS message: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error publishing TTS message: {e}")

    def destroy_node(self):
        """Cleanup resources"""
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
