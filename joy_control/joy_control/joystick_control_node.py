import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  # Changed: import Twist message
import zmq
import json

class JoystickControlNode(Node):
    def __init__(self):
        super().__init__('joystick_control_node')
        
        # Create publisher for Twist messages on cmd_vel_joy topic
        self.publisher = self.create_publisher(
            Twist,
            'cmd_vel_rolle',
            10
        )

        # Setup ZMQ subscriber for joystick messages
        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.SUB)
        self.zmq_socket.connect("tcp://localhost:5555")
        self.zmq_socket.setsockopt_string(zmq.SUBSCRIBE, "joystick")
        
        # Create timer for checking messages
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info('Joystick control node started')

    def timer_callback(self):
        try:
            topic = self.zmq_socket.recv_string(flags=zmq.NOBLOCK)
            message = self.zmq_socket.recv_string(flags=zmq.NOBLOCK)
            data = json.loads(message)
            
            # Expecting only joystick data with x and y values
            if data.get('type') == 'joystick':
                twist = Twist()
                twist.linear.x = float(data['data']['y']) / 2
                twist.angular.z = float(data['data']['x']) / 2
                # Other twist fields remain 0.0 by default.
                self.publisher.publish(twist)
                self.get_logger().debug(f'Published Twist: linear.x={twist.linear.x}, angular.z={twist.angular.z}')
                
        except zmq.Again:
            # No message available
            pass
        except Exception as e:
            self.get_logger().error(f'Error processing message: {str(e)}')

    def __del__(self):
        # Cleanup ZMQ resources
        if hasattr(self, 'zmq_socket'):
            self.zmq_socket.close()
        if hasattr(self, 'zmq_context'):
            self.zmq_context.term()

def main(args=None):
    rclpy.init(args=args)
    node = JoystickControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
