import rclpy
from rclpy.node import Node
from robot_messages.msg import JoystickCommand
import zmq
import json


class JoystickControlNode(Node):
    def __init__(self):
        super().__init__('joystick_control_node')
        
        # Create publisher
        self.publisher = self.create_publisher(
            JoystickCommand,
            'joystick_commands',
            10
        )

        # Setup ZMQ properly
        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.SUB)
        self.zmq_socket.connect("tcp://localhost:5555")
        self.zmq_socket.setsockopt_string(zmq.SUBSCRIBE, "robot_control")

        self.last_angle = None
        
        # Create timer for checking messages
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info('Joystick control node started')

    def timer_callback(self):
        try:
            topic = self.zmq_socket.recv_string(flags=zmq.NOBLOCK)
            message = self.zmq_socket.recv_string(flags=zmq.NOBLOCK)
            data = json.loads(message)
            
            if data['type'] == 'joystick' and data['data']['command'] == 'move':
                self.last_angle = data['data']['params']['angle']
                direction = data['data']['params']['direction']
                # print('done')
                msg = JoystickCommand()
                msg.command = data['data']['command']
                msg.angle = float(data['data']['params']['angle'])
                
                # Handle direction being None (rotation-only message)
                if direction is None:
                    msg.direction = 'rotate'
                else:
                    # Only set forward/backward if direction is valid
                    if direction.lower() in ['forward', 'backward']:
                        msg.direction = direction
                    else:
                        msg.direction = 'rotate'
                
                self.publisher.publish(msg)
                self.get_logger().info(f'Published: {msg.direction}, {msg.angle}')
                
            elif data['type'] == 'joystick' and data['data']['command'] == 'stop':
                msg = JoystickCommand()
                msg.command = data['data']['command']
                msg.direction = 'stop'
                msg.angle = float(self.last_angle if self.last_angle is not None else 0.0)
                
                self.publisher.publish(msg)
                self.get_logger().info(f'Published: {msg.direction}, {msg.angle}')
                
        except zmq.Again:
            # No message available
            pass
        except Exception as e:
            self.get_logger().error(f'Error processing message: {str(e)}')

    def __del__(self):
        """Cleanup ZMQ resources"""
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
