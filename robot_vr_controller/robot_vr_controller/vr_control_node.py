import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Vector3
from robot_messages.msg import JoystickCommand

class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control')
        
        # Subscribers
        self.joystick_sub = self.create_subscription(
            JoystickCommand, 
            '/joystick_commands', 
            self.joystick_callback, 
            10
        )
        
        self.pose_sub = self.create_subscription(
            Vector3, 
            '/pose_data', 
            self.pose_callback, 
            10
        )
        
        # Publisher for motor commands
        self.command_pub = self.create_publisher(
            String,
            '/motor_command',
            10
        )
        
        self.target_yaw = 0.0
        self.current_yaw = 0.0
        self.movement_cmd = None
        
        self.get_logger().info('VR Control Node has started')

    def joystick_callback(self, msg):
        self.movement_cmd = msg.direction  # 'forward' or 'backward'
        self.target_yaw = msg.angle  # VR yaw angle
        self.send_command()

    def pose_callback(self, msg):
        self.current_yaw = msg.z  # Robot's current yaw
        self.send_command()

    def send_command(self):
        if self.movement_cmd is None:
            return
            
        command = self.movement_cmd
        yaw_diff = self.target_yaw - self.current_yaw
        
        if abs(yaw_diff) > 5:  # Small threshold to avoid jitter
            if yaw_diff > 0:
                command = 'clockwise'
            else:
                command = 'anticlockwise'

        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        self.get_logger().info(f"Sent command: {command}, yaw_diff: {yaw_diff}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
