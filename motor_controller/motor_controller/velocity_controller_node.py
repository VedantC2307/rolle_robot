import time
import serial
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')
        
        self.get_logger().info('Velocity Controller Node Started')
        
        self.create_subscription(Twist, '/desired_velocity', self.cmd_vel_callback, 10)
        
        self.setpoint = Twist()
        self.estimated_vel = Twist()
        self.last_pose = Vector3()
        self.last_time = None
        
        # Serial communication setup (modify port and baud rate)
        self.serial_port = serial.Serial('/dev/ttyS0', 115200, timeout=1)

        self.create_timer(0.1, self.control_loop)

    def cmd_vel_callback(self, msg):
        self.setpoint.linear.x = msg.linear.x  
        self.setpoint.angular.z = msg.angular.z  # Rotation
        self.get_logger().info(f'Received velocity command - linear: {msg.linear.x}')

    def control_loop(self):
        # Check for rotation first
        if abs(self.setpoint.angular.z) > 0:
            # Handle rotation
            rotation_pwm = self.map_to_rotation_pwm(self.setpoint.angular.z)
            command = f"PWM:R:{rotation_pwm}\r\n"
        else:
            # Handle forward/backward
            linear_pwm = self.map_to_pwm(self.setpoint.linear.x)
            command = f"PWM:F:{linear_pwm}\r\n"
        
        # Send single command
        self.serial_port.write(command.encode())

        # Log the command
        self.get_logger().info(f'Sent PWM command: {command.strip()}')

    def map_to_pwm(self, value):
        # Define PWM limits
        min_forward_pwm = 70
        max_forward_pwm = 90
        min_backward_pwm = 70
        max_backward_pwm = 90

        # Map velocity to PWM ranges
        if value > 0:
            pwm = int(min_forward_pwm + (value * (max_forward_pwm - min_forward_pwm)))
            return min(max_forward_pwm, max(min_forward_pwm, pwm))
        elif value < 0:
            pwm = int(-min_backward_pwm + (value * (max_backward_pwm - min_backward_pwm)))
            return max(-max_backward_pwm, min(-min_backward_pwm, pwm))
        return 0

    def map_to_rotation_pwm(self, value):
        # Define PWM limits for rotation (might need different values than forward/backward)
        min_rotation_pwm = 70
        max_rotation_pwm = 90

        # Map angular velocity to PWM ranges
        if value > 0:  # Clockwise rotation
            pwm = int(min_rotation_pwm + (value * (max_rotation_pwm - min_rotation_pwm)))
            return min(max_rotation_pwm, max(min_rotation_pwm, pwm))
        elif value < 0:  # Counter-clockwise rotation
            pwm = int(-min_rotation_pwm + (value * (max_rotation_pwm - min_rotation_pwm)))
            return max(-max_rotation_pwm, min(-min_rotation_pwm, pwm))
        return 0

    def __del__(self):
        # Send zero velocity commands for both motions
        stop_commands = "PWM:F:0\r\nPWM:R:0\r\n"
        try:
            self.serial_port.write(stop_commands.encode())
            self.get_logger().info('Sent stop commands before shutdown')
            self.serial_port.close()
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    controller = VelocityController()
    try:
        rclpy.spin(controller)
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
