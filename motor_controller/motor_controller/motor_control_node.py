#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32
import serial
import math

# Robot and control parameters
WHEEL_BASE = 0.135    # Distance between wheels (m)
MAX_SPEED = 0.4       # Maximum expected wheel speed (m/s)
MIN_PWM = 150         # Minimum PWM (non-zero) to overcome static friction
MAX_PWM = 220         # Maximum PWM
KP = 950             # Proportional gain for the controller
DEADBAND_VEL = 0.1   # Deadband velocity threshold (m/s)

# Serial port configuration (adjust as needed)
SERIAL_PORT = '/dev/ttyS0'
BAUD_RATE = 115200

class DiffDriveController(Node):
    def __init__(self):
        super().__init__('diff_drive_controller')
        
        # Desired wheel speeds (from /cmd_vel)
        self.desired_left = 0.0
        self.desired_right = 0.0
        
        # Actual wheel speeds (from /robot_vel)
        self.actual_left = 0.0
        self.actual_right = 0.0
        
        # Subscribe to /cmd_vel for desired velocities
        self.create_subscription(Twist, '/cmd_vel_rolle', self.cmd_vel_callback, 10)
        self.create_subscription(Twist, '/robot_vel', self.robot_vel_callback, 10)
        
        # Timer for the control loop (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Initialize serial communication for PWM commands
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            self.get_logger().info(f"Serial port {SERIAL_PORT} opened at {BAUD_RATE} baud.")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port {SERIAL_PORT}: {e}")
            self.ser = None
        
        # Publishers for actual wheel speeds
        self.left_speed_pub = self.create_publisher(Float32, '/actual_left_speed', 10)
        self.right_speed_pub = self.create_publisher(Float32, '/actual_right_speed', 10)
    
    def cmd_vel_callback(self, msg: Twist):
        """
        Callback for /cmd_vel. Extracts desired forward and angular velocities,
        and computes the desired left/right wheel speeds using differential drive kinematics.
        """
        v = msg.linear.x   # Desired forward velocity (m/s)
        w = msg.angular.z  # Desired angular velocity (rad/s)
        
        # Differential drive kinematics for desired wheel speeds:
        self.desired_left = v - (w * WHEEL_BASE / 2.0)
        self.desired_right = v + (w * WHEEL_BASE / 2.0)
        
        self.get_logger().debug(f"Desired speeds -> Left: {self.desired_left:.2f} m/s, Right: {self.desired_right:.2f} m/s")
    
    def robot_vel_callback(self, msg: Twist):
        """
        Callback for /robot_vel. Updates actual wheel speeds based on robot velocity.
        """
        v = msg.linear.x   # Actual forward velocity (m/s)
        w = msg.angular.z  # Actual angular velocity (rad/s)
        
        # Apply deadband to linear velocity
        if abs(v) < DEADBAND_VEL:
            v = 0.0
        
        # Differential drive kinematics for actual wheel speeds:
        self.actual_left = v - (w * WHEEL_BASE / 2.0)
        self.actual_right = v + (w * WHEEL_BASE / 2.0)
        
        # Publish actual wheel speeds
        left_msg = Float32()
        right_msg = Float32()
        left_msg.data = float(self.actual_left)
        right_msg.data = float(self.actual_right)
        self.left_speed_pub.publish(left_msg)
        self.right_speed_pub.publish(right_msg)
        
        self.get_logger().info(f"Actual speeds -> Left: {self.actual_left:.2f} m/s, Right: {self.actual_right:.2f} m/s")

    def compute_pwm(self, desired_speed: float, actual_speed: float) -> int:
        """
        Compute the PWM command using a simple proportional controller.
        The error is the difference between desired and actual speeds.
        """
        # If desired speed is 0 (within a small threshold), return 0 PWM
        if abs(desired_speed) < 0.001:
            return 0
            
        error = desired_speed - actual_speed
        pwm = KP * error
        # print(f"Error: {error}, PWM: {pwm}")
        
        # Limit PWM to allowed range
        if pwm > MAX_PWM:
            pwm = MAX_PWM
        elif pwm < -MAX_PWM:
            pwm = -MAX_PWM
        
        # Apply a minimum threshold only if we want to move
        if pwm > 0 and pwm < MIN_PWM:
            pwm = MIN_PWM
        elif pwm < 0 and pwm > -MIN_PWM:
            pwm = -MIN_PWM
        
        return int(pwm)
    
    def control_loop(self):
        """
        Timer callback running at 10 Hz. Uses the actual wheel speeds from /robot_vel,
        applies the P controller to compute PWM commands, and sends the commands via serial.
        """
        # Compute PWM commands using the P controller for each wheel
        left_pwm = self.compute_pwm(self.desired_left, self.actual_left)
        right_pwm = self.compute_pwm(self.desired_right, self.actual_right)
        
        self.get_logger().debug(f"Computed PWM -> Left: {left_pwm}, Right: {right_pwm}")
                
        # Format the command string for the motor controller
        command_str = f"PWM:DIFF:{left_pwm},{right_pwm}\r\n"
        self.get_logger().debug(f"Sending command: {command_str.strip()}")
        
        # Send the command via serial if available
# if self.ser is not None:
#     self.ser.write(command_str.encode())

def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down diff drive controller.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
