#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32
import serial
import math

# Robot and control parameters
WHEEL_BASE = 0.14    # Distance between wheels (m)
MAX_SPEED = 0.5       # Maximum expected wheel speed (m/s)
MIN_PWM = 70         
MAX_PWM = 250         # Maximum PWM
DEADBAND_VEL = 0.07   # Deadband velocity threshold (m/s)
ROTATION_GAIN = 12.0
MIN_FEEDFORWARD = 0

# PI controller parameters
KP = 500  
KI = 30
# Removed feedforward gain (KF)

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

        # Integral error terms
        self.left_integral = 0.0
        self.right_integral = 0.0

        self.last_time = self.get_clock().now()
        
        
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

        # Apply same ROTATION_GAIN as in robot_vel_callback
        if abs(v) < 0.01 and abs(w) > 0.01:
            w *= ROTATION_GAIN
        
        # Differential drive kinematics for desired wheel speeds:
        self.desired_left = v - (w * WHEEL_BASE / 2.0)
        self.desired_right = v + (w * WHEEL_BASE / 2.0)
        
        self.get_logger().debug(f"Desired speeds -> Left: {self.desired_left:.2f} m/s, Right: {self.desired_right:.2f} m/s")

    def apply_saturation(self, pwm: float) -> int:
        """
        Saturate the raw pwm value: If the absolute value is below MIN_PWM (and nonzero),
        use MIN_PWM to overcome motor deadband. Otherwise, cap to MAX_PWM.
        """
        if pwm > 0:
            if pwm < MIN_PWM:
                pwm = MIN_PWM
        elif pwm < 0:
            if pwm > -MIN_PWM:
                pwm = -MIN_PWM
                
        # Cap the pwm to the max limits
        pwm = max(min(pwm, MAX_PWM), -MAX_PWM)
        return int(pwm)

    def robot_vel_callback(self, msg: Twist):
        """
        Callback for /robot_vel. Updates actual wheel speeds based on robot velocity.
        """
        v = msg.linear.x   # Actual forward velocity (m/s)
        w = msg.angular.z  # Actual angular velocity (rad/s)
        
        # Apply deadband to linear velocity
        if abs(v) < DEADBAND_VEL:
            v = 0.0

        if abs(w) < 0.05:  # You can tune this threshold (rad/s)
            w = 0.0
        
        # w *= ROTATION_GAIN
        # Differential drive kinematics for actual wheel speeds:
        self.actual_left = v - (w * WHEEL_BASE / 2.0)
        self.actual_right = v + (w * WHEEL_BASE / 2.0)
        
        # # Publish actual wheel speeds
        # left_msg = Float32()
        # right_msg = Float32()
        # left_msg.data = float(self.actual_left)
        # right_msg.data = float(self.actual_right)
        # self.left_speed_pub.publish(left_msg)
        # self.right_speed_pub.publish(right_msg)
        
        # self.get_logger().debug(f"Actual speeds -> Left: {self.actual_left:.2f} m/s, Right: {self.actual_right:.2f} m/s")

    def compute_pwm(self, desired_speed: float, actual_speed: float, wheel: str, dt: float) -> int:
        """
        Compute the PWM command using a PI controller without feedforward.
        dt is the control loop interval.
        """
        # If desired speed is very small, reset the integral and return 0.
        if abs(desired_speed) < 0.002:
            if wheel == "left":
                self.left_integral = 0.0
            else:
                self.right_integral = 0.0
            return 0
            
        error = desired_speed - actual_speed
        max_integral = 100  # Maximum clamp for the integration term

        # Integrate error using the dt computed in the control_loop
        if wheel == "left":
            proposed_integral = self.left_integral + error * dt
            self.left_integral = max(min(proposed_integral, max_integral), -max_integral)
            integral_term = KI * self.left_integral
        else:
            proposed_integral = self.right_integral + error * dt
            self.right_integral = max(min(proposed_integral, max_integral), -max_integral)
            integral_term = KI * self.right_integral

        raw_pwm = 40 +KP * error + integral_term

        # Apply saturation (and deadband) to the raw PWM before sending to motors.
        pwm = self.apply_saturation(raw_pwm)
        return pwm
    
    def control_loop(self):
        # Compute dt once for the control cycle
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert nanoseconds to seconds
        self.last_time = current_time

        # Compute PWM commands for left and right wheels using the same dt.
        left_pwm = self.compute_pwm(self.desired_left, self.actual_left, "left", dt)
        right_pwm = self.compute_pwm(self.desired_right, self.actual_right, "right", dt)
        
        self.get_logger().info(f"Computed PWM -> Left: {left_pwm}, Right: {right_pwm}")
                
        # Format the command string for the motor controller.
        command_str = f"PWM:DIFF:{left_pwm},{right_pwm}\r\n"
        self.get_logger().debug(f"Sending command: {command_str.strip()}")
        
        # Send the command via serial if available.
        if self.ser is not None:
            self.ser.write(command_str.encode())

def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down diff drive controller.")
    finally:
        # Send stop command to motors before shutting down
        if node.ser is not None:
            stop_command = "PWM:DIFF:0,0\r\n"
            node.ser.write(stop_command.encode())
            node.get_logger().info("Sent stop command to motors.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
