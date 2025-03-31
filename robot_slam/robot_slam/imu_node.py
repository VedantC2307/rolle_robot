#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import serial
import json
import time
import math

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        
        # Create publisher
        self.imu_publisher = self.create_publisher(Imu, 'imu/data', 10)
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyS0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 55.0)  # Hz
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Initialize serial connection and previous quaternion
        self.ser = None
        self.prev_quaternion = None
        self.connect_serial()
        
        # Create timer for checking serial data
        self.timer = self.create_timer(1.0/self.publish_rate, self.timer_callback)
        
        self.get_logger().info('IMU Publisher node initialized with JSON protocol')
        
    def connect_serial(self):
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0.5
            )
            self.get_logger().info(f'Connected to {self.serial_port} at {self.baud_rate} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to serial port: {e}')
            self.ser = None
            
    def timer_callback(self):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn('Serial connection not available, attempting to reconnect...')
            self.connect_serial()
            return
            
        try:
            # Read line from serial
            line = self.ser.readline().decode('utf-8').strip()
            
            if not line:
                return

            # Replace lowercase nan with null to satisfy JSON parser
            line = line.replace("nan", "null")
            
            # Parse JSON data
            try:
                data = json.loads(line)
                
                # Check if we have all quaternion components
                if all(key in data for key in ['qw', 'qx', 'qy', 'qz']):
                    quaternion = [data['qw'], data['qx'], data['qy'], data['qz']]
                    # Check for None values and use previous value if necessary
                    if any(q is None for q in quaternion):
                        if self.prev_quaternion is not None:
                            self.get_logger().warn('Received nan values, using previous quaternion')
                            quaternion = self.prev_quaternion
                        else:
                            self.get_logger().warn('Received nan values and no previous quaternion available')
                            return
                    self.publish_imu_data(quaternion)
                else:
                    self.get_logger().warn(f'Incomplete quaternion data: {data}')
                    
            except json.JSONDecodeError as e:
                self.get_logger().warn(f'Invalid JSON: {line}, Error: {e}')
                
        except Exception as e:
            self.get_logger().error(f'Error reading from serial: {e}')
            
    def publish_imu_data(self, quaternion_values):
        # Create Imu message
        imu_msg = Imu()
        
        # Set header
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.frame_id
        
        # Set orientation (w, x, y, z from quaternion_values)
        imu_msg.orientation.w = quaternion_values[0]
        imu_msg.orientation.x = quaternion_values[1]
        imu_msg.orientation.y = quaternion_values[2]
        imu_msg.orientation.z = quaternion_values[3]
        
        # Set orientation covariance to -1 (unknown)
        imu_msg.orientation_covariance = [0.0] * 9
        
        # Set velocity to 0.0
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0
        imu_msg.angular_velocity_covariance = [-1.0] * 9
        
        # Set acceleration to 0.0
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 0.0
        imu_msg.linear_acceleration_covariance = [-1.0] * 9
        
        # Publish the message
        self.imu_publisher.publish(imu_msg)
        self.get_logger().debug(f'Published IMU data: w={quaternion_values[0]:.2f}, x={quaternion_values[1]:.2f}, y={quaternion_values[2]:.2f}, z={quaternion_values[3]:.2f}')
        # Update previous quaternion
        self.prev_quaternion = quaternion_values
        
def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        if node.ser is not None and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()