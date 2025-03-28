import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus2
import time

class BNO08Node(Node):
    def __init__(self):
        super().__init__('bno08_node')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        # Publish sensor data every 0.1 seconds
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Initialize the I²C bus (bus 1 is common for Raspberry Pi)
        self.bus = smbus2.SMBus(1)
        self.address = 0x4A  # Update with your sensor's I²C address
        
        # Run any necessary sensor initialization (refer to your sensor datasheet)
        self.initialize_sensor()

    def initialize_sensor(self):
        """
        Initialize the BNO08 sensor.
        Depending on your sensor and configuration, you might need to:
        - Reset the sensor
        - Configure operating modes
        - Set data output formats
        This function should send the proper configuration commands.
        """
        # Example: write to a register to start sensor in a desired mode
        # self.bus.write_byte_data(self.address, register_address, value)
        pass

    def timer_callback(self):
        """
        Read sensor data from the BNO08 and publish an Imu message.
        Replace the I²C read sequence below with the correct registers and data lengths 
        as defined by your sensor's datasheet.
        """
        try:
            # Example: Read 6 bytes of data (adjust register and length as needed)
            data = self.bus.read_i2c_block_data(self.address, 0x00, 6)
            
            imu_msg = Imu()
            # Convert raw sensor data to physical values
            # Here we assume that data bytes represent acceleration values
            imu_msg.linear_acceleration.x = self.convert_acc(data[0], data[1])
            imu_msg.linear_acceleration.y = self.convert_acc(data[2], data[3])
            imu_msg.linear_acceleration.z = self.convert_acc(data[4], data[5])
            
            # Populate header with the current time and frame id
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'
            
            self.publisher_.publish(imu_msg)
            self.get_logger().info('Published IMU data')
        except Exception as e:
            self.get_logger().error(f'Error reading from sensor: {e}')

    def convert_acc(self, low_byte, high_byte):
        """
        Convert two bytes of data into a signed acceleration value.
        The scaling factor should be adjusted according to your sensor's specification.
        """
        raw_value = (high_byte << 8) | low_byte
        # Convert to signed 16-bit integer
        if raw_value > 32767:
            raw_value -= 65536
        # Example scale factor (update based on datasheet)
        scale = 0.01  
        return raw_value * scale

def main(args=None):
    rclpy.init(args=args)
    node = BNO08Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
