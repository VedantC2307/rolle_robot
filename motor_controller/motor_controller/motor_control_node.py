import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf_transformations import quaternion_from_euler
import tf2_ros
import time
import queue
from threading import Thread, Lock
import math
import os.path

class MotorControlNode(Node):
    # Constants for frame IDs
    ODOM_FRAME_ID = "odom"
    BASE_FRAME_ID = "base_footprint"

    def __init__(self):
        super().__init__('motor_control_node')
        
        # Create a queue for odometry messages
        self.odom_queue = queue.Queue(maxsize=10) 
        self.serial_lock = Lock()
        
        # Increase update rate to better match ESP32's high data rate
        self.sample_time = 20/1000  # 100 Hz - faster processing of incoming data
        
        # Add command timeout tracking for safety
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = 0.5  # seconds before sending zero velocity when no commands received
        self.cmd_watchdog_timer = self.create_timer(0.1, self.cmd_watchdog)  # Check every 100ms
        
        # Use the UART port for RPi-ESP32 communication
        try:
            # Simple check for ESP32 port - try ACM0 first, then ACM1
            if os.path.exists('/dev/ttyACM0'):
                try:
                    self.serial_port = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=1)
                    self.get_logger().info('Connected to /dev/ttyACM0')
                except serial.SerialException:
                    self.serial_port = serial.Serial('/dev/ttyACM1', baudrate=115200, timeout=1)
                    self.get_logger().info('Connected to /dev/ttyACM1')
            else:
                self.serial_port = serial.Serial('/dev/ttyACM1', baudrate=115200, timeout=1)
                self.get_logger().info('Connected to /dev/ttyACM1')
                
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

        # Publisher for odometry data with higher QoS
        self.odom_pub = self.create_publisher(
            Odometry, 
            '/odom', 
            10)
        
        # TF2 broadcaster for odom->base_link transform
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.get_logger().info(f'TF broadcaster initialized for {self.ODOM_FRAME_ID}->{self.BASE_FRAME_ID} transform')
        
        # Subscriber for cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel_rolle',
            self.cmd_vel_callback,
            10)
        self.get_logger().info('Subscribed to /cmd_vel topic')
        
        # Timer for processing queued data
        self.timer = self.create_timer(self.sample_time, self.process_odom_queue)
        
        # Start background thread for reading serial data
        self.serial_thread_running = True
        self.serial_thread = Thread(target=self.read_serial_data, daemon=True)
        self.serial_thread.start()
        
        self.get_logger().info('Motor Control Node initialized')
        self.get_logger().info(f'Serial port opened at {self.serial_port.port} with baudrate {self.serial_port.baudrate}')
        
        # Track last successful communication
        self.last_successful_read = self.get_clock().now()
        
        # Covariance matrices for more accurate odometry
        self.position_covariance = [0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001]
        self.twist_covariance = [0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001]
        
        # Use tf_buffer for smoother transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Last reported odometry for filtering
        self.last_odom = {
            'x': 0.0, 'y': 0.0, 'theta': 0.0,
            'vx': 0.0, 'vy': 0.0, 'omega': 0.0,
            'timestamp': self.get_clock().now()
        }
        
        # Filter parameters
        self.filter_alpha = 0.7  # EMA filter parameter (higher = less filtering)
    
    def cmd_vel_callback(self, msg):
        """
        Callback for processing cmd_vel messages and sending to ESP32
        """
        linear_x = msg.linear.x 
        angular_z = - msg.angular.z
        
        # Format the command as expected by the ESP32
        cmd = f"cmd_vel: {linear_x:.2f},{angular_z:.2f}\n"
        
        try:
            with self.serial_lock:
                self.serial_port.write(cmd.encode('utf-8'))
                self.serial_port.flush()  # Ensure data is sent immediately
            # self.get_logger().debug(f"Sent command to ESP32: {cmd.strip()}")
            self.last_cmd_time = self.get_clock().now()  # Update last command time
        except Exception as e:
            self.get_logger().error(f"Failed to send command to ESP32: {e}")

    def read_serial_data(self):
        """
        Background thread that continuously reads from serial and queues data
        """
        buffer = ""
        
        while self.serial_thread_running:
            try:
                with self.serial_lock:
                    if self.serial_port.in_waiting > 0:
                        new_data = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8', errors='replace')
                        buffer += new_data
                        self.last_successful_read = self.get_clock().now()
                
                # Process any complete lines in the buffer
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    
                    if "odom:" in line:
                        try:
                            # Queue the odometry data to be processed by the main thread
                            if not self.odom_queue.full():
                                self.odom_queue.put_nowait(line)
                            else:
                                # If queue is full, remove oldest item and add new one
                                try:
                                    self.odom_queue.get_nowait()
                                    self.odom_queue.put_nowait(line)
                                except queue.Empty:
                                    pass
                        except Exception as e:
                            pass  # Silently ignore errors in background thread
            
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
            
            # Short sleep to prevent CPU overuse
            time.sleep(0.001)

    def process_odom_queue(self):
        """
        Process queued odometry data in the main thread
        """
        try:
            # Process all queued messages
            messages_processed = 0
            
            while not self.odom_queue.empty() and messages_processed < 5:
                line = self.odom_queue.get_nowait()
                messages_processed += 1
                
                # Process the most recent odometry message
                if "odom:" in line:
                    # Extract everything after "odom:" 
                    data_part = line.split("odom:")[1].strip()
                    data = data_part.split(",")
                    # print(data_part)
                    
                    if len(data) == 6:
                        try:
                            odom_data = [float(value) for value in data]
                            
                            # Apply EMA filter to smooth the values
                            alpha = self.filter_alpha
                            
                            # Get current time
                            current_time = self.get_clock().now()
                            dt = (current_time.nanoseconds - self.last_odom['timestamp'].nanoseconds) / 1e9
                            dt = max(0.001, min(dt, 0.1))  # Limit dt to reasonable range
                            
                            # Only apply filtering if we have previous data and dt is reasonable
                            if dt > 0:
                                filtered_x = alpha * odom_data[0] + (1-alpha) * self.last_odom['x']
                                filtered_y = alpha * odom_data[1] + (1-alpha) * self.last_odom['y']
                                
                                # Special handling for theta to deal with discontinuity at ±π
                                theta_diff = odom_data[2] - self.last_odom['theta']
                                # Normalize the angle difference to [-π, π]
                                if theta_diff > math.pi:
                                    theta_diff -= 2 * math.pi
                                elif theta_diff < -math.pi:
                                    theta_diff += 2 * math.pi
                                filtered_theta = self.last_odom['theta'] + alpha * theta_diff
                                # Keep theta in range [-π, π]
                                if filtered_theta > math.pi:
                                    filtered_theta -= 2 * math.pi
                                elif filtered_theta < -math.pi:
                                    filtered_theta += 2 * math.pi
                                    
                                filtered_vx = alpha * odom_data[3] + (1-alpha) * self.last_odom['vx']
                                filtered_vy = alpha * odom_data[4] + (1-alpha) * self.last_odom['vy']
                                filtered_omega = alpha * odom_data[5] + (1-alpha) * self.last_odom['omega']
                                
                                # Update last_odom with filtered values
                                self.last_odom = {
                                    'x': filtered_x,
                                    'y': filtered_y,
                                    'theta': filtered_theta,
                                    'vx': filtered_vx,
                                    'vy': filtered_vy,
                                    'omega': filtered_omega,
                                    'timestamp': current_time
                                }
                            else:
                                # If this is the first message or time went backwards, use raw values
                                self.last_odom = {
                                    'x': odom_data[0],
                                    'y': odom_data[1],
                                    'theta': odom_data[2],
                                    'vx': odom_data[3],
                                    'vy': odom_data[4],
                                    'omega': odom_data[5],
                                    'timestamp': current_time
                                }
                            
                            # Create and populate the Odometry message
                            odom_msg = Odometry()
                            odom_msg.header.stamp = current_time.to_msg()
                            odom_msg.header.frame_id = self.ODOM_FRAME_ID
                            odom_msg.child_frame_id = self.BASE_FRAME_ID
                            
                            # Set position and orientation from filtered values
                            odom_msg.pose.pose.position.x = self.last_odom['x']
                            odom_msg.pose.pose.position.y = self.last_odom['y']
                            odom_msg.pose.pose.position.z = 0.0
                            
                            # Convert yaw to quaternion
                            quaternion = quaternion_from_euler(0.0, 0.0, self.last_odom['theta'])
                            odom_msg.pose.pose.orientation.x = quaternion[0]
                            odom_msg.pose.pose.orientation.y = quaternion[1]
                            odom_msg.pose.pose.orientation.z = quaternion[2]
                            odom_msg.pose.pose.orientation.w = quaternion[3]
                            
                            # Set covariance (helps with filter fusion)
                            # Fill the 6x6 covariance matrix (position x,y,z and rotation x,y,z)
                            for i in range(3):
                                odom_msg.pose.covariance[i*6+i] = self.position_covariance[i]
                            
                            # Set linear and angular velocities
                            odom_msg.twist.twist.linear.x = self.last_odom['vx']
                            odom_msg.twist.twist.linear.y = self.last_odom['vy']
                            odom_msg.twist.twist.linear.z = 0.0
                            odom_msg.twist.twist.angular.x = 0.0
                            odom_msg.twist.twist.angular.y = 0.0
                            odom_msg.twist.twist.angular.z = self.last_odom['omega']
                            
                            # Set twist covariance
                            for i in range(3):
                                odom_msg.twist.covariance[i*6+i] = self.twist_covariance[i]
                                odom_msg.twist.covariance[(i+3)*6+(i+3)] = self.twist_covariance[i]

                            # Publish the Odometry message
                            self.odom_pub.publish(odom_msg)
                            
                            # Broadcast the transform
                            self.broadcast_transform(
                                self.last_odom['x'], self.last_odom['y'], 0.0,  # x, y, z
                                quaternion[0], quaternion[1], quaternion[2], quaternion[3],  # qx, qy, qz, qw
                                odom_msg.header.stamp
                            )
                            
                        except ValueError as ve:
                            self.get_logger().warning(f"Error converting odometry values: {ve}")
                    else:
                        self.get_logger().debug(f"Invalid odometry data format. Expected 6 values, got {len(data)}: {data}")
                        
            # Check if we've lost communication with the ESP32            
            time_since_last_read = (self.get_clock().now() - self.last_successful_read).nanoseconds / 1e9
            if time_since_last_read > 2.0:  # 2 seconds without data
                self.get_logger().warning(f"No data received from ESP32 for {time_since_last_read:.1f} seconds")
                    
        except Exception as e:
            self.get_logger().error(f"Error processing odometry queue: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

    def cmd_watchdog(self):
        """
        Safety function that sends a zero velocity command if no cmd_vel 
        messages have been received within the timeout period
        """
        time_since_last_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        
        if time_since_last_cmd > self.cmd_timeout:
            # Send zero velocity command as safety measure
            zero_cmd = Twist()
            zero_cmd.linear.x = 0.0
            zero_cmd.angular.z = 0.0
            
            try:
                with self.serial_lock:
                    cmd = f"cmd_vel: 0.00,0.00\n"
                    self.serial_port.write(cmd.encode('utf-8'))
                    self.serial_port.flush()
            except Exception as e:
                self.get_logger().error(f"Failed to send watchdog zero command: {e}")

    def broadcast_transform(self, x, y, z, qx, qy, qz, qw, timestamp):
        """
        Broadcast a transform from odom to base_link frames
        
        Args:
            x, y, z: Position
            qx, qy, qz, qw: Orientation quaternion
            timestamp: ROS timestamp for the transform
        """
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = self.ODOM_FRAME_ID
        t.child_frame_id = self.BASE_FRAME_ID
        
        # Set translation
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        
        # Set rotation
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        # Send the transform
        self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        # Send zero velocity command to ensure the robot stops
        try:
            self.get_logger().info('Stopping robot by sending zero velocity command')
            zero_cmd = Twist()
            self.cmd_vel_callback(zero_cmd)
        except Exception as e:
            self.get_logger().error(f"Failed to send stop command: {e}")
        
        # Clean shutdown of background thread
        self.serial_thread_running = False
        if hasattr(self, 'serial_thread') and self.serial_thread.is_alive():
            self.serial_thread.join(timeout=1.0)
            
        # Close serial port
        if hasattr(self, 'serial_port') and self.serial_port is not None and self.serial_port.is_open:
            with self.serial_lock:
                self.serial_port.close()
            self.get_logger().info('Serial port closed')
            
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()