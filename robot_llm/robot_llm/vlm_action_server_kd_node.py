import rclpy
from rclpy.node import Node
import rclpy.action
import json
from robot_messages.action import LLMTrigger
from std_msgs.msg import String
from geometry_msgs.msg import Vector3, Twist
from robot_llm.robot_control_openai import LLMClient
from robot_controller import config
from threading import Lock
import math
from robot_llm.helper_functions_v2 import process_llm_result, capture_image
import asyncio
import zmq  # Add ZMQ import
import time

class LLMImageActionServer(Node):
    def __init__(self):
        super().__init__('llm_image_action_server')
        self.get_logger().info("Started LLM Image Action Server")
     
        self.latest_image_data = None
        self.pose_lock = Lock()
        
        # Constants
        # self.SAFE_DISTANCE_THRESHOLD = 0.20  # meters
        self.MOVEMENT_CHECK_RATE = 0.05  # seconds (20 Hz)

        self.llm_client = LLMClient()  # Initialize LLMClient
        # Initialize action server
        self.action_server = rclpy.action.ActionServer(
            self,
            LLMTrigger,
            'llm_interaction',
            self.execute_callback
        )

        # Initialize ZeroMQ connection
        self.zmq_port = 5555
        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.REQ)
        self.zmq_socket.connect(f"tcp://192.168.0.176:{self.zmq_port}")
        self.get_logger().info(f"Connected to ZMQ server on port {self.zmq_port}")

        self.camera_subscription = self.create_subscription(
            String,
            '/base64_image',
            self.camera_callback,
            5
        )

        self.pose_subscription = self.create_subscription(
            Vector3,
            '/rolle/pose',
            self.pose_callback,
            10
        )

        # Create a timer for movement monitoring
        self.movement_timer = self.create_timer(
            self.MOVEMENT_CHECK_RATE,
            self.movement_control_callback
        )

        self.speech_publisher = self.create_publisher(String, '/mobile_sensor/tts', 2)

        # self.publish_motor_command = self.create_publisher(String, '/motor_command', 10)
        self.publish_motor_velocity = self.create_publisher(Twist, '/cmd_vel_rolle', 10)  # Fixed typo


        #Initialize zeromq
        
        self.current_pose = None
        self.is_moving = False
        self.start_pose = None
        self.target_distance = None
        self.target_rotation = None
        self.movement_type = None

        # Add initialization for rotation variables
        self.rotation_ramp_done = False
        self.rotation_start_time = None

        self.INITIAL_ROTATION_SPEED = 0.4  # Initial high rotation speed
        self.CRUISE_ROTATION_SPEED = 0.1

    def pose_callback(self, msg):
        """Thread-safe pose update"""
        with self.pose_lock:
            self.current_pose = msg
            self.get_logger().debug(f"Position Z updated: {msg.z}")

    def camera_callback(self, msg):
        """Thread-safe image update"""
        with self.pose_lock:
            self.latest_image_data = msg.data  # Store just the string data
            self.get_logger().debug("Received new image data")

    def send_velocity_command(self, command):
        """Send velocity command to the robot"""
        vel_msg = Twist()
        if command == "FORWARD":
            vel_msg.linear.x = 0.1
        elif command == "BACKWARD":
            vel_msg.linear.x = -0.1
        elif command == "CLOCKWISE":
            if not self.rotation_ramp_done:
                vel_msg.angular.z = -self.INITIAL_ROTATION_SPEED  # Start with high speed
                self.rotation_start_time = self.get_clock().now()
                self.rotation_ramp_done = False
            else:
                vel_msg.angular.z = -self.CRUISE_ROTATION_SPEED  # Continue with lower speed
        elif command == "ANTICLOCKWISE":
            if not self.rotation_ramp_done:
                vel_msg.angular.z = self.INITIAL_ROTATION_SPEED  # Start with high speed
                self.rotation_start_time = self.get_clock().now()
                self.rotation_ramp_done = False
            else:
                vel_msg.angular.z = self.CRUISE_ROTATION_SPEED  # Continue with lower speed
        else:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
        
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        
        self.publish_motor_velocity.publish(vel_msg)  # Fixed typo
        self.get_logger().info(f"Published velocity command: {command}")

    async def execute_callback(self, goal_handle):
        """Executes the action when goal is received."""
        self.get_logger().info(f"Triggering LLM with Goal: {goal_handle.request.prompt}")

        # Reset any leftover state
        self.is_moving = False
        self.start_pose = None
        self.target_distance = None
        self.target_rotation = None
        self.movement_type = None
        
        feedback_msg = LLMTrigger.Feedback()
        result = LLMTrigger.Result()
        
        prompt = goal_handle.request.prompt
        
        with self.pose_lock:
            b64_image = self.latest_image_data
        
        if b64_image is None:
            self.get_logger().warn("No Image data available. Not sending anything to the LLM")
            result.success = False
            result.message = "Failed to get image."
            goal_handle.abort()  # Remove 'result' argument
            return result

        capture_image(self, b64_image)  # Call the helper function

        try:
            # Send the prompt and base64 image to the LLM using zeromq
            request_data = {
                'user_command': prompt  # Using the actual prompt from the request
            }
            
            self.get_logger().info(f"Sending request to LLM via ZMQ...")
            self.zmq_socket.send_json(request_data)
            
            # Wait for response with timeout (120 seconds)
            if self.zmq_socket.poll(timeout=120000):
                response = self.zmq_socket.recv_json()
                
                if response.get('status') == 'success':
                    response_result = response.get('result', {})
                    llm_response = response_result.get('responses', [])  # Get the list directly, default to empty list
                    print(llm_response)
                    # self.get_logger().info(f"Received successful response from LLM:{llm_response}")
                else:
                    self.get_logger().error(f"LLM returned error: {response.get('error', 'Unknown error')}")
                    raise Exception(f"LLM error: {response.get('error', 'Unknown error')}")
            else:
                self.get_logger().error("Timeout waiting for LLM response")
                raise Exception("Timeout waiting for LLM response")

        except Exception as e:
            self.get_logger().error(f"Error calling LLM: {e}")
            result.success = False
            result.message = "Error calling LLM."
            goal_handle.abort()  # Remove 'result' argument
            return result
        
        print(llm_response)
        # Remove the extremely long sleep that blocks execution
        # time.sleep(10000)

        if llm_response:
            self.get_logger().info("LLM calling successful")
            feedback_msg.status = "LLM processing successful."
            goal_handle.publish_feedback(feedback_msg)

            # Process the raw LLM response
            motor_command, units, commands_or_data, speech = process_llm_result(self, llm_response)
            
            # Check if we have a sequence of commands to execute
            if motor_command is None and isinstance(commands_or_data, list):
                # Sequential execution of commands
                command_count = len(commands_or_data)
                self.get_logger().info(f"Executing sequence of {command_count} commands")

                time.sleep(1)  
                
                for i, command_data in enumerate(commands_or_data):
                    self.get_logger().info(f"Executing command {i+1}/{command_count}")
                    time.sleep(5)
                    
                    # Process each command individually
                    cmd_str = command_data.get("command", "")
                    robot_speech = command_data.get("description", "")
                    
                    # Handle speech for each command if present
                    if robot_speech:
                        msg = String()
                        msg.data = robot_speech
                        self.speech_publisher.publish(msg)
                        self.get_logger().info(f"Published speech: {robot_speech}")
                    
                    # Skip further processing for TALK command
                    if cmd_str == "TALK":
                        self.get_logger().info("Executed TALK command, continuing to next command")
                        continue
                    
                    # Map LLM command to motor command and get distance/rotation
                    motor_cmd = None
                    distance_in_m = 0.0
                    rotation_value = 0.0
                    
                    if "MOVE_FORWARD" in cmd_str:
                        motor_cmd = "FORWARD"
                        distance_in_m = command_data.get("linear_distance", 0.0) / 100.0
                    elif "MOVE_BACKWARD" in cmd_str:
                        motor_cmd = "BACKWARD"
                        distance_in_m = command_data.get("linear_distance", 0.0) / 100.0
                    elif "ROTATE_CLOCKWISE" in cmd_str:
                        motor_cmd = "CLOCKWISE"
                        rotation_value = command_data.get("rotate_degree", 0.0)
                    elif "ROTATE_COUNTERCLOCKWISE" in cmd_str:
                        motor_cmd = "ANTICLOCKWISE"
                        rotation_value = command_data.get("rotate_degree", 0.0)
                    elif "WAIT" in cmd_str:
                        motor_cmd = "WAIT"
                    
                    if not motor_cmd:
                        self.get_logger().warn(f"Unknown command: {cmd_str}, skipping")
                        continue
                    
                    # Set movement parameters and execute
                    with self.pose_lock:
                        self.start_pose = self.current_pose
                        self.target_distance = distance_in_m
                        self.target_rotation = rotation_value
                        self.movement_type = motor_cmd
                    
                    # Send velocity command and wait for completion
                    self.send_velocity_command(motor_cmd)
                    self.is_moving = True
                    
                    # Instead of awaiting, use a synchronous wait
                    self.wait_for_movement_completion(goal_handle)
                    
                    feedback_msg.status = f"Completed command {i+1}/{command_count}"
                    goal_handle.publish_feedback(feedback_msg)
                
                self.get_logger().info("Completed all commands in sequence")
                result.success = True
                result.message = "Sequential LLM Tasks Executed"
                result.llm_response = json.dumps(llm_response)
                goal_handle.succeed()
                return result

                # Then return the result separately            
            # Handle single command case
            self.get_logger().info("LLM calling successful")
            feedback_msg.status = "LLM processing successful."
            goal_handle.publish_feedback(feedback_msg)

            # Process the raw LLM response
            motor_command, units, previous_task_data, speech = process_llm_result(self, llm_response)

            # Handle speech if present
            if speech:
                self.msg = String()
                self.msg.data = speech
                self.speech_publisher.publish(self.msg)
            
            # If there's no motor command, we're done after processing speech
            if not motor_command:
                self.get_logger().info("No movement command to execute")
                self.is_moving = False
                result.success = True
                result.message = "LLM Task Executed."
                result.llm_response = json.dumps(llm_response)
                goal_handle.succeed()
                return result
            elif motor_command == "TALK":
                self.is_moving = False
                result.success = True
                result.message = "LLM Task Executed."
                result.llm_response = json.dumps(llm_response)
                goal_handle.succeed()
                return result

            # Process movement command if present
            distance_in_m = units/100 if units is not None else 0

            with self.pose_lock:
                self.start_pose = self.current_pose
                self.target_distance = distance_in_m
                self.target_rotation = units if motor_command in ["CLOCKWISE", "ANTICLOCKWISE"] else 0
                self.movement_type = motor_command

            # Send velocity command instead of motor command
            self.send_velocity_command(motor_command)
            self.get_logger().info(f"Sent velocity command for: {motor_command}")
            
            self.is_moving = True

            # Remove redundant check
            result.success = True
            result.message = "LLM Task Executed."
            result.llm_response = json.dumps(llm_response)  # Convert to proper JSON string
            goal_handle.succeed()
        else:
            self.get_logger().warn("No response from LLM.")
            result.success = False
            result.message = "No response from LLM."
            goal_handle.abort()  # Remove 'result' argument

        return result

    def wait_for_movement_completion(self, goal_handle):
        """Waits for the current movement to complete before continuing."""
        self.get_logger().info("Waiting for movement to complete...")
        
        # Make sure the timeout is functioning
        poll_rate = 10.0  # Hz
        max_wait_time = 20.0  # seconds, slightly reduced to avoid long waits
        start_time = time.time()
        
        try:
            while self.is_moving and rclpy.ok():
                # Add timeout logic to avoid getting stuck in this loop
                if time.time() - start_time > max_wait_time:
                    self.get_logger().warn(f"Movement timeout after {max_wait_time} seconds - forcing stop")
                    self.stop_movement()
                    break
                    
                # Use spin_once which works with ROS's executor
                rclpy.spin_once(self, timeout_sec=1.0/poll_rate)
        except Exception as e:
            self.get_logger().error(f"Error during movement completion wait: {e}")
            self.stop_movement()
            
        self.get_logger().info("Movement completed or timed out")
    
    def movement_control_callback(self):
        """Timer callback for continuous movement monitoring"""
        if not self.is_moving or self.start_pose is None or self.current_pose is None:
            return
        
        # Handle rotation ramping
        if self.movement_type in ["CLOCKWISE", "ANTICLOCKWISE"] and not self.rotation_ramp_done:
            current_time = self.get_clock().now()
            if self.rotation_start_time is not None:
                # Switch to cruise speed after 0.5 seconds
                time_diff = (current_time - self.rotation_start_time).nanoseconds / 1e9
                if time_diff > 0.5:  # 500ms of initial high-speed rotation
                    self.rotation_ramp_done = True
                    # Resend command to apply the cruise speed
                    self.send_velocity_command(self.movement_type)
                    self.get_logger().info("Switched to cruise rotation speed")
        
        with self.pose_lock:
            if self.start_pose is None:
                self.start_pose = self.current_pose
                return
            try:
                if self.movement_type in ["FORWARD", "BACKWARD"]:
                    traveled_distance = self.calculate_distance(self.start_pose, self.current_pose)
                    self.get_logger().info(f"Traveled distance: {traveled_distance:.2f} meters, Target: {self.target_distance:.2f} meters")

                    safe_distance_threshold = traveled_distance + 0.20  # meters
                    # Remove the safety margin to make it more accurate
                    if safe_distance_threshold >= self.target_distance:
                        self.get_logger().info(f"Target distance {self.target_distance:.2f} meters reached")
                        self.stop_movement()
                
                elif self.movement_type in ["CLOCKWISE", "ANTICLOCKWISE"]:
                    current_angle = self.current_pose.z
                    if self.movement_type == "CLOCKWISE":
                        rotated_angle = abs(self.start_pose.z - current_angle)
                    else:
                        rotated_angle = abs(current_angle - self.start_pose.z)
                    
                    self.get_logger().info(f"Rotated angle: {rotated_angle:.2f} degrees, Target: {self.target_rotation:.2f} degrees")
                    
                    # Add a safety threshold like in the working implementation
                    safe_rotation_threshold = rotated_angle + 20  # degrees
                    if safe_rotation_threshold >= self.target_rotation:
                        self.get_logger().info(f"Target rotation {self.target_rotation:.2f} degrees reached")
                        self.stop_movement()
                        
            except Exception as e:
                self.get_logger().error(f"Error in movement control: {e}")
                self.stop_movement()

    def stop_message(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.0   
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0
        
        self.publish_motor_velocity.publish(vel_msg)
        self.get_logger().info("Published stop command")

    def stop_movement(self):
        """Stop the robot movement"""
        self.get_logger().info("Stopping movement now")
        self.stop_message()  # Send stop command first
        self.is_moving = False
        self.start_pose = None
        self.target_distance = None
        self.target_rotation = None
        self.movement_type = None
        self.rotation_ramp_done = False  # Reset the rotation ramp flag
        self.rotation_start_time = None  # Reset the rotation start time
        self.get_logger().info("Movement stopped")

    def calculate_distance(self, start_pose, current_pose):
        """Calculate the distance traveled along the z-axis"""
        return math.sqrt((current_pose.x - start_pose.x)**2 + (current_pose.y - start_pose.y)**2)

    def destroy_node(self):
        """Destroys the node."""
        self.get_logger().info("Shutting down LLM Image Action Server")
        # Close ZMQ connection
        if hasattr(self, 'zmq_socket'):
            self.zmq_socket.close()
        if hasattr(self, 'zmq_context'):
            self.zmq_context.term()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    llm_image_action_server = LLMImageActionServer()

    try:
        rclpy.spin(llm_image_action_server)
    except KeyboardInterrupt:
        llm_image_action_server.get_logger().info("Keyboard Interrupt, shutting down")
    finally:
        llm_image_action_server.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()