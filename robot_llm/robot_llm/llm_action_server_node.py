import rclpy
from rclpy.node import Node
import rclpy.action
import json
from robot_messages.action import LLMTrigger
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from robot_llm.robot_control_openai import LLMClient
from robot_controller import config
from threading import Lock
import math
from robot_llm.helper_functions import process_llm_result, capture_image

class LLMImageActionServer(Node):
    def __init__(self):
        super().__init__('llm_image_action_server')
        self.get_logger().info("Started LLM Image Action Server")
     
        self.latest_image_data = None
        self.pose_lock = Lock()
        
        # Constants
        # self.SAFE_DISTANCE_THRESHOLD = 0.20  # meters
        self.MOVEMENT_CHECK_RATE = 0.1  # seconds
        

        self.llm_client = LLMClient()  # Initialize LLMClient
        # Initialize action server
        self.action_server = rclpy.action.ActionServer(
            self,
            LLMTrigger,
            'llm_interaction',
            self.execute_callback
        )

        self.camera_subscription = self.create_subscription(
            String,
            '/base64_image',
            self.camera_callback,
            5
        )

        self.pose_subscription = self.create_subscription(
            Vector3,
            '/pose_data',
            self.pose_callback,
            10
        )

        # Create a timer for movement monitoring
        self.movement_timer = self.create_timer(
            self.MOVEMENT_CHECK_RATE,
            self.movement_control_callback
        )

        self.speech_publisher = self.create_publisher(String, '/robot_speech', 2)

        self.publish_motor_command = self.create_publisher(String, '/motor_command', 10)
        
        self.current_pose = None
        self.is_moving = False
        self.start_pose = None
        self.target_distance = None
        self.target_rotation = None
        self.movement_type = None

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

    async def execute_callback(self, goal_handle):
        """Executes the action when goal is received."""
        self.get_logger().info(f"Triggering LLM with Goal...")

        feedback_msg = LLMTrigger.Feedback()
        result = LLMTrigger.Result()
        
        prompt = goal_handle.request.prompt
        
        with self.pose_lock:
            b64_image = self.latest_image_data
        
        if b64_image is None:
            self.get_logger().warn("No Image data available. Not sending anything to the LLM")
            result.success = False
            result.message = "Failed to get image."
            goal_handle.abort(result=result)
            return result

        capture_image(self, b64_image)  # Call the helper function

        try:
            llm_response = self.llm_client.detect_object_with_gpt(b64_image, prompt)
        except Exception as e:
            self.get_logger().error(f"Error calling LLM: {e}")
            result.success = False
            result.message = "Error calling LLM."
            goal_handle.abort(result=result)
            return result
        
        print(llm_response)

        if llm_response:
            self.get_logger().info("LLM calling successful")
            feedback_msg.status = "LLM processing successful."
            goal_handle.publish_feedback(feedback_msg)

            # Process the raw LLM response
            motor_command, units, previous_task_data, speech = process_llm_result(self, llm_response)

            print(speech)

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
                goal_handle.succeed()  # Remove result argument
                return result
            elif motor_command == "TALK":
                self.is_moving = False
                result.success = True
                result.message = "LLM Task Executed."
                result.llm_response = json.dumps(llm_response)
                goal_handle.succeed()  # Remove result argument
                return result

            # Process movement command if present
            distance_in_m = units/100 if units is not None else 0

            with self.pose_lock:
                self.start_pose = self.current_pose
                self.target_distance = distance_in_m
                self.target_rotation = units if motor_command in ["CLOCKWISE", "ANTICLOCKWISE"] else 0
                self.movement_type = motor_command

            self.msg = String()
            self.msg.data = motor_command
            self.publish_motor_command.publish(self.msg)
            self.get_logger().info(f"Published motor command: {motor_command}")
            
            self.is_moving = True

            if not self.is_moving:
                # Set the result to be the LLM's response. (Needs your actual result msg implementation)
                result.success = True
                result.message = "LLM Task Executed."
                result.llm_response = json.dumps(llm_response)  # Convert to proper JSON string
                goal_handle.succeed()
        else:
            self.get_logger().warn("No response from LLM.")
            result.success = False
            result.message = "No response from LLM."
            goal_handle.abort(result=result)

        return result
    
    def movement_control_callback(self):
        """Timer callback for continuous movement monitoring"""
        if not self.is_moving or self.start_pose is None or self.current_pose is None:
            return
        
        with self.pose_lock:
            if self.start_pose is None:
                self.start_pose = self.current_pose
                return
            try:
                if self.movement_type in ["FORWARD", "BACKWARD"]:
                    traveled_distance = self.calculate_distance(self.start_pose, self.current_pose)
                    self.get_logger().debug(f"Traveled distance: {traveled_distance:.2f} meters")

                    safe_travel_distance = traveled_distance + 0.3
                    
                    if safe_travel_distance >= self.target_distance:
                        self.get_logger().info(f"Target distance {self.target_distance:.2f} meters reached")
                        self.stop_movement()
                
                elif self.movement_type in ["CLOCKWISE", "ANTICLOCKWISE"]:
                    current_angle = self.current_pose.z
                    if self.movement_type == "CLOCKWISE":
                        rotated_angle = abs(self.start_pose.z - current_angle)
                    else:
                        rotated_angle = abs(current_angle - self.start_pose.z)
                    
                    self.get_logger().debug(f"Rotated angle: {rotated_angle:.2f} degrees")
                    
                    if rotated_angle >= self.target_rotation:
                        self.get_logger().info(f"Target rotation {self.target_rotation:.2f} degrees reached")
                        self.stop_movement()
                        
            except Exception as e:
                self.get_logger().error(f"Error in movement control: {e}")
                self.stop_movement()

    def stop_movement(self):
        """Stop the robot movement"""
        stop_msg = String()
        stop_msg.data = "STOP"
        self.publish_motor_command.publish(stop_msg)
        self.is_moving = False
        self.start_pose = None
        self.target_distance = None
        self.target_rotation = None
        self.movement_type = None
        self.get_logger().info("Movement stopped")

    def calculate_distance(self, start_pose, current_pose):
        """Calculate the distance traveled along the z-axis"""
        return math.sqrt((current_pose.x - start_pose.x)**2 + (current_pose.y - start_pose.y)**2)

    def destroy_node(self):
        """Destroys the node."""
        self.get_logger().info("Shutting down LLM Image Action Server")
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