import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from robot_messages.action import LLMTrigger, MotorControl
import json
import asyncio
import websockets
import ssl
from threading import Thread, Event
from geometry_msgs.msg import Vector3
import time

class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')
        self.get_logger().info("Main Controller Node Started")

        # Action Clients
        self.llm_action_client = ActionClient(self, LLMTrigger, 'llm_interaction')
        self.motor_control_client = ActionClient(self, MotorControl, 'motor_control')

        # WebSocket Setup
        self.declare_parameter('websocket_prompt_uri', "wss://192.168.0.214:4000/transcription")
        self.websocket_prompt_uri = self.get_parameter('websocket_prompt_uri').get_parameter_value().string_value

        # Set up SSL context
        self.ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        self.ssl_context.check_hostname = False
        self.ssl_context.verify_mode = ssl.CERT_NONE

        # Create an asyncio event loop for WebSocket communication
        self.loop = asyncio.new_event_loop()
        self.shutdown_event = Event()

        # Start WebSocket listener thread
        self.websocket_thread = Thread(target=self.run_async_loop, daemon=True)
        self.websocket_thread.start()

        self.current_prompt = None
        self.processing_prompt = False
        self.last_processed_prompt = None

        # Subscriber for pose data
        self.subscription = self.create_subscription(
            Vector3,
            '/robot_position',
            self.position_callback,
            2
        )
        
        self.current_position = None
        self.start_position = None
    
        # Create timer with shorter interval
        self.create_timer(1.0, self.timer_callback)  # Changed from 5.0 to 1.0 seconds

    def timer_callback(self):
        """Simple timer callback that checks for new prompts and processes them"""
        # Skip if we're already processing or no new prompt
        if self.processing_prompt or not self.current_prompt:
            return
        
        # Skip if we've already processed this prompt
        if self.current_prompt == self.last_processed_prompt:
            return

        self.get_logger().info("Starting to process new prompt...")
        self.processing_prompt = True
        
        try:
            # Prepare the complete prompt with position information
            robot_position = ""
            if self.current_position and self.start_position:
                start_x = round(self.start_position.x, 2)
                start_y = round(self.start_position.y, 2)
                start_z = round(self.start_position.z, 2)
                robot_position = f"The robots starting position is x:{start_z}, y:{start_x}, z:{start_y}. Your current position is x:{self.current_position.z}, y:{self.current_position.x}, z:{self.current_position.y}"
            
            complete_prompt = self.current_prompt + ". " + robot_position

            # Send to LLM
            self.get_logger().info("Sending prompt to LLM...")
            llm_result = self.send_llm_request(complete_prompt)
            
            if llm_result:
                self.get_logger().info(f"Received LLM response: {llm_result.llm_response}")
                
                # Process result
                motor_command, distance, task_complete, _ = self.process_llm_result(llm_result)
                
                if motor_command and distance is not None:
                    # Send motor command
                    self.get_logger().info(f"Sending motor command: {motor_command}, distance: {distance}")
                    success = self.send_motor_command(motor_command, distance)
                    
                    if success:
                        self.get_logger().info("Motor command executed successfully")
                    else:
                        self.get_logger().error("Motor command failed")
                else:
                    self.get_logger().error("Failed to process LLM result")
            else:
                self.get_logger().error("Failed to get LLM response")

            # Mark this prompt as processed
            self.last_processed_prompt = self.current_prompt
            
        except Exception as e:
            self.get_logger().error(f"Error in main logic: {str(e)}")
        finally:
            self.processing_prompt = False

    def send_llm_request(self, prompt):
        """Send request to LLM server"""
        try:
            # Wait for server
            if not self.llm_action_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().error("LLM server not available")
                return None

            # Create and send goal
            goal_msg = LLMTrigger.Goal()
            goal_msg.prompt = prompt
            
            self.get_logger().info("Sending goal to LLM server...")
            future = self.llm_action_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, future)
            
            if not future.result() or not future.result().accepted:
                self.get_logger().error("LLM goal was rejected")
                return None

            # Get result
            result_future = future.result().get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            
            return result_future.result()

        except Exception as e:
            self.get_logger().error(f"Error in send_llm_request: {str(e)}")
            return None

    def send_motor_command(self, command, distance):
        """Send command to motor control server"""
        try:
            # Wait for server
            if not self.motor_control_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().error("Motor server not available")
                return False

            # Create and send goal
            goal_msg = MotorControl.Goal()
            goal_msg.command = command
            goal_msg.distance = float(distance)
            
            future = self.motor_control_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, future)
            
            if not future.result() or not future.result().accepted:
                self.get_logger().error("Motor goal was rejected")
                return False

            # Get result
            result_future = future.result().get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            
            return result_future.result().success

        except Exception as e:
            self.get_logger().error(f"Error in send_motor_command: {str(e)}")
            return False

    def run_async_loop(self):
        """Run the asyncio event loop for WebSocket"""
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.run_until_complete(self.listen_to_websocket())
        except asyncio.CancelledError:
            self.get_logger().info("WebSocket listener stopped.")
        finally:
            self.loop.close()

    async def listen_to_websocket(self):
        """Listen to WebSocket for prompts"""
        while not self.shutdown_event.is_set():
            try:
                self.get_logger().info(f"Connecting to WebSocket server at {self.websocket_prompt_uri}...")
                async with websockets.connect(self.websocket_prompt_uri, ssl=self.ssl_context) as websocket:
                    self.get_logger().info("Connected to WebSocket server.")
                    async for message in websocket:
                        if self.shutdown_event.is_set():
                            break
                        try:
                            data = json.loads(message)
                            if 'message' in data and 'prompt' in data['message']:
                                self.current_prompt = data['message']['prompt']
                                self.get_logger().info(f"Received new prompt: {self.current_prompt}")
                        except json.JSONDecodeError as e:
                            self.get_logger().error(f"Invalid JSON received: {str(e)}")
                        except Exception as e:
                            self.get_logger().error(f"Error processing prompt data: {str(e)}")
            except websockets.exceptions.ConnectionClosed as e:
                self.get_logger().warning(f"WebSocket connection closed: {str(e)}. Retrying in 5 seconds...")
                await asyncio.sleep(5)
            except Exception as e:
                self.get_logger().error(f"WebSocket error: {str(e)}. Retrying in 5 seconds...")
                await asyncio.sleep(5)

    def position_callback(self, msg):
        """Callback for robot position updates"""
        self.current_position = msg
        if self.start_position is None:
            self.start_position = msg
        self.get_logger().debug(f"Current robot position: x={msg.x}, z={msg.z}, roll={msg.y}")

    def process_llm_result(self, llm_result):
        """Process the LLM response and extract command information"""
        try:
            if not llm_result.llm_response:
                self.get_logger().error("Empty response from LLM server")
                return None, None, None, None

            data = json.loads(llm_result.llm_response)
            if not data or "command" not in data:
                self.get_logger().error("Invalid LLM response format")
                return None, None, None, None

            task_complete = data.get("task_complete", False)
            
            if "MOVE_FORWARD" in data["command"]:
                return "MOVE_FORWARD", data.get("linear_distance", 0.0), task_complete, data
            elif "MOVE_BACKWARD" in data["command"]:
                return "MOVE_FORWARD", data.get("linear_distance", 0.0), task_complete, data
            elif "ROTATE_CLOCKWISE" in data["command"]:
                return "ROTATE", data.get("rotate_degree", 0.0), task_complete, data
            elif "ROTATE_COUNTERCLOCKWISE" in data["command"]:
                return "ROTATE", data.get("rotate_degree", 0.0), task_complete, data
            elif "WAIT" in data["command"]:
                return "WAIT", 0.0, task_complete, data
            else:
                self.get_logger().warn(f"Unknown command: {data['command']}")
                return None, None, None, None

        except Exception as e:
            self.get_logger().error(f"Error processing LLM result: {str(e)}")
            return None, None, None, None

def main(args=None):
    rclpy.init(args=args)
    main_controller = MainController()
    try:
        rclpy.spin(main_controller)
    except KeyboardInterrupt:
        main_controller.get_logger().info("Keyboard interrupt, shutting down")
    finally:
        main_controller.shutdown_event.set()
        main_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()