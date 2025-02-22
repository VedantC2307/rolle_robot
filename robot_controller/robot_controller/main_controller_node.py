import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from robot_messages.action import LLMTrigger, MotorControl
import json
import asyncio
import websockets
import ssl
from threading import Thread, Event
from robot_controller import config
import time

class MainController(Node):
    def __init__(self):

        super().__init__('main_controller')
        self.get_logger().info("Main Controller Node Started")

        # Action Clients
        self.llm_action_client = ActionClient(self, LLMTrigger, 'llm_interaction')
        self.motor_control_client = ActionClient(self, MotorControl, 'motor_control')

        # Use the IP address from the environment or default
        ip_address = config.IP_ADDRESS

        # Setup websocket to receive speech
        websocket_prompt_uri = f"wss://{ip_address}:4000/transcription"
        self.declare_parameter('websocket_prompt_uri', websocket_prompt_uri)
        self.websocket_prompt_uri = self.get_parameter('websocket_prompt_uri').get_parameter_value().string_value

        # Setup websocket to send speech
        self.websocket_talking_url = f"wss://{ip_address}:8888/tts"

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

        self.speech_websocket_thread = Thread(target=self.run_speech_loop, daemon=True)
        self.speech_websocket_thread.start()

        self.current_prompt = None
        self.processing_prompt = False
        self.last_processed_prompt = None   

        # In __init__, need to add:
        self.speech_queue = asyncio.Queue()

        self.current_position = None
        self.start_position = None
        # self.main_logic_ready = True
    
        # Create the timer without making it async
        self.create_timer(5.0, self.timer_callback)

    def run_speech_loop(self):
        speech_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(speech_loop)
        try:
            speech_loop.run_until_complete(self.send_to_websocket())
        except asyncio.CancelledError:
            self.get_logger().info("Speech WebSocket stopped.")

    def timer_callback(self):
        """Non-async timer callback that creates and runs the coroutine"""
        if self.loop.is_running():
            asyncio.run_coroutine_threadsafe(self.main_logic(), self.loop)

    def run_async_loop(self):
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.run_until_complete(self.listen_to_websocket())
        except asyncio.CancelledError:
            self.get_logger().info("WebSocket listener stopped.")
        finally:
            self.loop.close()

    async def listen_to_websocket(self):
        """
        Connect to the WebSocket server and continuously listen for prompt updates.
        """
        while not self.shutdown_event.is_set():
            try:
                self.get_logger().info(f"Connecting to WebSocket server at {self.websocket_prompt_uri}...")
                async with websockets.connect(self.websocket_prompt_uri, ssl=self.ssl_context) as websocket:
                    self.get_logger().info("Connected to WebSocket server.")
                    async for message in websocket:
                        if self.shutdown_event.is_set():
                            break
                        try:
                            # self.get_logger().debug(f"Raw message received: {message}")
                            data = json.loads(message)
                            if 'message' in data and 'prompt' in data['message']:
                                self.current_prompt = data['message']['prompt']
                                self.get_logger().info(f"Received new prompt: {self.current_prompt}")
                            else:
                                self.get_logger().warning("Received message without prompt data.")
                        except json.JSONDecodeError:
                            self.get_logger().error(f"Invalid JSON received: {message}")
                        except Exception as e:
                            self.get_logger().error(f"Error processing prompt data: {str(e)}")
            except websockets.exceptions.ConnectionClosed as e:
                self.get_logger().warning(f"WebSocket connection closed: {str(e)}. Retrying in 5 seconds...")
                await asyncio.sleep(5)
            except Exception as e:
                self.get_logger().error(f"WebSocket error: {str(e)}. Retrying in 5 seconds...")
                await asyncio.sleep(5)
            

    async def call_llm_action_server(self, prompt):
        """Sends the prompt to the LLM action server."""
        try:
            # Ensure prompt is a string and not None
            if not isinstance(prompt, str) or not prompt.strip():
                self.get_logger().error("Invalid prompt received. Prompt must be a non-empty string.")
                return None

            goal_msg = LLMTrigger.Goal()
            goal_msg.prompt = prompt
            self.get_logger().info(f"Sending prompt to LLM action server: {prompt}")

            # Wait for server without await
            if not self.llm_action_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error("LLM action server not available within timeout.")
                return None
            self.get_logger().info("LLM action server available.")

            # Send goal asynchronously
            send_goal_future = await self.llm_action_client.send_goal_async(goal_msg)
            
            if not send_goal_future.accepted:
                self.get_logger().warning('LLM goal rejected')
                return None
            
            result_future = await send_goal_future.get_result_async()
            
            if not result_future:
                self.get_logger().error('LLM action failed')
                return None
            
            self.get_logger().info('LLM goal accepted')
            return result_future.result

        except Exception as e:
            self.get_logger().error(f"Error sending goal to LLM server: {str(e)}")
            return None

    async def motor_controller_action(self, command, distance):
        """Sends the command and distance to the motor control action server."""
        try:
            goal_msg = MotorControl.Goal()
            goal_msg.command = command
            goal_msg.distance = float(distance)

            if not self.motor_control_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error("Motor action server not available within timeout.")
                return None
            self.get_logger().info("Motor action server available.")

            send_goal_future = await self.motor_control_client.send_goal_async(goal_msg)
            
            if not send_goal_future.accepted:
                self.get_logger().warn('Motor control goal rejected')
                return None

            self.get_logger().info('Motor control goal accepted')

            result_future = await send_goal_future.get_result_async()
            
            if not result_future:
                self.get_logger().error('Motor control action failed')
                return None

            return result_future.result
            
        except Exception as e:
            self.get_logger().error(f"Error sending goal to motor server: {str(e)}")
            return None

    def process_llm_result(self, llm_result):
        """
        Processes the JSON result from the LLM server and extracts the motor command, distance,
        the raw data, and a robot speech string (if provided).
        """
        try:
            response = llm_result.llm_response
            if not response:
                self.get_logger().error("Empty response from LLM server")
                return None, None, None, None

            data = json.loads(response)
            if not data:
                self.get_logger().error("Empty JSON data from LLM response")
                return None, None, None, None

            command_str = data.get("command", "")
            if not command_str:
                self.get_logger().error("No command found in LLM response")
                return None, None, None, None

            # Set a default value for robot speech if not provided
            robot_speech = data.get("description", " ")

            # Define mapping of command substrings to (motor_command, distance_key)
            command_map = {
                "MOVE_FORWARD": ("MOVE_FORWARD", "linear_distance"),
                "MOVE_BACKWARD": ("MOVE_BACKWARD", "linear_distance"),
                "ROTATE_CLOCKWISE": ("ROTATE_CLOCKWISE", "rotate_degree"),
                "ROTATE_COUNTERCLOCKWISE": ("ROTATE_COUNTERCLOCKWISE", "rotate_degree"),
                "WAIT": ("WAIT", None),
            }

            for key, (motor_command, distance_key) in command_map.items():
                if key in command_str:
                    distance = data.get(distance_key, 0.0) if distance_key else 0.0
                    self.get_logger().info(f"Extracted command: {motor_command}, distance: {distance}")
                    return motor_command, distance, data, robot_speech

            self.get_logger().warn(f"Unknown action from LLM: {command_str}")
            return None, None, data, robot_speech

        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f"Error processing LLM result: {e}")
            return None, None, None, None

        
    async def send_to_websocket(self):
        """
        Connect to the WebSocket server and send speech messages.
        """
        while not self.shutdown_event.is_set():
            try:
                self.get_logger().info(f"Connecting to Speech WebSocket server at {self.websocket_talking_url}...")
                async with websockets.connect(self.websocket_talking_url, ssl=self.ssl_context) as websocket:
                    self.get_logger().info("Connected to Speech WebSocket server.")
                    while not self.shutdown_event.is_set():
                        try:
                            # Wait for a message in the queue
                            speech_data = await self.speech_queue.get()
                            
                            # Send the speech data
                            await websocket.send(json.dumps(speech_data))
                            self.get_logger().info(f"Sent speech data: {speech_data}")
                            
                        except websockets.exceptions.ConnectionClosed:
                            self.get_logger().warning("Speech WebSocket connection closed. Reconnecting...")
                            break
                        except Exception as e:
                            self.get_logger().error(f"Error sending speech data: {str(e)}")
                            await asyncio.sleep(1)
                            
            except Exception as e:
                self.get_logger().error(f"Speech WebSocket connection error: {str(e)}. Retrying in 5 seconds...")
                await asyncio.sleep(5)

    def process_speech(self, speech_text):
        """
        Process the speech text and prepare it for sending through WebSocket.
        Returns a formatted dictionary for TTS processing.
        """
        try:
            if not speech_text:
                return None
                
            speech_data = {
                "text": speech_text,
            }
            
            # Add the speech data to the queue
            if self.loop.is_running():
                asyncio.run_coroutine_threadsafe(
                    self.speech_queue.put(speech_data), 
                    self.loop
                )

            return speech_data
            
        except Exception as e:
            self.get_logger().error(f"Error processing speech: {str(e)}")
            return None

    
    async def main_logic(self):
        """
        Main logic of the controller:
        1. Send prompt to LLM action server.
        2. Process LLM result.
        3. Send command to motor control action server.
        """
        try:
            # self.main_logic_ready = False
            # Check if we have a valid prompt
            if self.processing_prompt or not self.current_prompt:
                # self.get_logger().debug("No prompt available, skipping main logic loop")
                return
            
            if self.current_prompt == self.last_processed_prompt:
                return
            
            self.processing_prompt = True
            self.last_processed_prompt = self.current_prompt

            prompt = self.current_prompt

            llm_response = await self.call_llm_action_server(prompt)

            if not llm_response.success:
                self.get_logger().info("Error printing response")
                
            self.get_logger().info(f"LLM Response: {llm_response.llm_response}")

            
            # 2. Process LLM result
            motor_command, distance, previous_task_data, speech = self.process_llm_result(llm_response)
            
            if speech:
                speech_data = self.process_speech(speech)
                if speech_data:
                    self.get_logger().info(f"Processed speech data: {speech_data}")

            print("motor_command:", motor_command)
            print("distance:", distance)
            print("Speech:", speech)

            time.sleep(10000)

            if not motor_command:
                self.get_logger().error("Could not determine motor command from LLM result")
                self.processing_prompt = False
                return

            # Send Goal to motor_controller action server
            motor_goal_handle = await self.motor_controller_action(motor_command, distance)

            # motor_result = await self.get_result_from_motor_server(motor_goal_handle)
            if not motor_goal_handle:
                self.get_logger().error("Failed to get result from motor control server")
            elif motor_goal_handle.success:
                self.get_logger().info("Motor control action succeeded")
            else:
                self.get_logger().info("Motor control action failed")

        except Exception as e:
            self.get_logger().error(f"Error in main logic: {str(e)}")
        finally:
            self.processing_prompt = False

    def __del__(self):
        """Cleanup method to ensure proper shutdown"""
        self.shutdown_event.set()
        if hasattr(self, 'prompt_websocket_thread'):
            self.listen_to_websocket.join(timeout=1)
        if hasattr(self, 'speech_websocket_thread'):
            self.send_to_websocket_.join(timeout=1)


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