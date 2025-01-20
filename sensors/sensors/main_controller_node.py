import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from robot_messages.action import LLMTrigger, MotorControl
import json
import asyncio
import websockets
import ssl
from threading import Thread, Event

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

        # Start the process using ROS 2 executor
        self.create_timer(5.0, self.main_logic)

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
                            self.get_logger().info(f"Raw message received: {message}")
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

    def main_logic(self):
        """
        Main logic of the controller triggered periodically:
        1. Send prompt to LLM action server.
        2. Process LLM result.
        3. Send command to motor control action server.
        """
        if self.current_prompt is None:
            self.get_logger().debug("No prompt received. Giving priority to other processes.")
            return

        self.get_logger().info("Processing received prompt...")
        rclpy.spin_until_future_complete(self, self.send_goal_to_llm_server(self.current_prompt))
        self.current_prompt = None  # Reset after processing

    async def send_goal_to_llm_server(self, prompt):
        """Sends the prompt to the LLM action server."""
        goal_msg = LLMTrigger.Goal()
        goal_msg.prompt = prompt

        self.get_logger().info("Waiting for LLM action server...")
        if not self.llm_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("LLM action server not available within timeout.")
            return None

        self.get_logger().info("LLM action server available. Sending goal.")
        send_goal_future = self.llm_action_client.send_goal_async(goal_msg)
        goal_handle = await send_goal_future

        if not goal_handle.accepted:
            self.get_logger().warning('LLM goal rejected')
            return None

        result = await self.get_result_from_llm_server(goal_handle)
        if result:
            motor_command, distance = self.process_llm_result(result)
            if motor_command:
                await self.send_goal_to_motor_server(motor_command, distance)
        return goal_handle

    async def get_result_from_llm_server(self, goal_handle):
        """Gets the result from the LLM action server."""
        result_future = goal_handle.get_result_async()
        result = await result_future

        if not result:
            self.get_logger().error('LLM action failed')
            return None

        self.get_logger().info(f'LLM result: {result.result.llm_response}')
        return result.result

    async def send_goal_to_motor_server(self, command, distance):
        """Sends the command and distance to the motor control action server."""
        goal_msg = MotorControl.Goal()
        goal_msg.command = command
        goal_msg.distance = float(distance)

        self.get_logger().info("Waiting for Motor action server...")
        if not self.motor_control_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Motor action server not available within timeout.")
            return None

        self.get_logger().info("Motor action server available. Sending goal.")
        send_goal_future = self.motor_control_client.send_goal_async(goal_msg)
        goal_handle = await send_goal_future

        if not goal_handle.accepted:
            self.get_logger().warning('Motor control goal rejected')
            return None

        return await self.get_result_from_motor_server(goal_handle)

    async def get_result_from_motor_server(self, goal_handle):
        """Gets the result from the motor control action server."""
        result_future = goal_handle.get_result_async()
        result = await result_future

        if not result:
            self.get_logger().error('Motor control action failed')
            return None

        if result.result.success:
            self.get_logger().info('Motor control action succeeded')
        else:
            self.get_logger().error('Motor control action failed')
        return result.result

    def process_llm_result(self, llm_result):
        """Processes the JSON result from the LLM server and extracts motor command and distance."""
        try:
            data = json.loads(llm_result.llm_response)
            if "MOVE_FORWARD" in data.get("command", ""):
                motor_command = "MOVE_FORWARD"
                distance = float(data.get("forward_distance", 0.5))  # Default 0.5 meters
                self.get_logger().info(f"Extracted command: {motor_command}, distance: {distance}")
                return motor_command, distance
            else:
                self.get_logger().warning("Unknown action from LLM.")
                return None, None
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self.get_logger().error(f"Error processing LLM result: {e}")
            return None, None

    def destroy_node(self):
        self.get_logger().info("Shutting down Main Controller...")
        self.shutdown_event.set()
        self.websocket_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    main_controller = MainController()
    try:
        rclpy.spin(main_controller)
    except KeyboardInterrupt:
        main_controller.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        main_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
