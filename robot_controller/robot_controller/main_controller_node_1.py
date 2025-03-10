import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from robot_messages.action import LLMTrigger
import json
import asyncio
import websockets
import ssl
from threading import Thread, Event
from robot_controller import config

class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')
        self.get_logger().info("Main Controller Node Started")

        # Action Clients
        self.llm_action_client = ActionClient(self, LLMTrigger, 'llm_interaction')

        # Setup websocket to receive prompt
        ip_address = config.IP_ADDRESS
        websocket_prompt_uri = f"wss://{ip_address}:4000/transcription"
        self.declare_parameter('websocket_prompt_uri', websocket_prompt_uri)
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

        # Create timer for prompt processing
        self.create_timer(1.0, self.timer_callback)

    def run_async_loop(self):
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.run_until_complete(self.listen_to_websocket())
        except asyncio.CancelledError:
            self.get_logger().info("WebSocket listener stopped.")
        finally:
            self.loop.close()

    async def listen_to_websocket(self):
        """Connect to the WebSocket server and continuously listen for prompt updates."""
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
                            # Updated JSON parsing to match the correct format
                            if data['type'] == 'transcription' and 'data' in data and 'prompt' in data['data']:
                                self.current_prompt = data['data']['prompt']
                                self.get_logger().info(f"Received new prompt: {self.current_prompt}")
                            else:
                                self.get_logger().warning(f"Unexpected message format: {data}")
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

    def timer_callback(self):
        """Non-async timer callback that creates and runs the coroutine"""
        if self.loop.is_running():
            asyncio.run_coroutine_threadsafe(self.main_logic(), self.loop)

    async def call_llm_action_server(self, prompt):
        """Sends the prompt to the LLM action server."""
        try:
            if not isinstance(prompt, str) or not prompt.strip():
                self.get_logger().error("Invalid prompt received")
                return None

            goal_msg = LLMTrigger.Goal()
            goal_msg.prompt = prompt
            self.get_logger().info(f"Sending prompt to LLM action server")

            if not self.llm_action_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error("LLM action server not available")
                return None

            send_goal_future = await self.llm_action_client.send_goal_async(goal_msg)
            
            if not send_goal_future.accepted:
                self.get_logger().warning('LLM goal rejected')
                return None
            
            result_future = await send_goal_future.get_result_async()
            return result_future.result if result_future else None

        except Exception as e:
            self.get_logger().error(f"Error sending goal to LLM server: {str(e)}")
            return None

    async def main_logic(self):
        """Process new prompts and send to LLM"""
        try:
            if not self.current_prompt:
                return
            
            # Log current state for debugging
            self.get_logger().debug(f"Processing state - Current: {self.current_prompt}, Last: {self.last_processed_prompt}, Processing: {self.processing_prompt}")
            
            # Only process if we have a new prompt and not currently processing
            if not self.processing_prompt and self.current_prompt != self.last_processed_prompt:
                self.processing_prompt = True
                current_prompt = self.current_prompt  # Store current prompt to process
                
                self.get_logger().info(f'Processing new prompt: {current_prompt}')
                llm_response = await self.call_llm_action_server(prompt=current_prompt)

                if llm_response and llm_response.success:
                    self.get_logger().info(f"LLM Response: {llm_response.llm_response}")
                    # Only update last processed prompt if successful
                    self.last_processed_prompt = current_prompt
                else:
                    self.get_logger().error("Failed to get LLM response")
                    # Reset processing state to allow retry
                    self.last_processed_prompt = None

        except Exception as e:
            self.get_logger().error(f"Error in main logic: {str(e)}")
            self.last_processed_prompt = None  # Reset on error to allow retry
        finally:
            # Always reset processing flag
            self.processing_prompt = False
            self.get_logger().debug("Main logic cycle completed")

    def destroy_node(self):
        """Cleanup method"""
        self.shutdown_event.set()
        if hasattr(self, 'loop') and self.loop.is_running():
            self.loop.stop()
        if hasattr(self, 'websocket_thread'):
            self.websocket_thread.join(timeout=1)
        super().destroy_node()

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