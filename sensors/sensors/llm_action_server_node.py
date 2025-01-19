import rclpy
from rclpy.node import Node
import rclpy.action
import asyncio
import websockets
import json
from robot_messages.action import LLMTrigger
from std_msgs.msg import String
import io
from PIL import Image
from sensors.robot_control_openai import LLMClient  # Assuming you have this
import base64
import ssl
from threading import Thread

class LLMImageActionServer(Node):
    def __init__(self):
        super().__init__('llm_image_action_server')
        self.get_logger().info("Started LLM Image Action Server")

        self.declare_parameter('websocket_uri', "wss://192.168.0.214:8888/video-stream")
        self.websocket_uri = self.get_parameter('websocket_uri').get_parameter_value().string_value
        self.latest_image_data = None
        self.llm_client = LLMClient()  # Initialize LLMClient

        # Set up SSL context
        self.ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        self.ssl_context.check_hostname = False
        self.ssl_context.verify_mode = ssl.CERT_NONE

        # Start WebSocket listener in a separate thread
        self.loop = asyncio.new_event_loop()
        self.websocket_thread = Thread(target=self.run_async_loop, daemon=True)
        self.websocket_thread.start()

        # Initialize action server
        self.action_server = rclpy.action.ActionServer(
            self,
            LLMTrigger,
            'llm_interaction',
            self.execute_callback
        )

        # Initialize publisher
        self.publisher_ = self.create_publisher(String, 'llm_response', 10)

    def run_async_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.listen_to_websocket())

    async def listen_to_websocket(self):
        while True:
            try:
                self.get_logger().info(f"Attempting to connect to WebSocket server at {self.websocket_uri}...")
                async with websockets.connect(self.websocket_uri, ssl=self.ssl_context, ping_interval=None) as websocket:
                    self.get_logger().info("Connected to WebSocket server successfully.")
                    while True:
                        try:
                            frame_blob = await websocket.recv()
                            if isinstance(frame_blob, bytes):
                                self.latest_image_data = frame_blob
                                self.get_logger().debug(f"Frame received, size: {len(frame_blob)} bytes")
                            else:
                                self.get_logger().warning("Received non-binary data over the websocket")
                        except websockets.exceptions.ConnectionClosed:
                            self.get_logger().warning("Connection closed by server. Reconnecting...")
                            break
                        except Exception as e:
                            self.get_logger().error(f"Error receiving message: {e}")
                            break
            except Exception as e:
                self.get_logger().error(f"Error connecting to WebSocket server: {e}")
                await asyncio.sleep(5)

    def capture_frame(self):
        """
        Captures the latest frame, rotates it and returns the base64-encoded image string for GPT processing.
        """
        if self.latest_image_data is None:
            self.get_logger().warn("No frame available yet.")
            return None
        try:
            # open image using PIL
            image = Image.open(io.BytesIO(self.latest_image_data))
            #convert to RGB
            image = image.convert("RGB")
            rotated_frame = image.rotate(90, expand=True)

            buffered = io.BytesIO()
            #save as jpeg
            rotated_frame.save(buffered, format="JPEG")
            b64_image = base64.b64encode(buffered.getvalue()).decode("utf-8")

            return b64_image
        except Exception as e:
            self.get_logger().error(f"Error capturing frame {e}")
            return None

    async def execute_callback(self, goal_handle):
        """Executes the action when goal is received."""
        self.get_logger().info(f"Executing action with goal: {goal_handle.request.prompt}")

        feedback_msg = LLMTrigger.Feedback()
        result = LLMTrigger.Result()
        
        prompt = goal_handle.request.prompt

        # Capture frame and encode
        b64_image = self.capture_frame()

        # If no image return error
        if b64_image is None:
            self.get_logger().warn("No Image data available. Not sending anything to the LLM")
            result.success = False
            result.message = "Failed to capture image."
            goal_handle.abort(result=result)
            return result

        try:
            # Use rclpy.get_global_executor() to run in a thread from ROS 2's thread pool:
            llm_response = self.llm_client.detect_object_with_gpt(b64_image, prompt)
            self.get_logger().info(f"llm_response:{llm_response}")
        except Exception as e:
            self.get_logger().error(f"Error calling LLM: {e}")
            result.success = False
            result.message = "Error calling LLM."
            goal_handle.abort(result=result)
            return result

        if llm_response:
            # Send back feedback (if any)
            self.get_logger().info("LLM calling successful")
            feedback_msg.status = "LLM processing successfull."
            goal_handle.publish_feedback(feedback_msg)

            # Set the result to be the LLM's response. (Needs your actual result msg implementation)
            result.success = True
            result.message = "LLM processing successfull."
            result.llm_response = json.dumps(llm_response)  # Convert to proper JSON string
            goal_handle.succeed()
        else:
            self.get_logger().warn("No response from LLM.")
            result.success = False
            result.message = "No response from LLM."
            goal_handle.abort(result=result)

        return result
    def destroy_node(self):
      """Cancels the receiving task and destroys the node."""
      if self.receiving_task:
        self.receiving_task.cancel()
        self.get_logger().info("Cancelling Receiving task.")
      if self.websocket and self.websocket.open:
           asyncio.get_event_loop().run_until_complete(self.websocket.close())
           self.get_logger().info("Closing websocket")
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