import rclpy
from rclpy.node import Node
import rclpy.action
import asyncio
import websockets
import json
from example_interfaces.action import Fibonacci # Replace with your actual action definition
from std_msgs.msg import String # Replace if you are not using a publisher

import io
from PIL import Image

# Import functions from robot_control_openai.py
from robot_control_openai import detect_object_with_gpt
import base64
import cv2

class LLMImageActionServer(Node):
    def __init__(self):
        super().__init__('llm_image_action_server')
        self.websocket_uri = "wss://192.168.0.214:8888/video-stream" # Replace with your server address
        self.latest_image_data = None
        self.websocket = None
        self.receiving_task = None
        self.action_server = rclpy.action.ActionServer(
            self,
            Fibonacci, # Replace with your actual action definition
            'llm_interaction',
            self.execute_callback
        )
        self.publisher_ = self.create_publisher(String, 'llm_response', 10) # Replace if you are not using a publisher

        self.start_websocket_connection() # start the websocket connection on init


    def start_websocket_connection(self):
          """Starts the WebSocket connection and begins receiving messages."""
          self.receiving_task = asyncio.ensure_future(self.receive_image_data())

    async def receive_image_data(self):
        """Connects to the WebSocket server and receives image data."""
        try:
            async with websockets.connect(self.websocket_uri, ping_interval=None) as websocket:
                self.websocket = websocket
                self.get_logger().info("Connected to WebSocket server for video stream.")
                while True:
                  try:
                    frame_blob = await self.websocket.recv()
                    if isinstance(frame_blob, bytes):
                      self.latest_image_data = frame_blob
                    else:
                      self.get_logger().warn("Received non-binary data over the websocket")
                  except websockets.exceptions.ConnectionClosed:
                      self.get_logger().warn("Connection closed by server.")
                      break
                  except Exception as e:
                      self.get_logger().error(f"Error receiving message: {e}")
                      break
        except Exception as e:
            self.get_logger().error(f"Error connecting to WebSocket server: {e}")
        finally:
            self.websocket = None
    
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
        self.get_logger().info(f"Executing action with goal {goal_handle.request.order}")

        feedback_msg = Fibonacci.Feedback() # Replace with your actual feedback message
        result = Fibonacci.Result() # Replace with your actual result message
        
        prompt = goal_handle.request.prompt #Replace with how you are getting prompt from goal

        # Capture frame and encode
        b64_image = self.capture_frame()

        # If no image return error
        if b64_image is None:
             self.get_logger().warn("No Image data available. Not sending anything to the LLM")
             goal_handle.aborted()
             return result

        # Trigger LLM
        llm_response = detect_object_with_gpt(b64_image, prompt)

        if llm_response:
              # Publish the LLM response
            msg = String() # Replace if you are not using a publisher
            msg.data = str(llm_response)
            self.publisher_.publish(msg)
            self.get_logger().info("LLM response published.")

            # Send back feedback (if any)
            feedback_msg.sequence = [1,2,3] # Replace with actual feedback logic
            goal_handle.publish_feedback(feedback_msg)

            # Set the result to be the LLM's response. (Needs your actual result msg implementation)
            result.sequence = [1,2,3,4] # Replace with actual result logic
            goal_handle.succeed(result)
        else:
          self.get_logger().warn("No response from LLM.")
          goal_handle.aborted()

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