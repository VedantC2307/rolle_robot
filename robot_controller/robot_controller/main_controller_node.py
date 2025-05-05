import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from robot_messages.action import LLMTrigger
from std_msgs.msg import String
import json
from threading import Event
from robot_controller import config

class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')
        self.get_logger().info("Main Controller Node Started")

        # Action Clients
        self.llm_action_client = ActionClient(self, LLMTrigger, 'llm_interaction')

        # Subscribe to prompt topic
        self.prompt_subscription = self.create_subscription(
            String,
            '/mobile_sensor/speech',
            self.prompt_callback,
            10
        )

        self.current_prompt = None
        self.processing_prompt = False
        self.last_processed_prompt = None

        # Create timer for prompt processing
        self.create_timer(1.0, self.timer_callback)
        
        self.shutdown_event = Event()

    def prompt_callback(self, msg):
        """Callback for receiving prompts from the topic"""
        self.current_prompt = msg.data
        self.get_logger().debug(f"Received new prompt: {self.current_prompt}")

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

    def timer_callback(self):
        """Process new prompts and send to LLM"""
        if not self.current_prompt:
            return
            
        # Only process if we have a new prompt and not currently processing
        if not self.processing_prompt and self.current_prompt != self.last_processed_prompt:
            self.processing_prompt = True
            current_prompt = self.current_prompt  # Store current prompt to process
            
            self.get_logger().info(f'Processing new prompt: {current_prompt}')
            
            # Create and send goal
            goal_msg = LLMTrigger.Goal()
            goal_msg.prompt = current_prompt
            
            self.get_logger().info('Waiting for LLM server...')
            self.llm_action_client.wait_for_server()
            
            self.get_logger().info('Sending goal to LLM server...')
            send_goal_future = self.llm_action_client.send_goal_async(goal_msg)
            send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Goal rejected')
            self.processing_prompt = False
            # Reset the last processed prompt to enable retrying
            self.last_processed_prompt = None 
            return

        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        try:
            result = future.result().result
            if result.success:
                self.get_logger().info(f"LLM Response: {result.llm_response}")
                self.last_processed_prompt = self.current_prompt
            else:
                self.get_logger().error("Failed to get LLM response")
                self.last_processed_prompt = None
        except Exception as e:
            self.get_logger().error(f"Error processing result: {e}")
            self.last_processed_prompt = None
        
        # Always reset processing flag to allow next command
        self.processing_prompt = False
        self.get_logger().info("Ready for next command")

    def destroy_node(self):
        """Cleanup method"""
        self.shutdown_event.set()
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