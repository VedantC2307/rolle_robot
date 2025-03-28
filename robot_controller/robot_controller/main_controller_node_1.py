import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from robot_messages.action import LLMTrigger
from std_msgs.msg import String

class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')
        self.get_logger().info("Main Controller Node Started")

        # Action Clients
        self.llm_action_client = ActionClient(self, LLMTrigger, 'llm_interaction')

        # Subscribe to audio prompt topic
        self.prompt_subscription = self.create_subscription(
            String,
            '/mobile_sensor/audio',
            self.prompt_callback,
            10
        )

        self.current_prompt = None
        self.processing_prompt = False
        self.last_processed_prompt = None

        # Create timer for prompt processing
        self.create_timer(1.0, self.process_prompt)

    def prompt_callback(self, msg):
        """Callback for receiving audio prompts"""
        self.current_prompt = msg.data
        self.get_logger().info(f"Received new prompt: {self.current_prompt}")

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

    def process_prompt(self):
        """Process new prompts and send to LLM"""
        try:
            if not self.current_prompt or self.processing_prompt:
                return
            
            if self.current_prompt != self.last_processed_prompt:
                self.processing_prompt = True
                current_prompt = self.current_prompt

                self.get_logger().info(f'Processing new prompt: {current_prompt}')
                # Create and run the coroutine
                future = rclpy.Future()
                rclpy.Task(self.call_llm_action_server(current_prompt), future)
                
                if future.result():
                    llm_response = future.result()
                    if llm_response and llm_response.success:
                        self.get_logger().info(f"LLM Response: {llm_response.llm_response}")
                        self.last_processed_prompt = current_prompt
                    else:
                        self.get_logger().error("Failed to get LLM response")
                        self.last_processed_prompt = None

        except Exception as e:
            self.get_logger().error(f"Error in process_prompt: {str(e)}")
            self.last_processed_prompt = None
        finally:
            self.processing_prompt = False

    def destroy_node(self):
        """Cleanup method"""
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    main_controller = MainController()
    try:
        rclpy.spin(main_controller)
    except KeyboardInterrupt:
        main_controller.get_logger().info("Keyboard interrupt, shutting down")
    finally:
        main_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()