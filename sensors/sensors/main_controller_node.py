import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from robot_messages.action import LLMTrigger, MotorControl
import json
import asyncio

class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')
        self.get_logger().info("Main Controller Node Started")

        # Action Clients
        # self.llm_action_client = ActionClient(self, LLMTrigger, 'llm_interaction')
        self.motor_control_client = ActionClient(self, MotorControl, 'motor_control')

        # Prompt (Initially hardcoded)
        self.prompt = "Move forward until you see a water bottle"  # Example prompt

        # Start the process using ROS 2 executor
        self.executor = rclpy.get_global_executor()
        self.executor.create_task(self.main_logic())

    async def main_logic(self):
        """
        Main logic of the controller:
        1. Send prompt to LLM action server.
        2. Process LLM result.
        3. Send command to motor control action server.
        """
        # 1. Send prompt to LLM action server
        # llm_goal_handle = await self.send_goal_to_llm_server(self.prompt)
        # if not llm_goal_handle:
        #     self.get_logger().error("LLM goal was rejected")
        #     return

        # llm_result = await self.get_result_from_llm_server(llm_goal_handle)
        # if not llm_result:
        #     self.get_logger().error("Failed to get result from LLM server")
        #     return

        # 2. Process LLM result
        # motor_command, distance = self.process_llm_result(llm_result)
        # if not motor_command:
        #     self.get_logger().error("Could not determine motor command from LLM result")
        #     return

        motor_command = "MOVE_FORWARD"
        distance = 0.5

        # 3. Send command to motor control action server
        motor_goal_handle = await self.send_goal_to_motor_server(motor_command, distance)
        if not motor_goal_handle:
            self.get_logger().error("Motor control goal was rejected")
            return

        motor_result = await self.get_result_from_motor_server(motor_goal_handle)
        if not motor_result:
            self.get_logger().error("Failed to get result from motor control server")
        elif motor_result.success:
            self.get_logger().info("Motor control action succeeded")
        else:
            self.get_logger().info("Motor control action failed")

    # async def send_goal_to_llm_server(self, prompt):
    #     """Sends the prompt to the LLM action server."""
    #     goal_msg = LLMTrigger.Goal()
    #     goal_msg.prompt = prompt
    #     print(self.prompt)

    #     self.get_logger().info("Waiting for LLM action server...")
    #     if not self.llm_action_client.wait_for_server(timeout_sec=10.0):
    #         self.get_logger().error("LLM action server not available within timeout.")
    #         return None
    #     self.get_logger().info("LLM action server available.")


    #     send_goal_future = self.llm_action_client.send_goal_async(goal_msg)
    #     goal_handle = await send_goal_future

    #     if not goal_handle.accepted:
    #         self.get_logger().warn('LLM goal rejected')
    #         return None

    #     self.get_logger().info('LLM goal accepted')
    #     return goal_handle

    # async def get_result_from_llm_server(self, goal_handle):
    #     """Gets the result from the LLM action server."""
    #     result_future = goal_handle.get_result_async()
    #     result = await result_future

    #     if not result:
    #         self.get_logger().error('LLM action failed')
    #         return None

    #     self.get_logger().info(f'LLM result: {result.result.llm_response}')
    #     return result.result

    async def send_goal_to_motor_server(self, command, distance):
        """Sends the command and distance to the motor control action server."""
        goal_msg = MotorControl.Goal()
        goal_msg.command = command
        goal_msg.distance = float(distance)  # Assuming distance is a float

        # await self.motor_control_client.wait_for_server()

        self.get_logger().info("Waiting for Motor action server...")
        if not self.motor_control_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Motor action server not available within timeout.")
            return None
        self.get_logger().info("Motor action server available.")


        send_goal_future = self.motor_control_client.send_goal_async(goal_msg)
        goal_handle = await send_goal_future

        if not goal_handle.accepted:
            self.get_logger().warn('Motor control goal rejected')
            return None

        self.get_logger().info('Motor control goal accepted')
        return goal_handle

    async def get_result_from_motor_server(self, goal_handle):
        """Gets the result from the motor control action server."""
        result_future = goal_handle.get_result_async()
        result = await result_future

        if not result:
            self.get_logger().error('Motor control action failed')
            return None

        self.get_logger().info(f'Motor control result: {result.result.success}')
        return result.result

    # def process_llm_result(self, llm_result):
    #     """
    #     Processes the JSON result from the LLM server and extracts the motor command and distance.

    #     Args:
    #         llm_result: The result from the LLM action server.

    #     Returns:
    #         A tuple (motor_command, distance), or (None, None) if the result could not be processed.
    #     """
    #     try:
    #         data = json.loads(llm_result.llm_response)
    #         # Example logic (adapt based on your actual LLM output):
    #         if "MOVE_FORWARD" in data["command"]:
    #             motor_command = "MOVE_FORWARD"
    #             distance = data.get("forward_distance", 0.0)  # Default distance of 0.5 meters
    #             self.get_logger().info(f"Extracted command: {motor_command}, distance: {distance}")
    #             return motor_command, distance
    #         else:
    #             self.get_logger().warn(f"Unknown action from LLM: {data['action']}")
    #             return None, None
    #     except (json.JSONDecodeError, KeyError) as e:
    #         self.get_logger().error(f"Error processing LLM result: {e}")
    #         return None, None

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
