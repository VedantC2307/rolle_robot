import json
import base64
from PIL import Image
import os
import io
import time

def process_llm_result(node, llm_result):
    """
    Processes the JSON result from the LLM server.
    Args:
        node: The ROS node (for logging)
        llm_result: The raw dictionary from LLM response
    Returns:
        If it's a single command format:
            motor_command, distance, data, speech
        If it's a multi-command format:
            None, None, commands_array, None
    """
    try:
        print("Started processing LLM result")
        # llm_result is already a dictionary
        data = llm_result
        if not data:
            node.get_logger().error("Empty response from LLM server")
            return None, None, None, None

        # Check if this is the new multi-command format
        if "commands" in data and isinstance(data["commands"], list):
            node.get_logger().info(f"Found {len(data['commands'])} commands to execute sequentially")
            return None, None, data["commands"], None

        # Original single-command format processing
        command_str = data.get("command")
        robot_speech = data.get("description", " ")

        # Define mapping of command substrings to (motor_command, distance_key)
        command_map = {
            "MOVE_FORWARD": ("FORWARD", "linear_distance"),
            "MOVE_BACKWARD": ("BACKWARD", "linear_distance"),
            "ROTATE_CLOCKWISE": ("CLOCKWISE", "rotate_degree"),
            "ROTATE_COUNTERCLOCKWISE": ("ANTICLOCKWISE", "rotate_degree"),
            "WAIT": ("WAIT", None),
            "TALK": ("TALK", None),
        }

        for key, (motor_command, distance_key) in command_map.items():
            if key in command_str:
                distance = data.get(distance_key, 0.0) if distance_key else 0.0
                node.get_logger().info(f"Extracted command: {motor_command}, distance: {distance}")
                return motor_command, distance, data, robot_speech

        node.get_logger().warn(f"Unknown action from LLM: {command_str}")
        return None, None, data, robot_speech

    except (AttributeError, KeyError) as e:
        node.get_logger().error(f"Error processing LLM result: {e}")
        return None, None, None, None
        
def capture_image(node, base64_string):
    """
    Saves a base64 encoded image to disk.
    Args:
        node: The ROS node (for logging)
        base64_string: Base64 encoded image data
    """
    try:
        # Create a directory called "saved_images" in the current working directory if it doesn't exist
        save_dir = os.path.join(os.getcwd(), "saved_images")
        os.makedirs(save_dir, exist_ok=True)

        # Decode base64 string to bytes
        image_data = base64.b64decode(base64_string)
        
        # Create PIL Image from bytes
        image = Image.open(io.BytesIO(image_data))
        
        # Generate a filename based on the current timestamp
        filename = f"image_{int(time.time())}.jpeg"
        file_path = os.path.join(save_dir, filename)
        print(file_path)
        
        # Save the image
        image.save(file_path, format="JPEG")
        node.get_logger().info(f"Image saved to {file_path}")
        
    except base64.binascii.Error as e:
        node.get_logger().error(f"Invalid base64 string: {e}")
    except Exception as e:
        node.get_logger().error(f"Error saving image to file: {e}")