"""
Test script for the LLM Bot ZMQ server.
Demonstrates both text-only and image-based command processing.
"""

import zmq
import json
import base64
import time
from pathlib import Path

class LLMBotClient:
    """
    Test client for LLM Bot ZMQ server.
    
    Attributes:
        port (int): Server port number
        context (zmq.Context): ZMQ context
        socket (zmq.Socket): ZMQ REQ socket
    """
    
    def __init__(self, port: int = 5555):
        """Initialize ZMQ client connection."""
        self.port = port
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect(f"tcp://192.168.0.176:{port}")
        
    def send_command(self, command: str, image_path: str = None) -> dict:
        """
        Send a command to the server with optional image.
        
        Args:
            command (str): Command text to process
            image_path (str, optional): Path to image file
            
        Returns:
            dict: Server response
        """
        # Prepare request data
        request_data = {
            'user_command': command
        }
        
        # Add image if provided
        if image_path:
            try:
                with open(image_path, 'rb') as img_file:
                    img_bytes = img_file.read()
                    img_base64 = base64.b64encode(img_bytes).decode('utf-8')
                    request_data['image'] = img_base64
            except Exception as e:
                print(f"Error loading image: {e}")
                return None
        
        try:
            # Send request
            print(f"\nSending command: {command}")
            self.socket.send_json(request_data)
            print(f"\nSent command: {command}")
            # Wait for response with timeout
            if self.socket.poll(timeout=1200000):  # 30 second timeout
                response = self.socket.recv_json()
                return response
            else:
                print("Error: Server response timeout")
                return None
                
        except Exception as e:
            print(f"Error communicating with server: {e}")
            return None
    
    def close(self):
        """Clean up ZMQ connection."""
        self.socket.close()
        self.context.term()

def run_tests():
    """Run a series of test commands."""
    client = LLMBotClient()
    
    # Test cases
    test_commands = [
        # Multiple commands
        "Move forward 2 meters, turn left 45 degrees, then move back 1 foot",
    ]
    
    try:
        # Run each test command
        for i, command in enumerate(test_commands, 1):
            print(f"\n=== Test {i} ===")
            response = client.send_command(command)
            
            if response:
                print("\nServer Response:")
                if response.get('status') == 'success':
                    result = response.get('result', {})
                    print("Raw Response:", result['raw'])
                else:
                    print("Full Response:", json.dumps(result, indent=2))
            else:
                print("Unexpected result format:", json.dumps(result, indent=2))
        else:
            print(f"Error: {response.get('error', 'Unknown error')}")
            if 'raw_result' in response:
                print(f"Raw Result: {response['raw_result']}")
            
            # Add delay between tests
            time.sleep(2)
        
        
            
    finally:
        client.close()

if __name__ == "__main__":
    run_tests() 