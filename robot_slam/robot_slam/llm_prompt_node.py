#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import zmq
import signal
import sys
import json

class LLMPromptNode(Node):
    def __init__(self):
        super().__init__('llm_prompt_node')
        
        # Initialize ZMQ subscriber
        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.SUB)
        self.zmq_socket.connect('tcp://localhost:5555')
        self.zmq_socket.subscribe('transcription')
        
        # Create ROS publisher
        self.publisher = self.create_publisher(String, '/mobile_sensor/speech', 10)
        
        # Create timer for checking ZMQ messages (30Hz)
        self.create_timer(0.033, self.receive_zmq_data)
        
        self.get_logger().info('LLM Prompt Node started')

    def receive_zmq_data(self):
        """Receives ZMQ data containing transcription messages"""
        try:
            topic = self.zmq_socket.recv_string(flags=zmq.NOBLOCK)
            message = self.zmq_socket.recv_string(flags=zmq.NOBLOCK)
            
            try:
                # Try to parse as JSON first
                data = json.loads(message)
                if isinstance(data, dict) and 'prompt' in data:
                    prompt = data['prompt']
                else:
                    prompt = message
            except json.JSONDecodeError:
                # If not JSON, use raw message
                prompt = message
            
            # Create and publish ROS message
            msg = String()
            msg.data = prompt
            self.publisher.publish(msg)
            self.get_logger().info(f'Published prompt: {prompt}')
            
        except zmq.Again:
            # No message available
            pass
        except Exception as e:
            self.get_logger().error(f'Error receiving message: {str(e)}')

    def __del__(self):
        """Cleanup method to ensure proper shutdown"""
        if hasattr(self, 'zmq_socket'):
            self.zmq_socket.close()
        if hasattr(self, 'zmq_context'):
            self.zmq_context.term()

def main(args=None):
    rclpy.init(args=args)
    node = LLMPromptNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down LLM Prompt Node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
