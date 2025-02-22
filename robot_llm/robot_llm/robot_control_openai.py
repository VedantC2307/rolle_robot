import json
import os
from openai import OpenAI
from dotenv import load_dotenv

class LLMClient:
    def __init__(self, api_key=None, model="gpt-4o-mini", system_prompt = None):
        # Load environment variables from .env file if .env file exists
        env_path = os.path.join(os.path.dirname(__file__), ".env")
        if os.path.exists(env_path):
           load_dotenv(dotenv_path=env_path)
        
        self.api_key = api_key if api_key else os.getenv("OPENAI_API_KEY") # Get the API key from environment or pass an argument
        if not self.api_key:
            raise ValueError("API key is missing or not set in environment variable OPENAI_API_KEY")
        self.client = OpenAI(api_key = self.api_key)
        self.model = model
        self.system_prompt = system_prompt if system_prompt else (
                        "Role:"
                        "You are a computer vision-powered robotic assistant specializing in object detection and obstacle avoidance. You can also talk friendly to the user and joke around."
                        "You are mounted static on a 4-wheel mobile robot. Your primary purpose is to decode what the user is saying and convert them into motor commands."
                        "Capabilities:"
                        "Movement:"
                        "You can navigate the environment using the following movement commands:"
                        '"MOVE_FORWARD" and "MOVE_BACKWARD" (requires a "linear_distance" parameter in centimeters).'
                        '"ROTATE_CLOCKWISE" and "ROTATE_COUNTERCLOCKWISE" (requires a "rotate_degree" parameter in degrees).'
                        "Your motion control should ensure obstacle avoidance and precise positioning."
                        
                        "Communication:"
                        "You can talk to the user using the 'description' parameter to provide status updates to the user if needed."
                        "Use the 'description' command to have normal conversations with the user if asked a question." 
                        "Communicate clearly when actions are completed or if assistance is needed."
                        
                        "Response Format:"
                        "Always provide output strictly in JSON format with the following keys only:"

                        "{"
                        '  "command": "MOVE_FORWARD" or "MOVE_BACKWARD" or "ROTATE_CLOCKWISE" or "ROTATE_COUNTERCLOCKWISE" '
                        '  "linear_distance": <value in cm> (if moving forward or backward),'
                        '  "rotate_degree": <value in degrees> (if rotating clockwise or anticlockwise),'
                        '  "description": "<message/talk to the user>"'
                        "}"


                    )


    def detect_object_with_gpt(self, b64_img, prompt):
        """
        Uses GPT-4-turbo to analyze an image and determine if the object is in the scene.
        """
        try:
             # Open Image and get the type of image
            img_type = f"image/jpeg"

            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {
                        "role": "system",
                        "content": self.system_prompt
                    },
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": f"{prompt}"},
                            {"type": "image_url",
                             "image_url": {
                                 "url": f"data:{img_type};base64,{b64_img}"
                             }}
                        ]
                    }
                ],
                response_format={"type": "json_object"}
            )


            response_content = response.choices[0].message.content
            return json.loads(response_content)
        except json.JSONDecodeError as json_err:
            print(f"Error during JSON decoding: {json_err}")
            return None
        except Exception as e:
            print(f"Error during GPT object detection: {e}")
            return None