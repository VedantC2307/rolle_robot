import json
import os
from openai import OpenAI
from dotenv import load_dotenv
import base64
import io
from PIL import Image

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
           "You are a versatile assistant mounted on a mobile robot. "
            "Your primary purpose is to decode user instructions, analyze the environment, and execute appropriate actions to accomplish tasks. "
            "Provide the response in JSON format, following the instructions in the user prompt.\n\n"
            "Output Format:\n"
            "Always respond with a JSON object containing the following fields:\n"
            "- 'command': The action to perform (e.g., 'MOVE_FORWARD', 'ROTATE', 'STOP').\n"
            "- 'forward_distance': The calculated distance to move forward (if applicable).\n"
            "- 'rotate_degree': The rotation angle (if applicable).\n"
            "- 'object': The object to find (if applicable).\n"
            "- 'in_scene': Boolean indicating whether the object is visible in the scene.\n"
            "- 'description': A concise summary of the current environment and the next steps.\n\n"
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