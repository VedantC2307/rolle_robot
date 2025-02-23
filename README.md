# ROS 2 Mobile Robot Control System

This project implements a ROS 2-based control system for a mobile robot, featuring natural language control, visual scene understanding, and SLAM-based localization.

## Overview

The robot is controlled through voice commands.  A separate JavaScript component handles speech-to-text, sending the transcribed text to the ROS 2 system.  An LLM (Large Language Model, specifically OpenAI's GPT-4 Vision) processes the commands and, optionally, analyzes images from the robot's camera. The LLM's output, formatted as JSON, specifies motor commands (forward/backward movement, rotation) and associated parameters (distance, angle).  The robot uses SLAM to determine its position and orientation.  A JavaScript component receives text-to-speech data from the ROS 2 system and plays audio through speakers.

## System Architecture

The system is composed of the following ROS 2 packages:

*   **`motor_controller`:** Directly controls the robot's motors based on commands received from the `robot_controller`.  Uses the RPi.GPIO library for low-level motor control.
*   **`robot_controller`:** The central control hub.  Receives text prompts, interacts with the LLM, and sends commands to the `motor_controller`.  Manages WebSocket connections for speech and SLAM data.
*   **`robot_llm`:** Handles interaction with the LLM for image analysis and prompt processing.  Receives image frames via WebSockets.
*   **`robot_slam`:** Receives SLAM data from an external system via WebSockets and publishes the robot's pose as a ROS 2 topic.
* **`robot_messages`**: (Not Present in File Structure but Required) This package would contain custom ROS 2 messages and action definitions.

**Data Flow:**

1.  **User Input:** The user speaks a command.
2.  **Speech-to-Text:** The JavaScript component converts the speech to text.
3.  **Prompt Processing:** The `MainController` (in `robot_controller`) receives the text prompt via a WebSocket.
4.  **LLM Interaction:** The `MainController` sends the prompt (and optionally, a camera frame) to the `LLMImageActionServer` (in `robot_llm`).
5.  **Image Analysis (Optional):** The `LLMImageActionServer` captures a camera frame from a WebSocket stream, rotates it, and sends it to the LLM (using the `LLMClient`).
6.  **LLM Response:** The LLM returns a JSON response containing a motor command, distance/angle, and/or a descriptive response for speech.
7.  **Motor Control:** The `MainController` parses the LLM response and sends the corresponding motor command and parameters to the `MotorControlNode` (in `motor_controller`).
8.  **Motor Actuation:** The `MotorControlNode` controls the robot's motors using the RPi.GPIO library.
9.  **Localization:** The `WebSocketListenerNode` (in `robot_slam`) continuously receives SLAM data via a WebSocket, extracts the pose (x, y, roll), and publishes it as a ROS 2 topic (`/pose_data`).
10. **Text-to-Speech:** The `MainController` sends text to the JavaScript component via a WebSocket for speech synthesis.
11. **Robot Feedback:** The `MotorControlNode` subscribes to the `/pose_data` topic and provides feedback during movement.
12. **Obstacle Detection:** *Potentially*, an ultrasonic sensor could be used for obstacle detection (code is present but commented out).  Other methods might be employed.

## Dependencies

*   ROS 2 (Specify your distribution, e.g., Humble, Foxy)
*   Python 3.x
*   `openai` (Python library)
*   `websockets` (Python library)
*   `Pillow` (PIL Fork - Python Imaging Library)
*   `RPi.GPIO` (Raspberry Pi GPIO library - only needed on the robot itself)
*   **[Add any other dependencies here]**
*   **JavaScript Dependencies (for the external component):** [You'll need to specify these, e.g., a specific WebSocket library, any libraries for interacting with the SLAM system, etc.]

## Hardware

*   **Robot Platform:** [Describe your robot platform - e.g., a custom 4-wheeled robot]
*   **Microcontroller:** Raspberry Pi [Specify Model, e.g., 4 Model B]
*   **Motor Driver:** [Specify your motor driver, e.g., L298N, TB6612FNG]
*   **Motors:** [Specify your motors, e.g., DC geared motors with encoders]
*   **Camera:** [Specify your camera, e.g., Raspberry Pi Camera Module V2]
*   **Ultrasonic Sensor (Optional):** [Specify if used, e.g., HC-SR04]
*   **Power Supply:** [Specify your power supply, e.g., 7.4V LiPo battery]
* **Computer:** [Specify computer details, e.g., x64 Processor]

## Setup and Installation

1.  **ROS 2 Installation:**  Follow the instructions for your ROS 2 distribution to install ROS 2.

2.  **Clone the Repository:**
    ```bash
    git clone [Your Repository URL]
    cd [Your Repository Name]
    ```

3.  **Create `robot_messages`:**
      Create a package named `robot_messages`. Inside `robot_messages`, define the following:
    *  **Action Definitions (`action/`):**
        *   **`MotorControl.action`:**
            ```action
            # Goal
            string command  # "MOVE_FORWARD", "MOVE_BACKWARD", "ROTATE_CLOCKWISE", "ROTATE_COUNTERCLOCKWISE"
            float32 distance  # meters for movement, degrees for rotation
            float32 rotation_degrees
            ---
            # Result
            bool success
            ---
            # Feedback
            string status
            ```
        *   **`LLMTrigger.action`:**
            ```action
            # Goal
            string prompt
            ---
            # Result
            bool success
            string message
            string llm_response  # JSON string
            ---
            # Feedback
            string status
            ```
4.  **Install Dependencies:**
    ```bash
    cd [Your Repository Name]
    rosdep install -i --from-path src --rosdistro <your_ros_distro> -y
    pip3 install -r requirements.txt
    ```
    Create `requirements.txt` file in the root directory of your project and put the following inside it.

     ```
     openai
     websockets
     Pillow
     RPi.GPIO
     ```

5.  **Build the Workspace:**
    ```bash
    colcon build --symlink-install
    ```

6.  **Source the Workspace:**
    ```bash
    . install/setup.bash
    ```

7.  **Configure `config.py`:**
    *   Create a `config.py` file in the `robot_controller` package.
    *   Add the following line, replacing `YOUR_LOCAL_IP` with your robot's local IP address:
        ```python
        IP_ADDRESS = "YOUR_LOCAL_IP"
        ```

8.  **Set up the JavaScript Component:**
    *   [Provide instructions on how to set up and run the JavaScript component. This will likely involve installing Node.js and any necessary JavaScript libraries.] Include instructions for setting the websocket IP and Port.

## Running the System

1.  **Start the JavaScript Component:** [Provide instructions, e.g., `node your_script.js`]

2.  **Launch the ROS 2 Nodes:** You can run this by creating a launch file or simply running the nodes using `ros2 run`. Create a launch file by creating a new file `launch_file.py` inside a `launch` directory in the `robot_launch` package, and put the code below inside it.
