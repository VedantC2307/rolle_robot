import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # Launch llm_action_server node
    llm_action_server_node = Node(
        package='robot_llm',
        executable='llm_action_server',
        name='LLM_action_server',
        output='screen',
        emulate_tty=True
    )

    motor_control_node = Node(
        package='motor_controller',
        executable='motor_control_node',
        name='Motor_Control_Node',
        output='screen',
        emulate_tty=True
    )

    # Launch slam_node
    slam_node = Node(
        package='robot_slam',
        executable='slam_node',
        name='slam_node',
        output='screen',
        emulate_tty=True
    )

    # Base64 Camera Node
    camera_node = Node(
        package='robot_slam',
        executable='base64_camera_data_node',
        name='camera_data_node',
        output='screen',
        emulate_tty=True
    )

    # Robot Speech Node
    speech_node = Node(
        package='robot_slam',
        executable='robot_speech_node',
        name='robot_speech_node',
        output='screen',
        emulate_tty=True
    )
    return LaunchDescription([
        slam_node,
        camera_node,
        speech_node,
        motor_control_node,
        llm_action_server_node,
    ])