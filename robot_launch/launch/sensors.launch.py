import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # Launch llm_action_server node
    llm_action_server_node = Node(
        package='sensors',
        executable='LLM_action_server',
        name='LLM_action_server',
        output='screen',
        emulate_tty=True
    )

    # Launch motor_control_action_server node
    motor_control_node = Node(
        package='sensors',
        executable='motor_control_action_server',
        name='motor_control_action_server',
        output='screen',
        emulate_tty=True
    )

    # Launch ultrasonic_sensor node
    ultrasonic_sensor_node = Node(
        package='sensors',
        executable='ultrasonic_sensor',
        name='ultrasonic_sensor_node',
        output='screen',
        emulate_tty=True
    )

    # Launch slam_node
    slam_node = Node(
        package='sensors',
        executable='slam_node',
        name='slam_node',
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        slam_node,
        ultrasonic_sensor_node,
        llm_action_server_node,
        motor_control_node
    ])