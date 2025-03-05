import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the share directory for the launcher package
    robot_launch_pkg_share = get_package_share_directory('robot_launch')

    # Include the sensors launch file
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_launch_pkg_share, 'launch', 'sensors.launch.py')
        )
    )

    # Define nodes
    joystick_control_node = Node(
        package='joy_control',
        executable='joystick_control_node',
        name='joystick_control_node',
        output='screen',
        emulate_tty=True 
    )

    robot_vr_controller_node = Node(
        package='robot_vr_controller',
        executable='vr_control_node',
        name='vr_control_node',
        output='screen',
        emulate_tty=True 
    )

    main_controller_node = Node(
        package='robot_controller',
        executable='main_controller',
        name='main_controller',
        output='screen',
        emulate_tty=True 
    )

    # Get control mode from user
    control_mode = input("Enter control mode (vr/vlm): ")

    # Create launch description
    ld = LaunchDescription([sensors_launch])

    # Add nodes based on control mode
    if control_mode.lower() == 'vr':
        ld.add_action(robot_vr_controller_node)
        ld.add_action(joystick_control_node)
    elif control_mode.lower() == 'vlm':
        ld.add_action(main_controller_node)
    else:
        print(f"Invalid control mode: {control_mode}. Using default VR mode.")
        ld.add_action(robot_vr_controller_node)
        ld.add_action(joystick_control_node)

    return ld