import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the share directory for the 'sensors' and 'launcher' package
    sensors_pkg_share = get_package_share_directory('sensors')
    robot_launch_pkg_share = get_package_share_directory('launcher')

    # Define the launch description
    ld = LaunchDescription()

    # Include the launch file from the 'sensors' package
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_launch_pkg_share, 'launch', 'sensors.launch.py')
        )
    )

    # Launch main_controller node
    main_controller_node = Node(
        package='sensors',
        executable='main_controller',
        name='main_controller',
        output='screen',
        emulate_tty=True  # Optional: For colored output
    )

    # Add actions to the launch description
    ld.add_action(sensors_launch)
    ld.add_action(main_controller_node)

    return ld