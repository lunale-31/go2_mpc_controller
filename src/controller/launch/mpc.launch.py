import os
from ament_index_python.packages import get_package_share_directory 

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Locate the share directory where CMake installed the config 
    package_dir = get_package_share_directory('controller')
    config_file = os.path.join(package_dir, 'config', 'mpc_params.yaml')

    # Define the controller node execution mapping 
    state_estimator_node = Node(
        package = 'controller',
        executable = 'state_estimator_node',
        name = 'state_estimator',
        parameters = [config_file],
        output = 'screen', 
        emulate_tty = True
    )

    high_level_controller = Node(
        package = 'controller',
        executable = 'controller_node',
        name = 'high_level_control',
        parameters = [config_file],
        output = 'screen', 
        emulate_tty = True
    )
    return LaunchDescription([
        state_estimator_node,
        high_level_controller,
    ])

