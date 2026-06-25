import os
from ament_index_python.packages import get_package_share_directory 

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Locate the share directory where CMake installed the config 
    package_dir = get_package_share_directory('controller')
    config_file = os.path.join(package_dir, 'config', 'mpc_params.yaml')

    # Define the controller node execution mapping 
    controller_node = Node(
        package = 'controller',
        executable = 'dynamics_test',
        name = 'go2_mpc_controller',
        parameters = [config_file],
        output = 'screen', 
        emulate_tty = True
    )

    return LaunchDescription([
        controller_node
    ])

