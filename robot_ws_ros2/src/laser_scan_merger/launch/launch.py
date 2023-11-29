import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('laser_scan_merger'),
        'config',
        'params.yaml'
        )
        
    node=Node(
        package = 'laser_scan_merger',
        name = 'laser_scan_merger_node',
        executable = 'laser_scan_merger_node',
        parameters = [config]
    )

    ld.add_action(node)
    return ld