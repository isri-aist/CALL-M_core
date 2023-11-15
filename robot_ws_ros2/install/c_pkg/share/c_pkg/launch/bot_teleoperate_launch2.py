import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='c_pkg',
            executable='bot_control_driver_node',
            name='bot_control_driver_node',
            output='screen',
            emulate_tty=True,
            prefix=['sudo'],
        ),
    ])
