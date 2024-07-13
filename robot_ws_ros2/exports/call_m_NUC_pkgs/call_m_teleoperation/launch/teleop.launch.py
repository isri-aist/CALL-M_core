import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('call_m_teleoperation')
    joystick_param_file = os.path.join(pkg_dir, 'config', 'joystick_configuration.yaml')

    return LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['xterm', '-fn', 'xft:fixed:size=12', '-geometry', '60x20', '-e', 'ros2', 'run', 'call_m_teleoperation', 'keyboard_control_node'],
            output='screen',
        ),
        launch.actions.ExecuteProcess(
            cmd=['xterm', '-fn', 'xft:fixed:size=12', '-geometry', '60x40','-e', 'ros2', 'run', 'call_m_teleoperation', 'joystick_control_node','--ros-args','--params-file',joystick_param_file],
            output='screen',
        ),
    ])