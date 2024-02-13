import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['xterm', '-fn', 'xft:fixed:size=12', '-geometry', '60x20', '-e', 'ros2', 'run', 'call_m_teleoperation', 'keyboard_control_node'],
            output='screen',
        ),
        launch.actions.ExecuteProcess(
            cmd=['xterm', '-fn', 'xft:fixed:size=12', '-geometry', '60x35','-e', 'ros2', 'run', 'call_m_teleoperation', 'joystick_control_node'],
            output='screen',
        ),
    ])