import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['xterm', '-fn', 'xft:fixed:size=12', '-geometry', '60x20', '-e', 'ros2', 'run', 'c_pkg', 'keyboard_control_node'],
            output='screen',
        ),
        launch.actions.ExecuteProcess(
            cmd=['xterm', '-fn', 'xft:fixed:size=12', '-geometry', '100x20','-e', 'ros2', 'run', 'c_pkg', 'bot_control_driver_node'],
            output='screen',
        ),
        launch.actions.ExecuteProcess(
            cmd=['xterm', '-fn', 'xft:fixed:size=12', '-geometry', '60x30','-e', 'ros2', 'run', 'c_pkg', 'joystick_control_node'],
            output='screen',
        ),
    ])