import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    LAUNCHES
    """
    #Command to launch xterm and execute ROS 2 launch commands
    #cmd = ['xterm', '-fn', 'xft:fixed:size=12', '-geometry', '60x20', '-e', 'ros2', 'launch']
    cmd = ['ros2', 'launch']
    #sim_time used for syncing managed from bot_NUC.launch.py

    # Construct the absolute paths to the launch files
    hardware_launch = launch.actions.ExecuteProcess(cmd=cmd + ['call_m_hardware', 'bot_NUC.launch.py'], output='screen')
    
    return LaunchDescription([hardware_launch])
