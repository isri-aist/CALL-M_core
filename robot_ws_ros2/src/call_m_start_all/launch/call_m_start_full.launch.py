import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Command to launch xterm and execute ROS 2 launch commands
    #cmd = ['xterm', '-fn', 'xft:fixed:size=12', '-geometry', '60x20', '-e', 'ros2', 'launch']
    cmd = ['ros2', 'launch']

    # Construct the absolute paths to your launch files
    master_launch =  ['call_m_supervisor', 'master.launch.py','use_sim_time:=false']
    bot_launch = ['call_m_hardware', 'bot.launch.py']
    display_launch = ['call_m_monitor', 'display.launch.py']
    teleop_launch =  ['call_m_teleoperation', 'teleop.launch.py']
    nav2_launch = ['call_m_nav2', 'navigation_launch.py', 'use_sim_time:=false']

    """launch.actions.ExecuteProcess(
        cmd=cmd + nav2_launch,
        output='screen',
    ),"""

    return LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=cmd + master_launch,
            output='screen',
        ),
        launch.actions.ExecuteProcess(
            cmd=cmd + bot_launch,
            output='screen',
        ),
        launch.actions.ExecuteProcess(
            cmd=cmd + display_launch,
            output='screen',
        ),
        launch.actions.ExecuteProcess(
            cmd=cmd + teleop_launch,
            output='screen',
        ),
    ])