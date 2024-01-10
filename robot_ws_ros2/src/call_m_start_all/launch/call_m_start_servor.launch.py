import launch
from launch import LaunchDescription

def generate_launch_description():

    # Command to launch xterm and execute ROS 2 launch commands
    #cmd = ['xterm', '-fn', 'xft:fixed:size=12', '-geometry', '60x20', '-e', 'ros2', 'launch']
    cmd = ['ros2', 'launch']

    # Construct the absolute paths to your launch files
    master_launch =  ['call_m_supervisor', 'master.launch.py','use_sim_time:=false']
    bot_launch = ['call_m_hardware', 'bot.launch.py']

    return LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=cmd + master_launch,
            output='screen',
        ),
        launch.actions.ExecuteProcess(
            cmd=cmd + bot_launch,
            output='screen',
        ),
    ])