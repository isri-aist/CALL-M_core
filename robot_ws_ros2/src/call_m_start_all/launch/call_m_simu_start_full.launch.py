import launch
from launch import LaunchDescription
import launch_ros

def generate_launch_description():

    # Command to launch xterm and execute ROS 2 launch commands
    #cmd = ['xterm', '-fn', 'xft:fixed:size=12', '-geometry', '60x20', '-e', 'ros2', 'launch']
    cmd = ['ros2', 'launch']

    # Construct the absolute paths to your launch files
    master_launch =  ['call_m_supervisor', 'master.launch.py']
    simu_launch = ['call_m_simulation', 'simulation.launch.py']
    display_launch = ['call_m_monitor', 'display.launch.py']
    teleop_launch =  ['call_m_teleoperation', 'teleop.launch.py']

    return LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=cmd + master_launch,
            output='screen',
        ),
        launch.actions.ExecuteProcess(
            cmd=cmd + simu_launch,
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
