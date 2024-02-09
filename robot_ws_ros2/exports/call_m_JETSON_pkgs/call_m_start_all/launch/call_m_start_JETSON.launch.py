import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    DIRECTORIES
    """
    dir_slam = get_package_share_directory('call_m_start_all')

    """
    PARAMETERS
    """
    nav_type = "none" #'none', 'on_fly' or 'localize'
    slam_param = os.path.join(dir_slam, 'config/mapper_params_online_async.yaml') #path to params for slam

    """
    LAUNCHES
    """
    #Command to launch xterm and execute ROS 2 launch commands
    #cmd = ['xterm', '-fn', 'xft:fixed:size=12', '-geometry', '60x20', '-e', 'ros2', 'launch']
    cmd = ['ros2', 'launch']
    suffix = ['use_sim_time:=false']

    # Construct the absolute paths to the launch files
    master_launch =  launch.actions.ExecuteProcess(cmd=cmd + ['call_m_supervisor', 'master.launch.py'] + suffix, output='screen')
    slam_launch = launch.actions.ExecuteProcess(cmd=cmd + ['slam_toolbox', 'online_async_launch.py', 'params_file:='+slam_param]+ suffix, output='screen')
    hardware_launch = launch.actions.ExecuteProcess(cmd=cmd + ['call_m_hardware', 'bot_JETSON.launch.py'], output='screen')

    if nav_type == "on_fly":
        return LaunchDescription([master_launch,slam_launch,hardware_launch])
    elif nav_type == "localize":
        return LaunchDescription([master_launch,hardware_launch])
    else:
        return LaunchDescription([master_launch,slam_launch,hardware_launch])