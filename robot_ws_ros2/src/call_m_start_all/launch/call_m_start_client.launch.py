import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    DIRECTORIES
    """
    dir_nav2 = get_package_share_directory('call_m_nav2')
    dir_slam = get_package_share_directory('call_m_start_all')

    """
    PARAMETERS
    """
    nav_type = "none" #'none', 'on_fly' or 'localize'

    nav_mode = os.path.join(dir_nav2, 'config', 'nav2_params_diff.yaml') #'diff' or 'omni', path to config file
    map_loc = "/home/jrluser/test_odom.yaml"#os.path.join(dir_nav2, 'maps', 'ikeuchi_gate_3f.yaml') #path to the map if using localization
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
    display_launch = launch.actions.ExecuteProcess(cmd=cmd + ['call_m_monitor', 'display.launch.py'], output='screen')
    teleop_launch =  launch.actions.ExecuteProcess(cmd=cmd + ['call_m_teleoperation', 'teleop.launch.py'], output='screen')
    nav2_launch = launch.actions.ExecuteProcess(cmd=cmd + ['call_m_nav2', 'navigation_launch.py','params_file:='+nav_mode] + suffix, output='screen')
    nav2_launch_loc = launch.actions.ExecuteProcess(cmd=cmd + ['call_m_nav2', 'localization_launch.py','map:='+map_loc,'params_file:='+nav_mode] + suffix, output='screen')

    if nav_type == "on_fly":
        return LaunchDescription([master_launch,slam_launch,display_launch,teleop_launch,nav2_launch])
    elif nav_type == "localize":
        return LaunchDescription([master_launch,display_launch,teleop_launch,nav2_launch_loc,nav2_launch])
    else:
        return LaunchDescription([master_launch,slam_launch,display_launch,teleop_launch])