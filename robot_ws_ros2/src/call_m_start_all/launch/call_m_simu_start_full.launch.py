import launch
from launch import LaunchDescription
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    DIRECTORIES
    """
    dir_nav2 = get_package_share_directory('call_m_nav2')
    dir_slam = get_package_share_directory('call_m_supervisor')

    """
    PARAMETERS
    """
    nav_type = "localize" #'none', 'on_fly' or 'localize'

    nav_mode = os.path.join(dir_nav2, 'config', 'nav2_params_omni_simu.yaml') #'diff' or 'omni', path to config file
    map_loc = os.path.join(dir_nav2, 'maps', 'map_test.yaml') #path to the map if using localization
    slam_param = os.path.join(dir_slam, 'config/mapper_params_online_async.yaml') #path to params for slam

    """
    LAUNCHES
    """
    #Command to launch xterm and execute ROS 2 launch commands
    #cmd = ['xterm', '-fn', 'xft:fixed:size=12', '-geometry', '60x20', '-e', 'ros2', 'launch']
    cmd = ['ros2', 'launch']
    suffix = ['use_sim_time:=true']

    # Construct the absolute paths to the launch files
    master_launch =  launch.actions.ExecuteProcess(cmd=cmd + ['call_m_supervisor', 'master.launch.py'] + suffix, output='screen')
    slam_launch = launch.actions.ExecuteProcess(cmd=cmd + ['slam_toolbox', 'online_async_launch.py', 'params_file:='+slam_param]+ suffix, output='screen')
    simu_launch = launch.actions.ExecuteProcess(cmd=cmd + ['call_m_simulation', 'simulation.launch.py'], output='screen')
    display_launch = launch.actions.ExecuteProcess(cmd=cmd + ['call_m_monitor', 'display.launch.py'], output='screen')
    teleop_launch =  launch.actions.ExecuteProcess(cmd=cmd + ['call_m_teleoperation', 'teleop.launch.py'], output='screen')
    nav2_launch = launch.actions.ExecuteProcess(cmd=cmd + ['call_m_nav2', 'navigation_launch.py','params_file:='+nav_mode] + suffix, output='screen')
    nav2_launch_loc1 = launch.actions.ExecuteProcess(cmd=cmd + ['call_m_nav2', 'localization_launch.py','params_file:='+nav_mode,'map:='+map_loc] + suffix, output='screen')
    nav2_launch_loc2 = launch.actions.ExecuteProcess(cmd=cmd + ['call_m_nav2', 'navigation_launch.py','params_file:='+nav_mode,'map_subscribe_transient_local:=true'] + suffix, output='screen')
    #ros2 launch nav2_bringup localization_launch.py params_file:=/home/jrlintern/Desktop/work/CNRS_AIST_Work_All/CNRS_AIST_Mobile_Shopping_Robot/robot_ws_ros2/src/call_m_nav2/config/nav2_params _original.yaml map:=/home/jrlintern/Desktop/work/CNRS_AIST_Work_All/CNRS_AIST_Mobile_Shopping_Robot/robot_ws_ros2/src/call_m_nav2/maps/map_test.yaml use_sim_time:=true
    #ros2 launch call_m_nav2 localization_launch.py params_file:=/home/jrlintern/Desktop/work/CNRS_AIST_Work_All/CNRS_AIST_Mobile_Shopping_Robot/robot_ws_ros2/src/call_m_nav2/config/nav2_params_diff_simu.yaml map:=/home/jrlintern/Desktop/work/CNRS_AIST_Work_All/CNRS_AIST_Mobile_Shopping_Robot/robot_ws_ros2/src/call_m_nav2/maps/map_test.yaml use_sim_time:=true
    #ros2 launch call_m_nav2 navigation_launch.py params_file:=/home/jrlintern/Desktop/work/CNRS_AIST_Work_All/CNRS_AIST_Mobile_Shopping_Robot/robot_ws_ros2/src/call_m_nav2/config/nav2_params_diff_simu.yaml map_subscribe_transient_local:=true use_sim_time:=true

    if nav_type == "on_fly":
        return LaunchDescription([master_launch,slam_launch,simu_launch,display_launch,teleop_launch,nav2_launch])
    elif nav_type == "localize":
        return LaunchDescription([master_launch,simu_launch,display_launch,teleop_launch,nav2_launch_loc1,nav2_launch_loc2])
    else:
        return LaunchDescription([master_launch,slam_launch,simu_launch,display_launch,teleop_launch])