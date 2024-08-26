import launch
from launch import LaunchDescription
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    DIRECTORIES
    """
    dir_nav2 = get_package_share_directory('call_m_nav2')
    dir_slam = get_package_share_directory('call_m_start_all')

    """
    PARAMETERS
    """ 

    declare_param_mode = DeclareLaunchArgument(
        'mode',
        default_value='none (Please specify)',
        description='Mode to launch: COMPLETE or API'
    )

    declare_param_nav_type = DeclareLaunchArgument(
        'nav_type',
        default_value='none',
        description='Navigation type: none, on_fly or localize'
    )

    declare_param_nav_mode = DeclareLaunchArgument(
        'nav_mode',
        #default_value=os.path.join(dir_nav2, 'config', 'nav2_params_omni_simu.yaml'),
        default_value=os.path.join(dir_nav2, 'config', 'nav2_params_omni_simu_tempo.yaml'),
        description='Navigation mode: NAV2 configuration file.'
    )

    declare_param_map_loc = DeclareLaunchArgument(
        'map_loc',
        default_value=os.path.join(dir_nav2, 'maps', 'map.yaml'),
        description='Map: yaml file of known map to navigate in. (used only if nav_type = localize)'
    )

    declare_param_slam_param = DeclareLaunchArgument(
        'slam_param',
        default_value=os.path.join(dir_slam, 'config/mapper_params_online_async_simu.yaml'),
        description='SLAM parameters: yaml file for slam parameters (used only if nav_type != localize)'
    )

    # Use OpaqueFunction to call the function that uses the parameter
    processes_to_launch = OpaqueFunction(function=run_launch_with_parameters)

    # Return the launch description
    return LaunchDescription([
        declare_param_mode,
        declare_param_nav_type,
        declare_param_nav_mode,
        declare_param_map_loc,
        declare_param_slam_param,
        processes_to_launch
    ])


# Define a function to use the parameter in the launch file
def run_launch_with_parameters(context):

    #we get parameters values
    mode = context.launch_configurations['mode']
    nav_type = context.launch_configurations['nav_type']
    nav_mode = context.launch_configurations['nav_mode']
    map_loc = context.launch_configurations['map_loc']
    slam_param = context.launch_configurations['slam_param']

    if(not check_args(mode,nav_type,nav_mode,map_loc,slam_param)):
        return

    #Command to launch xterm and execute ROS 2 launch commands
    cmd_debug = ['xterm', '-fn', 'xft:fixed:size=12', '-geometry', '130x40', '-e', 'ros2', 'launch']
    cmd = ['ros2', 'launch']
    suffix = ['use_sim_time:=true']

    # Construct the absolute paths to the launch files
    master_launch =  launch.actions.ExecuteProcess(cmd=cmd + ['call_m_supervisor', 'master.launch.py'] + suffix, output='screen')
    simu_launch = launch.actions.ExecuteProcess(cmd=cmd + ['call_m_simulation', 'simulation.launch.py']+ suffix, output='screen')
    slam_launch = launch.actions.ExecuteProcess(cmd=cmd + ['slam_toolbox', 'online_async_launch.py', 'slam_params_file:='+slam_param]+ suffix, output='screen')
    display_launch = launch.actions.ExecuteProcess(cmd=cmd + ['call_m_monitor', 'display.launch.py']+ suffix, output='screen')
    teleop_launch =  launch.actions.ExecuteProcess(cmd=cmd + ['call_m_teleoperation', 'teleop.launch.py']+ suffix, output='screen')
    nav2_launch = launch.actions.ExecuteProcess(cmd=cmd + ['call_m_nav2', 'navigation_launch.py','params_file:='+nav_mode] + suffix, output='screen')
    nav2_launch_loc = launch.actions.ExecuteProcess(cmd=cmd + ['call_m_nav2', 'localization_launch.py','map:='+map_loc,'params_file:='+nav_mode] + suffix, output='screen')

    if(mode=="API"):
        if nav_type == "on_fly":
            return [master_launch,slam_launch,simu_launch,nav2_launch]
        elif nav_type == "localize":
            return [master_launch,simu_launch,nav2_launch_loc,nav2_launch]
        else:
            return [master_launch,slam_launch,simu_launch]   
    else: #we launch everything
        if nav_type == "on_fly":
            return [master_launch,slam_launch,simu_launch,display_launch,teleop_launch,nav2_launch]
        elif nav_type == "localize":
            return [master_launch,simu_launch,display_launch,teleop_launch,nav2_launch_loc,nav2_launch]
        else:
            return [master_launch,slam_launch,simu_launch,display_launch,teleop_launch]
        

def check_args(mode,nav_type,nav_mode,map_loc,slam_param):
    tuto = """
    call_m_start_all package launched (simulation version)
    POSSIBLE PARAMETERS: 
    mode: COMPLETE or API (Read README to check interfaces differences)
    nav_type: 'none' (SLAM only), on_fly (SLAM + NAV2) or localize (NAV2 only, know map requested)
    nav_mode: NAV2 configuration file to use. (Don't specify to keep default)
    map_loc: Known map to use (only used for localized navigation when nav_type = localize)
    slam_param: SLAM configuration file to use. (Don't specify to keep default)
    """

    print(tuto)

    #check that specified files exist
    check_mode = ["COMPLETE","API"]
    if(mode not in check_mode):
        print("ERROR: mode: Unknow parameter: ",mode)
        return False   
    check_nav_type = ["none","on_fly","localize"]
    if(nav_type not in check_nav_type):
        print("ERROR: nav_type: Unknow parameter: ",nav_type)
        return False      
    if(not os.path.isfile(nav_mode)):
        print("ERROR: nav_mode: Couldn't find: ",nav_mode)
        return False
    if(not os.path.isfile(map_loc)):
        print("ERROR: map_loc: Couldn't find: ",map_loc)
        return False
    if(not os.path.isfile(slam_param)):
        print("ERROR: slam_param: Couldn't find: ",slam_param)
        return False
    
    feedback = f"""
    LOADED PARAMETERS: 
    mode: {mode}
    nav_type: {nav_type}
    nav_mode: {nav_mode}
    map_loc: {map_loc}
    slam_param: {slam_param}
    """

    print(feedback)
    
    return True