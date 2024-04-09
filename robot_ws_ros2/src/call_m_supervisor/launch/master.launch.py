import launch
import launch_ros
import os
import xacro
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import re

def remove_comments(text):
    pattern = r'<!--(.*?)-->'
    return re.sub(pattern, '', text, flags=re.DOTALL)

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='call_m_supervisor').find('call_m_supervisor') 
    bot_model_subpath = 'description/bot/bot_description.urdf.xacro'

    cmd_debug = ['xterm', '-fn', 'xft:fixed:size=12', '-geometry', '130x40', '-e', 'ros2', 'launch']
    cmd = ['ros2', 'launch']

    command_master_node = launch_ros.actions.Node(
       package='call_m_supervisor',
       executable='command_master_node',
       name='command_master_node',
       output='screen',
    )

    #command_master_node =  launch.actions.ExecuteProcess(cmd=cmd_debug + ['call_m_supervisor', 'command_master_node'], output='screen')

    # Use xacro to process the file
    xacro_file = os.path.join(pkg_share,bot_model_subpath)
    robot_description_raw = remove_comments(xacro.process_file(xacro_file).toxml())

    print(robot_description_raw)

    # node to publish TFs of robot model
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': LaunchConfiguration('use_sim_time')}] # add other parameters here if required
    )

    # Execute laser_scan_merger launch file without xterm ow replaced by lasercan_toolbox node)
    """scan_merger_node=Node(
        package = 'laser_scan_merger',
        name = 'laser_scan_merger_node',
        executable = 'laser_scan_merger_node',
        parameters = [{
            'lidar1_start_angle': 0.785398163, #45 deg
            'lidar1_end_angle' : 5.497787144, #-45 = 315 deg
            'lidar1_angle_origin_offset' : 3.141592654, #180 deg with rpLidars
            'lidar2_start_angle' : 0.785398163, #45 deg
            'lidar2_end_angle' : 5.497787144, #-45 = 315 deg
            'lidar2_angle_origin_offset' : 3.141592654, #180 deg with rpLidars
            'topic_lid1' : "lidar1/scan",  
            'topic_lid2' : "lidar2/scan",
            'topic_out' : "scan",
            'new_frame' : "base2_link",
            'rate' : 20.0,
            'sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )"""

    laserscan_toolbox_params_file = os.path.join(pkg_share,"config/laserscan_toolbox_params.yaml")
    #laserscan_toolbox=launch.actions.ExecuteProcess(cmd=cmd + ['multi-laserscan-toolbox-ros2', 'laserscan_toolbox.launch.py','params_file:='+laserscan_toolbox_params_file,'use_sim_time:=true'], output='screen')
    #we use node directly because we want to get use_sim_time from another launch file.
    laserscan_toolbox=Node(
        parameters=[
          laserscan_toolbox_params_file,
          {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        package = 'multi-laserscan-toolbox-ros2',
        name = 'laserscan_toolbox_node',
        executable = 'laserscan_toolbox_node',
        output='screen'
    )

    # node to publish /clock used to ensure that robot's computer are synced even wihout internet connection (not needed if using chrony)
    clock_sync_publisher = Node(
        package='call_m_supervisor',
        executable='clock_sync_node',
        output='screen',
        parameters=[{'sim_time': LaunchConfiguration('use_sim_time')}] # add other parameters here if required
    )

    #joint states published by Gazebo for the simulation and by Hardware launch if hardware

    #Depth to scan converters
    #ros2 launch depth-filter-scan-converter depth_filter_scan_converter.launch.py use_sim_time:=true
    depth_converter_params_file1 = os.path.join(pkg_share,"config/depth_filter_scan_converter_params1.yaml")
    depth_converter1 = Node(
        parameters=[
          depth_converter_params_file1,
          {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        package = 'depth-filter-scan-converter',
        name = 'depth_filter_scan_converter_node',
        executable = 'depth_filter_scan_converter_node',
        output='screen'
    )

    depth_converter_params_file2 = os.path.join(pkg_share,"config/depth_filter_scan_converter_params2.yaml")
    depth_converter2 = Node(
        parameters=[
          depth_converter_params_file2,
          {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        package = 'depth-filter-scan-converter',
        name = 'depth_filter_scan_converter_node',
        executable = 'depth_filter_scan_converter_node',
        output='screen'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value="False",description='Flag to enable use_sim_time'),
        depth_converter1,
        depth_converter2,
        laserscan_toolbox, 
        command_master_node,
        node_robot_state_publisher,
        clock_sync_publisher,
    ])
