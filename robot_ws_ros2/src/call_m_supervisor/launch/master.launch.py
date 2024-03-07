import launch
import launch_ros
import os
import xacro
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='call_m_supervisor').find('call_m_supervisor') 
    bot_model_subpath = 'description/bot/bot_description.urdf.xacro'

    cmd_debug = ['xterm', '-fn', 'xft:fixed:size=12', '-geometry', '100x20', '-e', 'ros2', 'run']

    command_master_node = launch_ros.actions.Node(
       package='call_m_supervisor',
       executable='command_master_node',
       name='command_master_node',
       output='screen',
    )

    #command_master_node =  launch.actions.ExecuteProcess(cmd=cmd_debug + ['call_m_supervisor', 'command_master_node'], output='screen')

    # Use xacro to process the file
    xacro_file = os.path.join(pkg_share,bot_model_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # node to publish TFs of robot model
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': LaunchConfiguration('use_sim_time')}] # add other parameters here if required
    )

    # Execute laser_scan_merger launch file without xterm
    scan_merger_node=Node(
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
    )

    # node to publish /clock used to ensure that robot's computer are synced even wihout internet connection (not needed if using chrony)
    clock_sync_publisher = Node(
        package='call_m_supervisor',
        executable='clock_sync_node',
        output='screen',
        parameters=[{'sim_time': LaunchConfiguration('use_sim_time')}] # add other parameters here if required
    )

    #joint states published by Gazebo for the simulation and by Hardware launch if hardware

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value="False",description='Flag to enable use_sim_time'),
        scan_merger_node,
        command_master_node,
        node_robot_state_publisher,
        clock_sync_publisher,
    ])
