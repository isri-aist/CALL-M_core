import launch
from launch.substitutions import LaunchConfiguration
import launch_ros
import os
import xacro
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='call_m_supervisor').find('call_m_supervisor') 
    bot_model_subpath = 'description/bot/bot_description.urdf.xacro'

    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    command_master_node = launch_ros.actions.Node(
       package='call_m_supervisor',
       executable='command_master_node',
       name='command_master_node',
       output='screen',
    )

    # Use xacro to process the file
    xacro_file = os.path.join(pkg_share,bot_model_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # node to publish TFs of robot model
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
    )
    
    #joint states published by Gazebo for the simulation and by Hardware launch if hardware

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',description='Flag to enable use_sim_time'),
        # Execute laser_scan_merger launch file without xterm
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'launch', 'laser_scan_merger', 'launch.py'],
            output='screen',
         ),
        # Execute slam_toolbox launch file without xterm
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'launch', 'slam_toolbox', 'online_async_launch.py'],
            output='screen',
        ),
        command_master_node,
        robot_localization_node,
        node_robot_state_publisher,
    ])
