import os
from ament_index_python.packages import get_package_share_directory
import launch_ros
from launch import LaunchDescription
import launch
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'call_m_simulation'
    world_model_subpath = 'description/world/simple_world.sdf'
    #world_model_subpath = 'description/world/workshop_example.world'
    pkg_share = launch_ros.substitutions.FindPackageShare(package='call_m_simulation').find('call_m_simulation')

    """
    #joint states published by Gazebo for the simulation, see in URDF files
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )"""

    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
       #parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}, {'debug': True}, {'debug_out_file': os.path.join(pkg_share, 'config/ekf_debug.txt')}]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'call_m_bot'],
                    output='screen')

    #ros_control controllers, defined in controllers.yaml in call_m_supervisor package
    node_controller_wheels = launch.actions.ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner.py', 'wheels_cont'],
        output='screen',
    )

    node_controller_wheels_sup = launch.actions.ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner.py', 'wheels_sup_cont'],
        output='screen',
    )

    node_controller_cams = launch.actions.ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner.py', 'cams_cont'],
        output='screen',
    )

    #simulated bot driver
    node_simu_bot_driver = launch.actions.ExecuteProcess(
        #cmd=['ros2', 'run', 'call_m_simulation', 'simu_bot_driver_node'],
        cmd=['xterm', '-fn', 'xft:fixed:size=12', '-geometry', '100x20','-e', 'ros2', 'run', 'call_m_simulation', 'simu_bot_driver_node'],
        output='screen',
    )

    node_simu_odometry = launch.actions.ExecuteProcess(
        cmd=['ros2', 'run', 'call_m_simulation', 'simu_odometry_node'],
        output='screen',
    )

    # Run the node
    return LaunchDescription([
        #launch Gazebo
        launch.actions.ExecuteProcess(
                cmd=[
                    'gazebo',
                    '--verbose',
                    '-s', 'libgazebo_ros_init.so',
                    '-s', 'libgazebo_ros_factory.so',
                    os.path.join(get_package_share_directory(pkg_name),world_model_subpath),
                ],
                output='screen'),launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',description='Flag to enable use_sim_time'),
        #launch joint publisher
        #joint_state_publisher_node,
        #Load bot in the simulation
        spawn_entity,
        node_simu_odometry,
        robot_localization_node,
        node_controller_wheels,
        node_controller_wheels_sup,
        node_controller_cams,
        node_simu_bot_driver,
    ])


