import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch

from launch_ros.actions import Node

def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'call_m_simulation'
    world_model_subpath = 'description/world/simple_world.sdf'


    """
    #joint states published by Gazebo for the simulation
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )"""

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'call_m_bot'],
                    output='screen')

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
        spawn_entity
    ])


