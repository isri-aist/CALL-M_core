import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch
import launch_ros

from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'icps_bot'
    bot_model_subpath = 'description/bot/bot_description.urdf.xacro'
    world_model_subpath = 'description/world/simple_world.sdf'

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),bot_model_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # node to publish TFs of robot model
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
    )
    """
    #joint states published by Gazebo for the simulation
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )"""

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'icps_bot'],
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
        #launch state pubisher
        node_robot_state_publisher,
        #Load bot in the simulation
        spawn_entity
    ])


