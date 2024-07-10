#
#
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

from launch.events import Shutdown
from launch_ros.actions import Node

def generate_launch_description():
    joy_argument_ = DeclareLaunchArgument('joy_node', default_value="true")
    joy_ = LaunchConfiguration('joy_node')

    # Joy node
    joy_node = Node( package='joy', executable='joy_node',
                     condition=IfCondition(joy_))
  
    # Joy node
    config_1_ = os.path.join(get_package_share_directory('joy_twist'),
                            'config', 'parameters.yaml')
    joy_twist_node = Node( package='joy_twist', executable='joy_twist',
                           output='screen',
                           parameters=[config_1_])

    # trirob
    config_ = os.path.join(get_package_share_directory('triorb_ros2'),
                            'config', 'parameters.yaml')
                            
    triorb_node = Node(package='triorb_ros2', executable='triorb',
                       name='triorb', output='screen',
                       parameters=[config_], 
                       on_exit=Shutdown())

    #return LaunchDescription([joy_argument_, joy_node, joy_twist_node, triorb_node ])
    return LaunchDescription([triorb_node ])
