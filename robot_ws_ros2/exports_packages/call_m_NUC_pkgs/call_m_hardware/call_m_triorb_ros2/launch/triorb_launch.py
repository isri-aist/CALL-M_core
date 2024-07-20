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

    # triorb
    config_ = os.path.join(get_package_share_directory('call_m_triorb_ros2'),
                            'config', 'parameters.yaml')
    triorb_node = Node(package='call_m_triorb_ros2', executable='triorb',
                       name='triorb', output='screen',
                       parameters=[config_], 
                       on_exit=Shutdown())

    return LaunchDescription([triorb_node ])
