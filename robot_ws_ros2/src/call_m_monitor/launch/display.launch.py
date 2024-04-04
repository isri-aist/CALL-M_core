import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='call_m_monitor').find('call_m_monitor')
    default_rviz_config_path = os.path.join(pkg_share, 'config/rviz/urdf_config_nav2_humble2_debug_cam.rviz')

    #node to launch Rviz with
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,description='Absolute path to rviz config file'),
        rviz_node,
    ])
