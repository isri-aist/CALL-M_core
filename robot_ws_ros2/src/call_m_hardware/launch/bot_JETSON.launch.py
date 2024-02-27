import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import launch_ros
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='call_m_hardware').find('call_m_hardware')

    # ZED Wrapper nodes
    config_cam1_file = os.path.join(pkg_share, 'config/cam1_zedm.yaml')
    camera_1 = Node(
        package='zed_wrapper',
        namespace='cam1',
        executable='zed_wrapper',
        name='zed_node',
        output='screen',
        # prefix=['xterm -e valgrind --tools=callgrind'],
        # prefix=['xterm -e gdb -ex run --args'],
        #prefix=['gdbserver localhost:3000'],
        parameters=[
            # YAML files
            config_cam1_file,
        ]
    )

    config_cam2_file = os.path.join(pkg_share, 'config/cam2_zedm.yaml')
    camera_2 = Node(
        package='zed_wrapper',
        namespace='cam2',
        executable='zed_wrapper',
        name='zed_node',
        output='screen',
        # prefix=['xterm -e valgrind --tools=callgrind'],
        # prefix=['xterm -e gdb -ex run --args'],
        #prefix=['gdbserver localhost:3000'],
        parameters=[
            # YAML files
            config_cam2_file,
        ]
    )

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',description='Flag to enable use_sim_time'),
        camera_1, 
        camera_2, 
    ])