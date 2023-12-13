import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    lid1_node = Node(
        name='rplidar_composition',
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,  # A1 / A2
            'frame_id': 'laser1_link',
            'inverted': False,
            'angle_compensate': True,
            'topic_name':'lidar1_scan',
        }],
    )

    return LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['xterm', '-fn', 'xft:fixed:size=12', '-geometry', '100x20','-e', 'ros2', 'run', 'call_m_hardware', 'bot_control_driver_node'],
            output='screen',
        ),
        lid1_node,
    ])