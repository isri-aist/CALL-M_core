import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    lid1_node = Node(
        name='rplidar_composition',
        namespace='lidar2',
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB2',
            'serial_baudrate': 115200,  # A1 / A2
            'frame_id': 'laser1_link',
            'inverted': False,
            'angle_compensate': True,
            'topic_name':'scan',
        }],
    )

    lid2_node = Node(
        name='rplidar_composition',
        namespace='lidar1',
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB1',
            'serial_baudrate': 115200,  # A1 / A2
            'frame_id': 'laser2_link',
            'inverted': False,
            'angle_compensate': True,
            'topic_name':'scan',
        }],
    )

    #joint states published by Gazebo for the simulation, but with hardware we need to publish them for RVIZ
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    return LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['xterm', '-fn', 'xft:fixed:size=12', '-geometry', '100x20','-e', 'ros2', 'run', 'call_m_hardware', 'bot_control_driver_node'],
            output='screen',
        ),
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'launch', 'zed_wrapper', 'zed_camera.launch.py','camera_model:=zedm','camera_name:=cam1','serial_number:='],
            output='screen',
        ),
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'launch', 'zed_wrapper', 'zed_camera.launch.py','camera_model:=zedm','camera_name:=cam2','serial_number:='],
            output='screen',
        ),
        #lid1_node,
        #lid2_node,
        joint_state_publisher_node,
    ])