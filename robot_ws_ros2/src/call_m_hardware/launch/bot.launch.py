import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def find_port_by_device_id(device_id):
    serial_by_id_dir = '/dev/serial/by-id/'
    for entry in os.listdir(serial_by_id_dir):
        print("Devices: ",entry)
        entry_path = os.path.join(serial_by_id_dir, entry)
        if os.path.islink(entry_path):
            link_target = os.path.realpath(entry_path)
            if device_id == entry:
                print("Port: ",link_target)
                return link_target
    return 'None'

def generate_launch_description():

    #IDs obtain with 'ls /dev/serial/by-id/' on ubuntu
    lid1_ID = 'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0003-if00-port0'
    lid2_ID = 'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0002-if00-port0' 
    cameras_id = 'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
    port_lid_1=find_port_by_device_id(lid1_ID)
    port_lid_2=find_port_by_device_id(lid2_ID)
    port_cameras=find_port_by_device_id(cameras_id)

    lid1_node = Node(
        name='rplidar_composition',
        namespace='lidar1',
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': port_lid_1,
            'serial_baudrate': 115200,  # A1 / A2
            'frame_id': 'lidar1_link',
            'inverted': False,
            'angle_compensate': True,
            'topic_name':'scan',
        }],
    )

    lid2_node = Node(
        name='rplidar_composition',
        namespace='lidar2',
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': port_lid_2,
            'serial_baudrate': 115200,  # A1 / A2
            'frame_id': 'lidar2_link',
            'inverted': False,
            'angle_compensate': True,
            'topic_name':'scan',
        }],
    )

    camera_control_driver_node = Node(
        name='camera_control_driver_node',
        package='call_m_hardware',
        executable='camera_control_driver_node',
        output='screen',
        parameters=[{
            'device_name': port_cameras,
        }],
    )

    #joint states published by Gazebo for the simulation, but with hardware we need to publish them for RVIZ
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    return LaunchDescription([
        lid1_node,
        lid2_node,
        joint_state_publisher_node,
        camera_control_driver_node,
        launch.actions.ExecuteProcess(
            cmd=['xterm', '-fn', 'xft:fixed:size=12', '-geometry', '100x20','-e', 'ros2', 'run', 'call_m_hardware', 'bot_control_driver_node'],
            output='screen',
        ),
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'launch', 'zed_wrapper', 'zed_camera.launch.py','camera_model:=zedm','camera_name:=cam1','serial_number:=15255448'],
            output='screen',
        ),
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'launch', 'zed_wrapper', 'zed_camera.launch.py','camera_model:=zedm','camera_name:=cam2','serial_number:=15267217'],
            output='screen',
        ),
    ])