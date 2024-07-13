import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import launch_ros
from launch.substitutions import LaunchConfiguration


def find_port_by_device_id(device_id):
    serial_by_id_dir = '/dev/serial/by-id/'
    try:
        for entry in os.listdir(serial_by_id_dir):
            #print("Devices: ",entry)
            entry_path = os.path.join(serial_by_id_dir, entry)
            if os.path.islink(entry_path):
                link_target = os.path.realpath(entry_path)
                if device_id == entry:
                    print("Device found: " + device_id+ "\nPort: ",link_target)
                    return link_target
        print("No device found for: ",device_id)
        return 'None'
    except:
        print("Error, no devices connected?")
        return 'None'

def generate_launch_description():
    #IDs obtain with 'ls /dev/serial/by-id/' on ubuntu
    #servo_motors_ID= 'usb-Prolific_Technology_Inc._USB-Serial_Controller_D-if00-port0'
    cameras_servos_id = 'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0010-if00-port0'
    lid1_ID = 'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0011-if00-port0'
    lid2_ID = 'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0012-if00-port0' 
    ultrasonic1_ID = "usb-MaxBotix_MB1403_HRUSB-MaxSonar-EZ0_MB7VU3MA-if00-port0"
    ultrasonic2_ID = "usb-MaxBotix_MB1403_HRUSB-MaxSonar-EZ0_MB7VU36L-if00-port0"
    ultrasonic3_ID = "usb-MaxBotix_MB1403_HRUSB-MaxSonar-EZ0_MB7VU3JS-if00-port0"
    ultrasonic4_ID = "usb-MaxBotix_MB1403_HRUSB-MaxSonar-EZ0_MB7VU357-if00-port0"
    #port_servo_motors_ID=find_port_by_device_id(servo_motors_ID)
    port_cameras_servos=find_port_by_device_id(cameras_servos_id)
    port_lid_1=find_port_by_device_id(lid1_ID)
    port_lid_2=find_port_by_device_id(lid2_ID)
    port_ultrasonic1 = find_port_by_device_id(ultrasonic1_ID)
    port_ultrasonic2 = find_port_by_device_id(ultrasonic2_ID)
    port_ultrasonic3 = find_port_by_device_id(ultrasonic3_ID)
    port_ultrasonic4 = find_port_by_device_id(ultrasonic4_ID)

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

    #bot_driver
    """bot_control_driver = launch.actions.ExecuteProcess(
            cmd=['xterm','-fn', 'xft:fixed:size=12', '-geometry', '100x20','-e', 'ros2', 'run', 'call_m_hardware', 'bot_control_driver_node','--ros-args','-p', 'device_name:='+port_servo_motors_ID,'-p', 'sim_time:=true'],
            output='screen',
        )"""
    bot_control_driver_v2 = launch.actions.ExecuteProcess(
            cmd=['xterm','-fn', 'xft:fixed:size=12', '-geometry', '100x20','-e', 'ros2', 'launch', 'triorb_ros2', 'triorb_launch.py'],
            output='screen',
        )

    camera_control_driver_node = Node(
        name='camera_control_driver_node',
        package='call_m_hardware',
        executable='camera_control_driver_node',
        output='screen',
        parameters=[{
            'device_name': port_cameras_servos,
        }],
    )

    ultr1_node = Node(
        name='sonar_range_node',
        package='call_m_hardware',
        executable='sonar_range_node',
        output='screen',
        parameters=[{
            'device_name': port_ultrasonic1,
            'topic_out':"sonar1/range",
        }],
    )

    ultr2_node = Node(
        name='sonar_range_node',
        package='call_m_hardware',
        executable='sonar_range_node',
        output='screen',
        parameters=[{
            'device_name': port_ultrasonic2,
            'topic_out':"sonar2/range",
        }],
    )

    ultr3_node = Node(
        name='sonar_range_node',
        package='call_m_hardware',
        executable='sonar_range_node',
        output='screen',
        parameters=[{
            'device_name': port_ultrasonic3,
            'topic_out':"sonar3/range",
        }],
    )

    ultr4_node = Node(
        name='sonar_range_node',
        package='call_m_hardware',
        executable='sonar_range_node',
        output='screen',
        parameters=[{
            'device_name': port_ultrasonic4,
            'topic_out':"sonar4/range",
        }],
    )

    return LaunchDescription([
        bot_control_driver_v2, 
        camera_control_driver_node, 
        lid1_node, 
        lid2_node, 
        ultr1_node,
        ultr2_node,
        ultr3_node,
        ultr4_node,
    ])