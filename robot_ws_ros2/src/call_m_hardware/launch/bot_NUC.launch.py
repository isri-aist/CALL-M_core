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
                    #print("Port: ",link_target)
                    return link_target
        print("No device found for: ",device_id)
        return 'None'
    except:
        print("Error, no devices connected?")
        return 'None'

def generate_launch_description():
    #IDs obtain with 'ls /dev/serial/by-id/' on ubuntu
    servo_motors_ID= 'usb-Prolific_Technology_Inc._USB-Serial_Controller_D-if00-port0'
    port_servo_motors_ID=find_port_by_device_id(servo_motors_ID)

    #bot_driver
    bot_control_driver = launch.actions.ExecuteProcess(
            cmd=['xterm','-fn', 'xft:fixed:size=12', '-geometry', '100x20','-e', 'ros2', 'run', 'call_m_hardware', 'bot_control_driver_node','--ros-args','-p', 'device_name:='+port_servo_motors_ID],
            output='screen',
        )

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',description='Flag to enable use_sim_time'),
        bot_control_driver, 
    ])