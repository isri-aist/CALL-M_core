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
    pkg_share = launch_ros.substitutions.FindPackageShare(package='call_m_hardware').find('call_m_hardware')

    #IDs obtain with 'ls /dev/serial/by-id/' on ubuntu
    servo_motors_ID= 'usb-Prolific_Technology_Inc._USB-Serial_Controller_D-if00-port0'
    lid1_ID = 'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0011-if00-port0'
    lid2_ID = 'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0012-if00-port0' 
    cameras_servos_id = 'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
    port_lid_1=find_port_by_device_id(lid1_ID)
    port_lid_2=find_port_by_device_id(lid2_ID)
    port_cameras_servos=find_port_by_device_id(cameras_servos_id)
    port_servo_motors_ID=find_port_by_device_id(servo_motors_ID)

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
            'device_name': port_cameras_servos,
        }],
    )

    #joint states published by Gazebo for the simulation, but with hardware we need to publish them for RVIZ
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
       #parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}, {'debug': True}, {'debug_out_file': os.path.join(pkg_share, 'config/ekf_debug.txt')}]
    )

    #bot_driver
    bot_control_driver = launch.actions.ExecuteProcess(
            cmd=['xterm','-fn', 'xft:fixed:size=12', '-geometry', '100x20','-e', 'ros2', 'run', 'call_m_hardware', 'bot_control_driver_node','--ros-args','-p', 'device_name:='+port_servo_motors_ID],
            output='screen',
        )
    
    #cameras
    """
    camera_1 = launch.actions.ExecuteProcess(
            cmd=['ros2', 'launch', 'zed_wrapper', 'zed_camera.launch.py','camera_model:=zedm','camera_name:=cam1','serial_number:=15255448','use_sim_time:=true'],
            output='screen',
        )
        
    
    camera_2 = launch.actions.ExecuteProcess(
            cmd=['ros2', 'launch', 'zed_wrapper', 'zed_camera.launch.py','camera_model:=zedm','camera_name:=cam2','serial_number:=15267217','use_sim_time:=true'],
            output='screen',
        )
    """

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
        lid1_node, 
        lid2_node, 
        camera_control_driver_node, 
        bot_control_driver, 
        camera_1, 
        camera_2, 
        joint_state_publisher_node,
        robot_localization_node, 
    ])