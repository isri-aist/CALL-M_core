import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import launch_ros
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction

"""
Sensor IDs
#IDs obtain with 'ls /dev/serial/by-id/' on ubuntu
"""
cameras_servos_id = 'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0010-if00-port0'
lid1_ID = 'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0011-if00-port0'
lid2_ID = 'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0012-if00-port0' 
ultrasonic1_ID = "usb-MaxBotix_MB1403_HRUSB-MaxSonar-EZ0_MB7VU3MA-if00-port0"
ultrasonic2_ID = "usb-MaxBotix_MB1403_HRUSB-MaxSonar-EZ0_MB7VU36L-if00-port0"
ultrasonic3_ID = "usb-MaxBotix_MB1403_HRUSB-MaxSonar-EZ0_MB7VU3JS-if00-port0"
ultrasonic4_ID = "usb-MaxBotix_MB1403_HRUSB-MaxSonar-EZ0_MB7VU357-if00-port0"

def generate_launch_description():

    # Declare the launch arguments
    declare_param_version = DeclareLaunchArgument(
        'version',
        default_value='none(please specify)',
        description='Version to launch, depending on which computer is running the program: NUC, JETSON, CLIENT or FULL'
    )

    declare_param_mode = DeclareLaunchArgument(
        'mode',
        default_value='API',
        description='Mode to launch: DRIVERS or API'
    )


    # Use OpaqueFunction to call the function that uses the parameter
    processes_to_launch = OpaqueFunction(function=run_launch_with_parameters)

    return LaunchDescription([
        declare_param_version,
        declare_param_mode,
        processes_to_launch
    ])

# Define a function to use the parameter in the launch file
def run_launch_with_parameters(context):
    pkg_share = launch_ros.substitutions.FindPackageShare(package='call_m_drivers').find('call_m_drivers')

    #we get parameters values
    version = context.launch_configurations['version']
    mode = context.launch_configurations['mode']

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
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': False}],
        #parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}, {'debug': True}, {'debug_out_file': os.path.join(pkg_share, 'config/ekf_debug.txt')}]
    )

    #drivers and sensors nodes

    if(not check_args(version,mode)):
        return

    if(version == "NUC" and mode=="API"):
        return [
            joint_state_publisher_node,
            robot_localization_node,
            bot_control_driver(), 
            camera_control_driver_node(), 
        ]  + lid_nodes() + ultr_nodes()
    if(version == "NUC" and mode=="DRIVERS"):
        return [
            bot_control_driver(), 
            camera_control_driver_node(), 
        ]  + lid_nodes() + ultr_nodes()

    if(version == "CLIENT" and mode=="API"):
        print("CLIENT have nothing to manage in API mode, the NUC manage the features.")
        return []  #nothing to do, all manage by NUC
    if(version == "CLIENT" and mode=="DRIVERS"):
        return [
            joint_state_publisher_node,
            robot_localization_node,
        ]  

    if(version == "FULL"):
        return [
            joint_state_publisher_node,
            robot_localization_node,
            bot_control_driver(), 
            camera_control_driver_node(), 
        ] + lid_nodes() + ultr_nodes() + cameras_nodes()

    if(version == "JETSON"):
        return cameras_nodes()


def check_args(version,mode):
    tuto = """
    call_m_drivers package launched
    POSSIBLE PARAMETERS: 
    version: NUC, JETSON, CLIENT or FULL (select the computer on which is running the program)
    mode: DRIVERS or API (Read README to check interfaces differences)
    """

    print(tuto)

    #check that specified files exist
    check_version = ["NUC","JETSON","CLIENT","FULL"]
    if(version not in check_version):
        print("ERROR: version: Unknow parameter: ",version)
        return False 
    check_mode = ["DRIVERS","API"]
    if(mode not in check_mode):
        print("ERROR: mode: Unknow parameter: ",mode)
        return False   
    
    feedback = f"""
    LOADED PARAMETERS:
    version: {version}
    mode: {mode}
    """

    print(feedback)
    
    return True

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
        print("Error, no devices connected? Searched for ID: ",device_id)
        return 'None'

def bot_control_driver():
    #bot_driver V2
    return launch.actions.ExecuteProcess(
            cmd=['xterm','-fn', 'xft:fixed:size=12', '-geometry', '100x20','-e', 'ros2', 'launch', 'call_m_triorb_ros2', 'triorb_launch.py'],
            output='screen',
        )

def camera_control_driver_node():
    port_cameras_servos=find_port_by_device_id(cameras_servos_id)
    if(port_cameras_servos != 'None'):
        return Node(
            name='camera_control_driver_node',
            package='call_m_drivers',
            executable='camera_control_driver_node',
            output='screen',
            parameters=[{
                'device_name': port_cameras_servos,
            }],
        )
    else:
        return 

def lid_nodes():
    port_lid_1=find_port_by_device_id(lid1_ID)
    port_lid_2=find_port_by_device_id(lid2_ID)

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

    to_launch = []

    if(port_lid_1 != 'None'):
        to_launch.append(lid1_node)
    if(port_lid_2 != 'None'):
        to_launch.append(lid2_node)

    return to_launch 


def ultr_nodes():
    port_ultrasonic1 = find_port_by_device_id(ultrasonic1_ID)
    port_ultrasonic2 = find_port_by_device_id(ultrasonic2_ID)
    port_ultrasonic3 = find_port_by_device_id(ultrasonic3_ID)
    port_ultrasonic4 = find_port_by_device_id(ultrasonic4_ID)


    ultr1_node = Node(
        name='sonar_range_node',
        package='call_m_drivers',
        executable='sonar_range_node',
        output='screen',
        parameters=[{
            'device_name': port_ultrasonic1,
            'topic_out':"sonar1/range",
        }],
    )

    ultr2_node = Node(
        name='sonar_range_node',
        package='call_m_drivers',
        executable='sonar_range_node',
        output='screen',
        parameters=[{
            'device_name': port_ultrasonic2,
            'topic_out':"sonar2/range",
        }],
    )

    ultr3_node = Node(
        name='sonar_range_node',
        package='call_m_drivers',
        executable='sonar_range_node',
        output='screen',
        parameters=[{
            'device_name': port_ultrasonic3,
            'topic_out':"sonar3/range",
        }],
    )

    ultr4_node = Node(
        name='sonar_range_node',
        package='call_m_drivers',
        executable='sonar_range_node',
        output='screen',
        parameters=[{
            'device_name': port_ultrasonic4,
            'topic_out':"sonar4/range",
        }],
    )

    to_launch = []

    if(port_ultrasonic1 != 'None'):
        to_launch.append(ultr1_node)
    if(port_ultrasonic2 != 'None'):
        to_launch.append(ultr2_node)
    if(port_ultrasonic3 != 'None'):
        to_launch.append(ultr3_node)
    if(port_ultrasonic4 != 'None'):
        to_launch.append(ultr4_node)

    return to_launch 

def cameras_nodes():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='call_m_drivers').find('call_m_drivers')

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
        ],
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
        ],
    )

    return [camera_1,camera_2]
