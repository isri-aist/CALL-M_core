# CNRS_AIST_Mobile_Shopping_Robot
https://github.com/Noceo200/CNRS_AIST_Mobile_Shopping_Robot.git
git@github.com:Noceo200/CNRS_AIST_Mobile_Shopping_Robot.git

## Dependancies (project tested on ros2 foxy and ubuntu20.04): 
### ROS2 Foxy (Desktop install + Development tools):
https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

### Xterm: to open several terminal automaticcally with ros2 launch files
```
sudo apt install xterm
```

### Xacro for 3D model robot
```
sudo apt install ros-foxy-xacro
```

###  robot localization package
```
sudo apt install ros-foxy-robot-localization
```

###  SLAM package
```
sudo apt  install ros-foxy-slam-toolbox
```

### navigation2 (ROS Framework for robot navigation)
https://navigation.ros.org/getting_started/index.html

### Zed mini packages
Nvidia package (+ drivers):
https://developer.nvidia.com/cuda-downloads
ZED SDK:
https://www.stereolabs.com/developers/release


### If you want to use the simulation (simulation.launch.py):
```
sudo apt install ros-foxy-ros2-control
sudo apt install ros-foxy-ros2-controllers
sudo apt install ros-foxy-gazebo-ros-pkgs
sudo apt install ros-foxy-gazebo-ros2-control
```

## Set_up
### to execute ros2 workspaces:
(directories with src folder and no CmakeList, 'robot_ws_ros2')
```
colcon build --symlink-install
```
### to execute Cmake projects:
(directories with src folder and CmakeList like 'Tests_programs')
```
mkdir build
cd build
cmake ..
make
./<executable>
```

## Launch project
### Ready to flight:

Parameters:
* nav:(string): 
	- "none":(default): Do not use Nav2.
	- "on_fly": Use Nav2 without known map. Allow to navigate on a map generated with SLAM on fly.
	- "localize": Use Nav2 with a known map. Classic navigation with moving obstacles detection.

#### Simulation
```
ros2 launch call_m_start_all call_m_simu_start_full.launch.py
```

#### Real Robot, Hardware
```
ros2 launch call_m_start_all call_m_start_full.launch.py
```

#### Real Robot, Hardware, (Remote control)
Servor (on robot)
```
ros2 launch call_m_start_all call_m_start_servor.launch.py
```

Client (remote computer)
```
ros2 launch call_m_start_all call_m_start_client.launch.py
```

### Launch packages independantly (Useful for debug and testing):

These packages should be launch in the order they are presented below to avoid unwanted behavior.

#### Supervisor: (slam, state_publisher, commands master)
```
ros2 launch call_m_supervisor master.launch.py
```

#### Teleoperation: (Keyboard and joystick commands)
```
ros2 launch call_m_teleoperation teleop.launch.py
```

#### Simulation or Hardware (not both)
Hardware: (All hardware related nodes, joints publisher, localization node)
```
ros2 launch call_m_hardware bot.launch.py
```

Simulation: (Simulated robot and environnement, joints publisher, localization node)
```
ros2 launch call_m_simulation simulation.launch.py
```

#### Visualization: (Rviz)
```
ros2 launch call_m_monitor display.launch.py
```

#### Navigation: (Nav2)
Navigate and generate map on fly:
```
ros2 launch call_m_nav2 navigation_launch.py
```

Navigate with known map:
```
ros2 launch call_m_nav2 localization_launch.py
```

## Configurations
### Edit behavior of scan merger
```
laser_scan_merger/config/params.yaml
```

## Possible launches
### Launch Scan merger
To launch with yaml parameters:
```
ros2 launch laser_scan_merger launch.py
```

To launch with default parameters:
```
ros2 run laser_scan_merger laser_scan_merger_node
```

## Hardware Camera Zed M
https://www.stereolabs.com/docs/ros2/

Having a nvidia card installed is mandatory to use CUDA:
```
nvidia-smi
```

!!!Cable are not reversible for the Zedm, Arrows need to be on optical side!!!

Launch a camera:
```
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=<camera model> camera_name:=<namespace> serial_number:=<serial_number>
```

Parameters/config:
https://www.stereolabs.com/docs/ros2/zed-node#configuration-parameters

Serial numbers:
camera1: 15255448
camera2: 15267217
cameratest: 13024367

## Hardware Servomotor Cameras, Dynamixel XH540-V150
https://www.youtube.com/watch?v=E8XPqDjof4U

Node to control and read
```
ros2 run dynamixel_sdk_examples read_write_node 
or 
ros2 run call_m_hardware camera_control_driver_node
```

Write:
```
ros2 topic pub -1 /set_position dynamixel_sdk_custom_interfaces/msg/SetPosition "{id: 1, position: 2000}"
```

Read:
```
ros2 service call /get_position dynamixel_sdk_custom_interfaces/srv/GetPosition "id: 1"
```

positions = 1000 to 3000 in our case

## Hardware Rp_lidar
https://index.ros.org/p/rplidar_ros/#foxy

(see Hardware doc/Rplidar for more information)

```
ros2 run rplidar_ros rplidar_composition --ros-args -p "serial_port:=/dev/ttyUSB0" -p serial_baudrate:=115200 -p "frame_id:=lidar1_link" -p inverted:=false -p angle_compensate:=true -p "topic_name:=lidar1_scan"
```

### to identify lidars:
See plugs/Unplugs logs
```
dmesg | grep tty
```

See IDs of connected devices
```
ls /dev/serial/by-id/
```

See Current used USB port
```
ls /dev/ttyUSB*
```

### Parameters

```c++
  channel_type_ = this->declare_parameter("channel_type", "serial");
  tcp_ip_ = this->declare_parameter("tcp_ip", "192.168.0.7");
  tcp_port_ = this->declare_parameter("tcp_port", 20108);
  serial_port_ = this->declare_parameter("serial_port", "/dev/ttyUSB0");
  serial_baudrate_ = this->declare_parameter("serial_baudrate", 115200);
  frame_id_ = this->declare_parameter("frame_id", std::string("laser_frame"));
  inverted_ = this->declare_parameter("inverted", false);
  angle_compensate_ = this->declare_parameter("angle_compensate", false);
  flip_x_axis_ = this->declare_parameter("flip_x_axis", false);
  scan_mode_ = this->declare_parameter("scan_mode", std::string());
  topic_name_ = this->declare_parameter("topic_name", std::string("scan"));
  auto_standby_ = this->declare_parameter("auto_standby", false);
```


