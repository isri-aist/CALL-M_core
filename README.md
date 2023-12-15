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
sudo apt install ros-foxy-gazebo-ros-pkgs
sudo apt install ros-foxy-gazebo-ros2-control
```

## Set_up
### to execute ros2 workspaces:
(directories with src folder and no CmakeList)
```
colcon build
```
### to execute Cmake projects:
(directories with src folder and CmakeList)
```
mkdir build
cd build
cmake ..
make
./<executable>
```


## Launch project
For teleoperate real robot:
```
ros2 launch c_pkg bot_teleoperate_launch.py
```
For Nav2 simulation launch + scan merger:
```
colcon build ; ros2 launch nav2_simu display.launch.py
```

to publish map=>odom TF needed by Nav2 plugin:
```
ros2 launch slam_toolbox online_async_launch.py
```
to publish costmaps and footprints needed for all other Nav2 plugins, and also launch all Nav2 plugins configured in nav2_params.yaml:
```
ros2 launch nav2_bringup navigation_launch.py params_file:=<full/path/to/config/nav2_params.yaml>
```
```
ros2 launch nav2_bringup navigation_launch.py params_file:=/home/jrlintern/Desktop/work/CNRS_AIST_Work_All/CNRS_AIST_Mobile_Shopping_Robot/robot_ws_ros2/src/nav2_simu/config/nav2_params.yaml
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

Launch a camera:
```
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=<camera model> camera_name:=<namespace> serial_number:=<serial_number>
```

Parameters/config:
https://www.stereolabs.com/docs/ros2/zed-node#configuration-parameters

Serial numbers:
camera1:
camera2:
cameratest:

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


