# CNRS_AIST_Mobile_Shopping_Robot
https://github.com/Noceo200/CNRS_AIST_Mobile_Shopping_Robot.git
git@github.com:Noceo200/CNRS_AIST_Mobile_Shopping_Robot.git

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

## Dependancies (project tested on ros foxy): 
```
sudo apt install xterm
```
(xterm  = to open several terminal automaticcally with ros2 launch files)

For simulation (simulation.launch.py):
```
sudo apt install ros-foxy-gazebo-ros-pkgs
sudo apt install ros-foxy-gazebo-ros2-control
```

### navigation2 (ROS Framework for robot navigation)
https://navigation.ros.org/getting_started/index.html

### Laser Scan merger Package
The package ros2_laser_scan_merger allow to merge LIDAR data in one scan message.
https://github.com/mich1342/ros2_laser_scan_merger

It depend on:
https://github.com/ros-perception/pointcloud_to_laserscan

This packages have been cloned in ros_workspace/src.

!!! Need to clone foxy branches if using Foxy !!!
```
git clone --branch foxy https://github.com/mich1342/ros2_laser_scan_merger.git
```
```
git clone --branch foxy https://github.com/ros-perception/pointcloud_to_laserscan.git
```

Install:
```
colcon build && source install/setup.bash
```

#### Edit behavior of scan merger
```
laser_scan_merger/config/params.yaml
```

#### Launch Scan merger
To launch with yaml parameters:
```
ros2 launch laser_scan_merger launch.py
```

To launch with default parameters:
```
ros2 run laser_scan_merger laser_scan_merger_node
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

## Hardware Camera Zed M
https://www.stereolabs.com/docs/ros2/

## Hardware Rp_lidar
https://index.ros.org/p/rplidar_ros/#foxy
