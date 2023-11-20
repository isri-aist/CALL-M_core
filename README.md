# CNRS_AIST_Mobile_Shopping_Robot

## Dependancies: 
- xterm (to open several terminal automaticcally with ros2 launch files)
```
sudo apt install xterm
```
- navigation2 (ROS Framework for robot navigation)
https://navigation.ros.org/getting_started/index.html

### Used in tutorial
- joint-state-publisher-gui
```
sudo apt install ros-<ros2-distro>-joint-state-publisher-gui
```
- xacro (help you create a shorter and readable XML)
```
sudo apt install ros-<ros2-distro>-xacro
```

## Launch project
```
ros2 launch c_pkg bot_teleoperate_launch.py
```

## Navigation2 Framework

### Turtle_bot tutorial:
```
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```
Use 2d pose estimate button on RVIZ to estimate the initial position, and then use navigate button to go where wanted.

### First-time robot setup Tutorial

#### Writing URDF File:

