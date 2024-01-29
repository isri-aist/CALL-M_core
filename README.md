# CNRS_AIST_Mobile_Shopping_Robot
https://github.com/Noceo200/CNRS_AIST_Mobile_Shopping_Robot.git
git@github.com:Noceo200/CNRS_AIST_Mobile_Shopping_Robot.git

## Dependancies (project tested on ros2 humble and ubuntu22.04): 
### ROS2 Humble (Desktop install + Bare Bones + Development tools):
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

### Xterm: to open several terminal automaticcally with ros2 launch files
```
sudo apt install xterm
```

### Xacro for 3D model robot
```
sudo apt install ros-humble-xacro
```

###  robot localization package
```
sudo apt install ros-humble-robot-localization
```

###  SLAM package
```
sudo apt  install ros-humble-slam-toolbox
```

### navigation2 (ROS Framework for robot navigation)
https://navigation.ros.org/getting_started/index.html

### Zed mini packages
Having a nvidia card installed is mandatory to use CUDA.
Nvidia package (+ drivers):
https://developer.nvidia.com/cuda-downloads
ZED SDK:
https://www.stereolabs.com/developers/release

### Joint State Publisher
```
sudo apt install ros-humble-joint-state-publisher
```

### If you want to use the simulation (simulation.launch.py):
```
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-gazebo-ros2-control
```

## Set_up
### Compile ros2 workspace 'robot_ws_ros2'
```
colcon build --symlink-install
```

## Quick Start

* See the part 'Configurations and customizations' below for more customizations.
* See the part 'Workspace details' below for details.

### Use the simulated robot
```
ros2 launch call_m_start_all call_m_simu_start_full.launch.py
```

### Use the real Robot
Connect all the components:
* Zedm cameras x2
* Lidars x2
* TriOrb plateforme
* Servos x2 (only one cable)

Launch this on the robot computer:
```
ros2 launch call_m_start_all call_m_start_full.launch.py
```

### Remote control the real Robot
Connect all the components:
* Zedm cameras x2
* Lidars x2
* TriOrb plateforme
* Servos x2 (only one cable)

Launch the servor on the robot computer:
```
ros2 launch call_m_start_all call_m_start_servor.launch.py
```

Launch the client on any other computer on the same network:
```
ros2 launch call_m_start_all call_m_start_client.launch.py
```

## Workspace details
* Hardware_doc:
	Contains informations and links to help set up the components.
		
* resources:
	Contains divers resources like images, schematics.

* robot_ws_ros2:
	This is the main directory that contains the whole ros2 workspace for using the simulated or the real CALL_M robot. (Check the README file in this directory for more details)
	
* tools_programs:
	Contains other useful programs for debugging or set up the components correctly. (Check the README file in this directory for more details)

## Configurations and customizations

Parameters that can be changed in the following launch files:
* nav_type:(string): 
	- "none":(default): Do not use Nav2, just SLAM.
	- "on_fly": Use Nav2 without known map. Allow to navigate on a map generated with SLAM on fly.
	- "localize": Use Nav2 with a known map. Classic navigation with moving obstacles detection.
* nav_mode:(string):
	- "diff": The robot will navigate like a differential robot
	- "omni": The robot will navigate like an holonomic robot

## useful tips:
Check if Nvidia card is available:
```
nvidia-smi
```

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

