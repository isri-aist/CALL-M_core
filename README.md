# CNRS_AIST_Mobile_Shopping_Robot
https://github.com/Noceo200/CNRS_AIST_Mobile_Shopping_Robot.git
git@github.com:Noceo200/CNRS_AIST_Mobile_Shopping_Robot.git

## Clone the repository with all submodules
```
git clone --recurse-submodules -j8 <SSH or HTTPS Link>
```

## Dependancies (project tested on ros2 humble and ubuntu22.04): 
### ROS2 Humble (Desktop install + Bare Bones + Development tools):
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

### TriOrb control
```
sudo apt install python3-serial python3-pip ros-humble-lifecycle-py
sudo pip install numpy-quaternion
```

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
```
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```
Doc: https://navigation.ros.org/getting_started/index.html

### Zed mini packages
Having a nvidia card installed is mandatory to use CUDA.
Install Nvidia package (+ drivers):
https://developer.nvidia.com/cuda-downloads
Install ZED SDK:
https://www.stereolabs.com/developers/release

### Joint State Publisher
```
sudo apt install ros-humble-joint-state-publisher
```

### If you want to use the simulation (simulation.launch.py):

#### For amd systems:
```
sudo apt install gazebo
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-gazebo-ros2-control
```

#### For arm systems:
```
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
```
The classic gazebo is not working on arm, we need to get arm version.
```
sudo add-apt-repository ppa:openrobotics/gazebo11-non-amd64
sudo apt update
sudo apt install gazebo
```

And we need to install the packages gazebo_ros_pkgs and gazebo_ros2_control from source because they don't appear on arm64 apt packages:

dependencies:
```
sudo apt install libgazebo-dev
sudo apt install libcamera-info-manager-dev
```

Installation of gazebo_ros_pkgs and gazebo_ros2_control:
In any ros2 workspace:
```
cd src
git clone --branch ros2 https://github.com/ros-simulation/gazebo_ros_pkgs.git
git clone --branch humble https://github.com/ros-controls/gazebo_ros2_control.git
cd ..
colcon build
```
(don't forget to source this workspace)
This will install the needed packages, no need to compile them again after.

Details: https://github.com/gazebosim/gazebo-classic/issues/3236
ARM version doc: https://launchpad.net/~openrobotics/+archive/ubuntu/gazebo11-non-amd64

## Workspace Set_up
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

If neeeded, or in case of hardware connection problems, try to get permission for using ports:
```
sudo usermod -aG dialout $USER
sudo chmod a+rw /dev/ttyUSB*
```
!!!Zed mini cameras may need to be replugged on each start up!!!

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

If neeeded, or in case of hardware connection problems, try to get permission for using ports:
```
sudo usermod -aG dialout $USER
sudo chmod a+rw /dev/ttyUSB*
```
!!!Zed mini cameras may need to be replugged on each start up if the cable used are USB-A_USB-C!!!(USB-C_USB-C ok)

Launch the servor on the robot computer:
```
ros2 launch call_m_start_all call_m_start_servor.launch.py
```

Launch the client on any other computer on the same network:
```
ros2 launch call_m_start_all call_m_start_client.launch.py
```

### Sync the computers
```
sudo apt install chrony
```
Configurations files are in /chrony/, they need to replace /etc/chrony/chrony.conf. Servor = remote computer, clients = computers on call_m.
The IP adresses may need to be changed in the configurations files. 

https://robofoundry.medium.com/how-to-sync-time-between-robot-and-host-machine-for-ros2-ecbcff8aadc4
https://access.redhat.com/documentation/en-us/red_hat_enterprise_linux/7/html/system_administrators_guide/ch-configuring_ntp_using_the_chrony_suite#sect-Stopping_chronyd

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

To change the configurations and customize the navigation, check the README file in the workspace 'robot_ws_ros2'.

## Useful tips:
Save a map:
```
ros2 run nav2_map_server map_saver_cli -f "map_name" --ros-args -p map_subscribe_transient_local:=true
```

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

connect to Computers:
```
ssh -X jrluser@hostname.local
```
Or with IPs:
callm01c (NUC): 150.18.226.30
callm01v (JETSON): 150.18.226.22
BUFFALO router: 150.18.66.94

Install last verion of sutdio code on Ubuntu 22.04 (if default snap package crashes):
https://github.com/microsoft/vscode/issues/204159#issuecomment-2151226947

