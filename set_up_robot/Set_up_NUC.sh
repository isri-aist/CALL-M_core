#!/bin/bash

#install utilities
sudo apt install -y terminator
sudo apt install -y filezilla

#Install Chrony
sudo apt install -y chrony
sudo cp /call_m_workspace/chrony/chrony.conf.server /etc/chrony/chrony.conf

#ROS2
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-humble-ros-base
sudo apt install -y ros-dev-tools

#Triorb control
sudo apt install -y python3-serial python3-pip ros-humble-lifecycle-py
sudo pip install numpy-quaternion

#Xterm
sudo apt install -y xterm

#Xacro
sudo apt install -y ros-humble-xacro

#robot localization
sudo apt install -y ros-humble-robot-localization

#slam
sudo apt install -y ros-humble-slam-toolbox

#navigation2
sudo apt install -y ros-humble-navigation2
sudo apt install -y ros-humble-nav2-bringup

#joint state publisher
sudo apt install -y ros-humble-joint-state-publisher

#Gazebo
sudo apt install -y gazebo
sudo apt install -y ros-humble-ros2-control
sudo apt install -y ros-humble-ros2-controllers
sudo apt install -y ros-humble-gazebo-ros-pkgs
sudo apt install -y ros-humble-gazebo-ros2-control

#last instructions
echo ""
echo ""
echo "Don't forget the following steps: "
echo "1: Sourcing the ROS2 setup script in .bashrc (hidden file in /home/): source /opt/ros/humble/setup.bash"
