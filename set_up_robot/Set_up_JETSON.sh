#!/bin/bash

#install utilities
sudo apt install -y terminator
sudo apt install -y filezilla
sudo apt install -y git

#Install Chrony
sudo apt install -y chrony
sudo cp chrony/chrony.conf.client /etc/chrony/chrony.conf

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

#NVIDIA: Cuda base installer for JETSON, aarch_jetson
#https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=aarch64-jetson&Compilation=Native&Distribution=Ubuntu&target_version=22.04&target_type=deb_local
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/arm64/cuda-ubuntu2204.pin
sudo mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/12.5.1/local_installers/cuda-tegra-repo-ubuntu2204-12-5-local_12.5.1-1_arm64.deb
sudo dpkg -i cuda-tegra-repo-ubuntu2204-12-5-local_12.5.1-1_arm64.deb
sudo cp /var/cuda-tegra-repo-ubuntu2204-12-5-local/cuda-*-keyring.gpg /usr/share/keyrings/
sudo apt-get update
sudo apt-get -y install cuda-toolkit-12-5 cuda-compat-12-5

#Install zed wrapper package and dependancies
cd robot_ws_ros2/src/included_external_packages/
sudo rm -r zed-ros2-wrapper/
git clone --recurse-submodules -j8 -b master https://github.com/stereolabs/zed-ros2-wrapper.git

#last instructions
echo ""
echo ""
echo "Don't forget the following steps: "
echo "Check that the CUDA version used is the last supported one: 'nvcc --version' Upgrade if needed https://docs.nvidia.com/cuda/cuda-for-tegra-appnote/index.html#deployment-considerations-for-cuda-upgrade-package"
echo "1: Sourcing the ROS2 setup script in .bashrc (hidden file in /home/): source /opt/ros/humble/setup.bash"
echo "2: Install ZED mini SDK manually (JETSON Version!!!): https://www.stereolabs.com/developers/release"

