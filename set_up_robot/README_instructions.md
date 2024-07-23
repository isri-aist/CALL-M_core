# Install and set up for CALLM robot 
Last update: july 2024
## JETSON ORIN 16G
### 1) Install sdkmanager on this computer
```
sudo apt install sdkmanager
```
https://developer.nvidia.com/sdk-manager

### 2) Flash the JETSON
Start sdk manager:
```
sdkmanager
```
#### STEP1
- Product category: JETSON
- System configuration: Host Machine, Target Hardware: Jetson Orin NX 16GB
- SDK Version: Last
- Additionnal SDK: No

#### STEP2
- Target components: Jetson Linux, Jetson Runtime Components, Jetson SDK Components

#### STEP3
- Select Manual set up for JETSON ORIN 16G
- OEM configuration: Runtime
- Storage Device: NVME (To change if not)
- Plug a screen to the JETSON, it will allow to set it up during runtime when flashing is finished
- Follow instructions (Recovery pin is below fan) and Flash

### 3) After flash, Set up Ubuntu on the JETSON using a keyboard, a mouse and the connected screen
- name: jrluser
- computer name: callm0Xv (X = callm robot number)
- username: jrluser
- password: jrluser
- login automatically: yes

### 4) If no Internet connexion on JETSON, Set up a temporary connection !!!(Before continuing installation of SDK components via sdkmanager)!!!
- The JETSON should have internet through the USB cable connected to your computer or WI-FI
- If not, plug a Ethernet cable between the JETSON and your computer
- And in the 'wired connection settings' of your computer, set the IPV4 method to: 'shared with other computer'

### 5) Update the packages !!!(Before continuing installation of SDK components via sdkmanager)!!!
```
sudo apt update && sudo apt upgrade
```
```
sudo apt reboot
```

### 6) Set up preferences
- In power Settings, blank screen: never

### 7) Install SDK components via sdkmanager
- username: jrluser
- password: jrluser
- IP Adress: 192.168.55.1
- proxy: No proxy

### 8) If no WI-FI, download and install the drivers (This works for AX200 Wi-Fi module)
```
sudo add-apt-repository ppa:canonical-hwe-team/backport-iwlwifi
sudo apt-get update
sudo apt-get install backport-iwlwifi-dkms
```
- Restart the computer, WI-FI should be available.

For faster connection:
- Open '/etc/NetworkManager/conf.d/default-wifi-powersave-on.conf' and set wifi.powersave = 2

## NUC
### 1) Install Ubuntu 22.04 (Via USB bootable device or any other means)
### 2) Set up Ubuntu
- name: jrluser
- computer name: callm0Xc (X = callm robot number)
- username: jrluser
- password: jrluser
- login automatically: yes

### 3) Set up preferences
- In power Settings, blank screen: never

### 4) Update the system
```
sudo apt update && sudo apt upgrade
```
- Restart the computer

### 5) If no WI-FI, install drivers
- Connect the NUC to internet via another way (Ethernet, USB, Bluetooth)
- Follow instructions: https://installati.one/install-backport-iwlwifi-dkms-ubuntu-22-04/

# Set up CALLM workspace (JETSON and NUC)
- First, the JETSON, the NUC and your computer should be on the same network
- Set up your computer with:
```
bash 1_Set_up_USER_dev_amd64.sh
```
- Send necessary files to JETSON and NUC with:
```
bash 2_export_set_up_files.sh
```

## Set up JETSON
- connect to the JETSON with (X = robot number):
```
ssh -X jrluser@callm0Xv.local
```
- Set up with:
```
cd call_m_workspace/
bash Set_up_JETSON.sh
```
- Source ROS2: add the line 'source /opt/ros/humble/setup.bash" in home/.bashrc file
- Install ZED SDK, NVIDIA Jetson version, Manually: https://www.stereolabs.com/developers/release

## Set up NUC
- connect to the NUC with (X = robot number):
```
ssh -X jrluser@callm0Xc.local
```
- Set up with:
```
cd call_m_workspace/
bash Set_up_NUC.sh
```
- Source ROS2: add the line 'source /opt/ros/humble/setup.bash" in home/.bashrc file

# Install CALLM workspace
- You can close all ssh connections with the JETSON an the NUC
- In the CALLM github root repository, open /robot_ws_ros2/README.md, and follow instructions
