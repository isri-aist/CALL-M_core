#!/bin/bash

rm -r exports_set_up/
mkdir exports_set_up/
touch exports_set_up/COLCON_IGNORE
mkdir -p exports_set_up/NUC/call_m_workspace/ exports_set_up/JETSON/call_m_workspace/

#NUC files
#export files for 'chrony' NUC version
mkdir exports_set_up/NUC/call_m_workspace/chrony/
cp ../chrony/chrony.conf.server exports_set_up/NUC/call_m_workspace/chrony/

#export files for ros2 workspace, only external packages, NUC version
mkdir -p exports_set_up/NUC/call_m_workspace/robot_ws_ros2/src/included_external_packages/
cp -r ../robot_ws_ros2/src/included_external_packages/DynamixelSDK/ exports_set_up/NUC/call_m_workspace/robot_ws_ros2/src/included_external_packages/
cp -r ../robot_ws_ros2/src/included_external_packages/nmea_msgs/ exports_set_up/NUC/call_m_workspace/robot_ws_ros2/src/included_external_packages/
cp -r ../robot_ws_ros2/src/included_external_packages/rplidar_ros/ exports_set_up/NUC/call_m_workspace/robot_ws_ros2/src/included_external_packages/
cp -r ../robot_ws_ros2/src/included_external_packages/triorb-core/ exports_set_up/NUC/call_m_workspace/robot_ws_ros2/src/included_external_packages/

#set_up file, NUC Version
cp Set_up_NUC.sh exports_set_up/NUC/call_m_workspace/

#build file NUC Version
cp ../robot_ws_ros2/clean_build.sh exports_set_up/NUC/call_m_workspace/robot_ws_ros2/

echo "-- NUC set_up files exported in /exports_set_up/NUC/"

#JETSON files
#export files for 'chrony' JETSON version
mkdir exports_set_up/JETSON/call_m_workspace/chrony/
cp ../chrony/chrony.conf.client exports_set_up/JETSON/call_m_workspace/chrony/

#export files for tools programs, JETSON version
mkdir -p exports_set_up/JETSON/call_m_workspace/tools_programs/
cp -r ../tools_programs/get_serial_Zed_cameras/ exports_set_up/JETSON/call_m_workspace/tools_programs/

#export files for ros2 workspace, only external packages, JETSON version
mkdir -p exports_set_up/JETSON/call_m_workspace/robot_ws_ros2/src/included_external_packages/
cp -r ../robot_ws_ros2/src/included_external_packages/DynamixelSDK/ exports_set_up/JETSON/call_m_workspace/robot_ws_ros2/src/included_external_packages/
cp -r ../robot_ws_ros2/src/included_external_packages/nmea_msgs/ exports_set_up/JETSON/call_m_workspace/robot_ws_ros2/src/included_external_packages/

#set_up file, JETSON Version
cp Set_up_JETSON.sh exports_set_up/JETSON/call_m_workspace/

#build file JETSON Version
cp ../robot_ws_ros2/clean_build.sh exports_set_up/JETSON/call_m_workspace/robot_ws_ros2/

echo "-- JETSON set_up files exported in /exports_set_up/JETSON/"

#send NUC files
scp -r exports_set_up/NUC/call_m_workspace/ jrluser@callm01c.local:/home/jrluser/

#send JETSON files
scp -r exports_set_up/JETSON/call_m_workspace/ jrluser@callm01v.local:/home/jrluser/
