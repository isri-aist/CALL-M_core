#!/bin/bash

# Remove and recreate directories
rm -r exports_packages/
mkdir exports_packages/
touch exports_packages/COLCON_IGNORE
mkdir -p exports_packages/call_m_JETSON_pkgs/ exports_packages/call_m_NUC_pkgs/

# Check if parameter is provided and handle NUC packages
if [ "$1" = 2 ] || [ "$1" = 4 ]; then
    # NUC packages
    # Export files for 'call_m_hardware/call_m_drivers' NUC version
    mkdir -p exports_packages/call_m_NUC_pkgs/call_m_hardware/call_m_drivers/src
    mkdir -p exports_packages/call_m_NUC_pkgs/call_m_hardware/call_m_drivers/config
    mkdir -p exports_packages/call_m_NUC_pkgs/call_m_hardware/call_m_drivers/launch
    cp -r src/call_m_hardware/call_m_drivers/src/ exports_packages/call_m_NUC_pkgs/call_m_hardware/call_m_drivers/
    cp src/call_m_hardware/call_m_drivers/launch/bot.launch.py exports_packages/call_m_NUC_pkgs/call_m_hardware/call_m_drivers/launch/
    cp src/call_m_hardware/call_m_drivers/CMakeLists.txt exports_packages/call_m_NUC_pkgs/call_m_hardware/call_m_drivers/
    cp src/call_m_hardware/call_m_drivers/package.xml exports_packages/call_m_NUC_pkgs/call_m_hardware/call_m_drivers/

    # Export files for 'call_m_hardware/call_m_triorb_ros2' NUC version
    cp -r src/call_m_hardware/call_m_triorb_ros2/ exports_packages/call_m_NUC_pkgs/call_m_hardware/

    # Export files for 'call_m_start_all' NUC version
    cp -r src/call_m_start_all/ exports_packages/call_m_NUC_pkgs/

    # Export files for 'call_m_teleoperation', just in case needed
    cp -r src/call_m_teleoperation/ exports_packages/call_m_NUC_pkgs/
    
    # Export files for 'call_m_supervisor'
    cp -r src/call_m_supervisor/ exports_packages/call_m_NUC_pkgs/
    
    # Export files for 'call_m_nav2'
    cp -r src/call_m_nav2/ exports_packages/call_m_NUC_pkgs/

    echo "-- NUC packages exported in /exports_packages/call_m_NUC_pkgs/"
fi

# Check if parameter is provided and handle JETSON packages
if [ "$1" = 3 ] || [ "$1" = 4 ]; then
    # JETSON packages
    # Export files for 'call_m_hardware/call_m_drivers' JETSON version
    mkdir -p exports_packages/call_m_JETSON_pkgs/call_m_hardware/call_m_drivers/src
    mkdir -p exports_packages/call_m_JETSON_pkgs/call_m_hardware/call_m_drivers/config
    mkdir -p exports_packages/call_m_JETSON_pkgs/call_m_hardware/call_m_drivers/launch
    cp -r src/call_m_hardware/call_m_drivers/src/ exports_packages/call_m_JETSON_pkgs/call_m_hardware/call_m_drivers/
    cp src/call_m_hardware/call_m_drivers/config/cam1_zedm.yaml exports_packages/call_m_JETSON_pkgs/call_m_hardware/call_m_drivers/config/
    cp src/call_m_hardware/call_m_drivers/config/cam2_zedm.yaml exports_packages/call_m_JETSON_pkgs/call_m_hardware/call_m_drivers/config/
    cp src/call_m_hardware/call_m_drivers/launch/bot.launch.py exports_packages/call_m_JETSON_pkgs/call_m_hardware/call_m_drivers/launch/
    cp src/call_m_hardware/call_m_drivers/CMakeLists.txt exports_packages/call_m_JETSON_pkgs/call_m_hardware/call_m_drivers/
    cp src/call_m_hardware/call_m_drivers/package.xml exports_packages/call_m_JETSON_pkgs/call_m_hardware/call_m_drivers/

    # Export files for 'call_m_start_all' JETSON version
    mkdir -p exports_packages/call_m_JETSON_pkgs/call_m_start_all/config
    mkdir -p exports_packages/call_m_JETSON_pkgs/call_m_start_all/launch
    cp src/call_m_start_all/launch/call_m_hardware_start.launch.py exports_packages/call_m_JETSON_pkgs/call_m_start_all/launch/
    cp src/call_m_start_all/CMakeLists.txt exports_packages/call_m_JETSON_pkgs/call_m_start_all/
    cp src/call_m_start_all/package.xml exports_packages/call_m_JETSON_pkgs/call_m_start_all/

    echo "-- JETSON packages exported in /exports_packages/call_m_JETSON_pkgs/"
fi

#Cleaning NUC former Packages
if [ "$1" = 2 ] || [ "$1" = 4 ]; then
     echo "Cleaning NUC packages..."
      ssh jrluser@callm$2c.local << EOF
        cd call_m_workspace/robot_ws_ros2/src
        rm -r call_m_NUC_pkgs
EOF
fi

#Cleaning JETSON former Packages
if [ "$1" = 3 ] || [ "$1" = 4 ]; then
     echo "Cleaning JETSON packages..."
      ssh jrluser@callm$2v.local << EOF
        cd call_m_workspace/robot_ws_ros2/src
        rm -r call_m_JETSON_pkgs
EOF
fi

# Update NUC packages
if [ "$1" = 2 ] || [ "$1" = 4 ]; then
    echo "Updating NUC packages..."
    scp -r exports_packages/call_m_NUC_pkgs/ jrluser@callm$2c.local:/home/jrluser/call_m_workspace/robot_ws_ros2/src/
fi

# Update JETSON packages
if [ "$1" = 3 ] || [ "$1" = 4 ]; then
    echo "Updating JETSON packages..."
    scp -r exports_packages/call_m_JETSON_pkgs/ jrluser@callm$2v.local:/home/jrluser/call_m_workspace/robot_ws_ros2/src/
fi

