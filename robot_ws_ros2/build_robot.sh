#!/bin/bash

# Check if a parameter is provided
if [ $# -lt 2 ]; then
  echo "Not enough parameters provided."
  echo "Valid parameters: (1:REMOTE 2:REMOTE+NUC, 3:REMOTE+JETSON, 4:REMOTE+NUC+JETSON) and 'robot_number identifier'"
  echo ""
  exit
else
  # Check if the parameter is an integer between 1 and 3
  if [[ $1 =~ ^-?[0-9]+$ ]] && [ $1 -gt 0 ] && [ $1 -lt 5 ]; then
    echo "Integer parameter provided: $1"
    if [ $1 -eq 1 ]; then
   	# Build client only
   	echo "Building REMOTE..."
  	colcon build --symlink-install
    elif [ $1 -eq 2 ]; then
      echo "Building REMOTE + NUC (Ensure they are on the same network)"
      # Build client
      echo "Building REMOTE..."
      colcon build --symlink-install
      # Export packages to robot's computers
      echo ""
      bash export_NUC_JETSON_pkgs.sh "$@"
      echo ""
      # Build robot's computers
      echo ""
      echo "Building NUC..."
      echo ""
      ssh jrluser@callm$2c.local << EOF
      	source /opt/ros/humble/setup.bash
        cd call_m_workspace/robot_ws_ros2
        colcon build --symlink-install
EOF
    elif [ $1 -eq 3 ]; then
      echo "Building REMOTE + JETSON (Ensure they are on the same network)"
      # Build client
      echo "Building REMOTE..."
      colcon build --symlink-install
      # Export packages to robot's computers
      echo ""
      bash export_NUC_JETSON_pkgs.sh "$@"
      echo ""
      # Build robot's computers
      echo ""
      echo "Building JETSON..."
      echo ""
      ssh jrluser@callm$2v.local << EOF
      	source /opt/ros/humble/setup.bash
        cd call_m_workspace/robot_ws_ros2
        colcon build --symlink-install
EOF
    elif [ $1 -eq 4 ]; then
      echo "Building REMOTE + NUC + JETSON (Ensure they are on the same network)"
      # Build client
      echo "Building REMOTE..."
      colcon build --symlink-install
      # Export packages to robot's computers
      echo ""
      bash export_NUC_JETSON_pkgs.sh "$@"
      echo ""
      # Build NUC
      echo ""
      echo "Building NUC..."
      echo ""
      ssh jrluser@callm$2c.local << EOF
      	source /opt/ros/humble/setup.bash
        cd call_m_workspace/robot_ws_ros2
        colcon build --symlink-install
EOF
      # Build JETSON
      echo ""
      echo "Building JETSON..."
      echo ""
      ssh jrluser@callm$2v.local << EOF
      	source /opt/ros/humble/setup.bash
        cd call_m_workspace/robot_ws_ros2
        colcon build --symlink-install
EOF
    fi
  else
    echo "Invalid parameter. Please provide an integer value among [1,2,3,4] or no parameter. (1:REMOTE 2:REMOTE+NUC, 3:REMOTE+JETSON, 4:REMOTE+NUC+JETSON) and 'robot_number identifier'"
    exit 1
  fi
fi

echo ""
echo "!!!Don't forget to source the workspace setup.bash file in .bashrc!!! (source /home/jrluser/call_m_workspace/robot_ws_ros2/install/setup.bash)"
echo "It might be needed to restart the Terminals for new installed nodes or files"
echo ""


