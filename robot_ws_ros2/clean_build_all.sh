#!/bin/bash

# Check if a parameter is provided
if [ $# -eq 0 ]; then
  echo "No parameter provided. Building REMOTE computer only."
  echo "Valid parameters: (None:REMOTE 1:REMOTE+NUC, 2:REMOTE+JETSON, 3:REMOTE+NUC+JETSON)"
  echo ""
  # Build client only
  bash clean_build.sh
else
  # Check if the parameter is an integer between 1 and 3
  if [[ $1 =~ ^-?[0-9]+$ ]] && [ $1 -gt 0 ] && [ $1 -lt 4 ]; then
    echo "Integer parameter provided: $1"
    if [ $1 -eq 1 ]; then
      echo "Building REMOTE + NUC (Ensure they are on the same network)"
      # Build client
      echo "Building REMOTE..."
      bash clean_build.sh
      # Export packages to robot's computers
      echo ""
      bash export_NUC_JETSON_pkgs.sh "$@"
      echo ""
      # Build robot's computers
      echo ""
      echo "Building NUC..."
      echo ""
      ssh jrluser@callm01c.local << EOF
      	source /opt/ros/humble/setup.bash
        cd call_m_workspace/robot_ws_ros2
        bash clean_build.sh
EOF
    elif [ $1 -eq 2 ]; then
      echo "Building REMOTE + JETSON (Ensure they are on the same network)"
      # Build client
      echo "Building REMOTE..."
      bash clean_build.sh
      # Export packages to robot's computers
      echo ""
      bash export_NUC_JETSON_pkgs.sh "$@"
      echo ""
      # Build robot's computers
      echo ""
      echo "Building JETSON..."
      echo ""
      ssh jrluser@callm01v.local << EOF
      	source /opt/ros/humble/setup.bash
        cd call_m_workspace/robot_ws_ros2
        bash clean_build.sh
EOF
    elif [ $1 -eq 3 ]; then
      echo "Building REMOTE + NUC + JETSON (Ensure they are on the same network)"
      # Build client
      echo "Building REMOTE..."
      bash clean_build.sh
      # Export packages to robot's computers
      echo ""
      bash export_NUC_JETSON_pkgs.sh "$@"
      echo ""
      # Build NUC
      echo ""
      echo "Building NUC..."
      echo ""
      ssh jrluser@callm01c.local << EOF
      	source /opt/ros/humble/setup.bash
        cd call_m_workspace/robot_ws_ros2
        bash clean_build.sh
EOF
      # Build JETSON
      echo ""
      echo "Building JETSON..."
      echo ""
      ssh jrluser@callm01v.local << EOF
      	source /opt/ros/humble/setup.bash
        cd call_m_workspace/robot_ws_ros2
        bash clean_build.sh
EOF
    fi
  else
    echo "Invalid parameter. Please provide an integer value among [1,2,3] or no parameter. (None:REMOTE 1:REMOTE+NUC, 2:REMOTE+JETSON, 3:REMOTE+NUC+JETSON)"
    exit 1
  fi
fi

echo ""
echo "It might be needed to restart the Terminals for new installed nodes or files"
echo ""

