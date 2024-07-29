#!/bin/bash

# Check if at least 2 parameters are provided
if [ $# -lt 2 ]; then
  printf "\nNot enough parameters provided."
  printf "Valid parameters: \n1:REMOTE only \n2:NUC only \n3:JETSON only \n4:REMOTE+NUC \n5:REMOTE+JETSON \n6:NUC+JETSON \n7:REMOTE+NUC+JETSON \nAnd 'robot_number identifier'\n\n"
  exit 1
fi

# Check if the parameter is an integer between 1 and 7
if [[ $1 =~ ^-?[0-9]+$ ]] && [ $1 -gt 0 ] && [ $1 -lt 8 ]; then
  printf "Integer parameter provided: $1"
  if [ $1 -eq 1 ]; then
    printf "Running REMOTE..."
    ros2 launch call_m_start_all call_m_start_CLIENT.launch.py
  elif [ $1 -eq 2 ]; then
    printf "Running NUC... (Ensure It is connected to this network)"
    ssh jrluser@callm$2c.local 'source ~/.bashrc; ros2 launch call_m_start_all call_m_start_NUC.launch.py' || printf "Error: Failed to run NUC" && exit
  elif [ $1 -eq 3 ]; then
    printf "Running JETSON... (Ensure It is connected to this network)"
    ssh jrluser@callm$2v.local 'ros2 launch call_m_start_all call_m_start_JETSON.launch.py' || printf "Error: Failed to run JETSON" && exit
  elif [ $1 -eq 4 ]; then
    printf "Running NUC... (Ensure It is connected to this network)"
    ssh jrluser@callm$2c.local 'ros2 launch call_m_start_all call_m_start_NUC.launch.py' || printf "Error: Failed to run NUC" && exit
    printf "Running REMOTE..."
    ros2 launch call_m_start_all call_m_start_CLIENT.launch.py
  elif [ $1 -eq 5 ]; then
    printf "Running JETSON... (Ensure It is connected to this network)"
    ssh jrluser@callm$2v.local 'ros2 launch call_m_start_all call_m_start_JETSON.launch.py' || printf "Error: Failed to run JETSON" && exit
    printf "Running REMOTE..."
    ros2 launch call_m_start_all call_m_start_CLIENT.launch.py
  elif [ $1 -eq 6 ]; then
    printf "Running NUC... (Ensure It is connected to this network)"
    ssh jrluser@callm$2c.local 'ros2 launch call_m_start_all call_m_start_NUC.launch.py' || printf "Error: Failed to run NUC" && exit
    printf "Running JETSON... (Ensure It is connected to this network)"
    ssh jrluser@callm$2v.local 'ros2 launch call_m_start_all call_m_start_JETSON.launch.py' || printf "Error: Failed to run JETSON" && exit
  elif [ $1 -eq 7 ]; then
    printf "Running NUC... (Ensure It is connected to this network)"
    ssh jrluser@callm$2c.local 'ros2 launch call_m_start_all call_m_start_NUC.launch.py' || printf "Error: Failed to run NUC" && exit
    printf "Running JETSON... (Ensure It is connected to this network)"
    ssh jrluser@callm$2v.local 'ros2 launch call_m_start_all call_m_start_JETSON.launch.py' || printf "Error: Failed to run JETSON" && exit
    printf "Running REMOTE..."
    ros2 launch call_m_start_all call_m_start_CLIENT.launch.py
  fi
else
  printf "Invalid parameter. Please provide an integer value among [1,2,3,4,5,6,7]. \n1:REMOTE only \n2:NUC only \n3:JETSON only \n4:REMOTE+NUC \n5:REMOTE+JETSON \n6:NUC+JETSON \n7:REMOTE+NUC+JETSON \nAnd 'robot_number identifier'"
  exit 1
fi

printf "\n\nRun finished\n\n"

