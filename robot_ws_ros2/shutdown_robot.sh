#!/bin/bash

# Check if at least 2 parameters are provided
if [ $# -lt 2 ]; then
  echo "Not enough parameters provided."
  echo "Valid parameters: (1:NUC, 2:JETSON, 3:NUC+JETSON) and 'robot_number identifier'"
  echo ""
  exit 1
fi

# Check if the parameter is an integer between 1 and 3
if [[ $1 =~ ^-?[0-9]+$ ]] && [ $1 -gt 0 ] && [ $1 -lt 4 ]; then
  echo "Integer parameter provided: $1"
  if [ $1 -eq 1 ]; then
    echo "Shutting down NUC (Ensure It is connected to this network)"
    ssh jrluser@callm$2c.local 'sudo shutdown now' || echo "Error: Failed to shut down NUC"
  elif [ $1 -eq 2 ]; then
    echo "Shutting down JETSON (Ensure It is connected to this network)"
    ssh jrluser@callm$2v.local 'sudo shutdown now' || echo "Error: Failed to shut down JETSON"
  elif [ $1 -eq 3 ]; then
    echo "Shutting down NUC and JETSON (Ensure they are connected to this network)"
    ssh jrluser@callm$2c.local 'sudo shutdown now' || echo "Error: Failed to shut down NUC"
    ssh jrluser@callm$2v.local 'sudo shutdown now' || echo "Error: Failed to shut down JETSON"
  fi
else
  echo "Invalid parameter. Please provide an integer value among [1,2,3]. (1:NUC, 2:JETSON, 3:NUC+JETSON), and 'robot_number identifier'"
  exit 1
fi

echo ""
echo "Shutdown finished"
echo ""

