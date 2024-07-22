#!/bin/bash

# Check if a parameter is provided
if [ $# -eq 0 ]; then
  echo "No parameter provided."
  echo "Valid parameters: (1:NUC, 2:JETSON, 3:NUC+JETSON)"
  echo ""
else
  # Check if the parameter is an integer between 1 and 3
  if [[ $1 =~ ^-?[0-9]+$ ]] && [ $1 -gt 0 ] && [ $1 -lt 4 ]; then
    echo "Integer parameter provided: $1"
    if [ $1 -eq 1 ]; then
      echo "Shutting down NUC (Ensure It is connected to this network)"
      ssh jrluser@callm01c.local << EOF
      	sudo shutdown now
EOF
    elif [ $1 -eq 2 ]; then
      echo "Shutting down JETSON (Ensure It is connected to this network)"
      ssh jrluser@callm01v.local << EOF
      	sudo shutdown now
EOF
    elif [ $1 -eq 3 ]; then
      echo "Shutting down NUC and JETSON (Ensure they are connected to this network)"
      ssh jrluser@callm01c.local << EOF
      	sudo shutdown now
EOF
      ssh jrluser@callm01v.local << EOF
      	sudo shutdown now
EOF
    fi
  else
    echo "Invalid parameter. Please provide an integer value among [1,2,3] or no parameter. (1:NUC, 2:JETSON, 3:NUC+JETSON)"
    exit 1
  fi
fi

echo ""
echo "Shutdown finished"
echo ""
