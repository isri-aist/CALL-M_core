# CALL-M_core, ros2 workspace
https://github.com/Noceo200/CALL-M_core.git

## Details
This directory contains all CALL-M ROS2 packages, the ones starting with "call_m..." can be modified as much as wanted. They are all the packages specifically related to the robot.

The other packages used by the robot are in the directory "included_external_packages", usually there are from other github, so modify them is possible but require to ask the git owner to accept the changements. Usually, the call_m packages are jut overwritting their configuration files with specific ones.

## Building workspace

* Classic build:
```
bash build_robot.sh $1 $2
```
This file allow to update packages and build the robot without the need to connect to each computers. Specific packages will be automatically send to NUC and JETSON through SSH.

Valid parameters: 
* $1: (1:REMOTE 2:REMOTE+NUC, 3:REMOTE+JETSON, 4:REMOTE+NUC+JETSON) 
* $2: 'robot_number identifier'.

The'robot_number identifier' X will be used like this: callmXc (NUC name), callmXv (JETSON name)

* Clean build:
```
bash clean_build_robot.sh $1 $2
```
This file is similar to a classic build, but first delete the directories 'build', 'install' and 'log' of the ROS2 workspaces.

* Using terminal on each computers
```
colcon build --symlink-install
```

## Runing worspace
### Simulation
```
ros2 launch call_m_start_all call_m_simulation.launch.py mode:=<COMPLETE or API> nav_type:=<none, on_fly or localize>
```

### Real Robot
* execute on the NUC:
```
ros2 launch call_m_start_all call_m_hardware.launch.py version:=NUC mode:=<DRIVERS or API> nav_type:=<none, on_fly or localize>
```
* execute on the JETSON:
```
ros2 launch call_m_start_all call_m_hardware.launch.py version:=JETSON
```
* execute on the terminal (your Laptop):
```
ros2 launch call_m_start_all call_m_hardware.launch.py version:=CLIENT mode:=<DRIVERS or API> nav_type:=<none, on_fly or localize>
```
* For the real robot to move, the TriOrb needs to be activated, so you can follow the Joystick instructions or do it manually by executing those commands, one by one in the same order:
```
ros2 lifecycle set /triorb configure
ros2 lifecycle set /triorb activate
```

* Check PARAMETERS in the [main README File](../README.md)

### ROS2 structure
// Update draw.io
