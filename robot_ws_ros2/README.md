# CNRS_AIST_Mobile_Shopping_Robot, ros2 workspace
https://github.com/Noceo200/CNRS_AIST_Mobile_Shopping_Robot.git
git@github.com:Noceo200/CNRS_AIST_Mobile_Shopping_Robot.git

## Details
### Workspace Tree
//add image tree

### ROS2 structure
//add image all packages links, let call_m_start_all with details to see how to launch the others packages.

## Configurations and customizations
### Launch
Parameters that can be changed in the following launch files:
* nav_type:(string): 
	- "none":(default): Do not use Nav2, just SLAM.
	- "on_fly": Use Nav2 without known map. Allow to navigate on a map generated with SLAM on fly.
	- "localize": Use Nav2 with a known map. Classic navigation with moving obstacles detection.

* nav_mode file:(string):
	- "diff": The robot will navigate like a differential robot
	- "omni": The robot will navigate like an holonomic robot
* nav_tree file
* map file
* slam file

### Navigation2
- behavior tree
- nav params
- maps

### SLAM
- slam

### Zedm cameras
- cam1
- cam2

//Add instructions to indicate what are the configurable files and what is their purpose
