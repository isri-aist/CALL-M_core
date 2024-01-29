# CNRS_AIST_Mobile_Shopping_Robot, ros2 workspace
https://github.com/Noceo200/CNRS_AIST_Mobile_Shopping_Robot.git
git@github.com:Noceo200/CNRS_AIST_Mobile_Shopping_Robot.git

## Configurations and customizations

Parameters that can be changed in the following launch files:
* nav_type:(string): 
	- "none":(default): Do not use Nav2, just SLAM.
	- "on_fly": Use Nav2 without known map. Allow to navigate on a map generated with SLAM on fly.
	- "localize": Use Nav2 with a known map. Classic navigation with moving obstacles detection.
* nav_mode:(string):
	- "diff": The robot will navigate like a differential robot
	- "omni": The robot will navigate like an holonomic robot


