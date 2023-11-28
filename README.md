# CNRS_AIST_Mobile_Shopping_Robot

## Dependancies: 
- xterm (to open several terminal automaticcally with ros2 launch files)
```
sudo apt install xterm
```
- navigation2 (ROS Framework for robot navigation)
https://navigation.ros.org/getting_started/index.html

## Launch project
For teleoperate real robot:
```
ros2 launch c_pkg bot_teleoperate_launch.py
```
For Nav2 simulation launch:
```
colcon build ; ros2 launch nav2_simu display.launch.py
```
to publish map=>odom TF needed by Nav2 plugin:
```
ros2 launch slam_toolbox online_async_launch.py
```
to publish costmaps and footprints needed for all other Nav2 plugins, and also launch all Nav2 plugins configured in nav2_params.yaml:
```
ros2 launch nav2_bringup navigation_launch.py params_file:=<full/path/to/config/nav2_params.yaml>
```
```
ros2 launch nav2_bringup navigation_launch.py params_file:=/home/jrlintern/Desktop/CNRS_AIST_Work/CNRS_AIST_Mobile_Shopping_Robot/robot_ws_ros2/src/nav2_simu/config/nav2_params.yaml
```


