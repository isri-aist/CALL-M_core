# triorb_ros2
This repoditory contains a simple control node for TriOrb.

## Requirements
- ROS2 Humble
- Lifecycle of ROS2 python (ros-humble-lifecycle-py via apt)
- numpy-quaternion (install via pip)
- triorb-core (download from Github)
- pyserial (python3-serial via apt)
~~~
 $ sudo apt install python3-serial python3-pip ros-humble-lifecycle-py
 $ sudo pip install numpy-quaternion
~~~

## Quick Setup
~~~
 $ mkdir -p ~/ros2_ws/src
 $ cd ~/ros2_ws/src
 $ git clone https://github.com/TriOrb-Inc/triorb-core
 $ git cloen https://github.com/haraisao/triorb_ros2
 $ cd ~/ros2_ws
 $ source /opt/ros/humble/setup.bash
 $ colcon build --symlink-install
 $ source install/setup.bash
~~~

Connect to the controll ECU on TriOrb.
Check serial port name for TriOrb, default is '/dev/ttyACM0'
Change access mode of TriOrb port to r/w.
Run TriOrb node
~~~
 $ ros2 run triorb_ros2 trorb --ros-args -p "triorb_port:=/dev/ttyACM0"
~~~
Control the node's lifecycle state transition.

1.Configure
~~~
 $ ros2 lifecycle set /triorb configure
~~~

2.Activate
~~~
 $ ros2 lifecycle set /triorb activate
~~~

3.Deactivate
~~~
 $ ros2 lifecycle set /triorb deactivate
~~~

4.Shutdown
~~~
 $ ros2 lifecycle set /triorb shutdown
~~~