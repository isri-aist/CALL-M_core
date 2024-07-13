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
----
## Quick start with joystick
### Install py_twist
~~~
 $ cd ~/ros2_ws/src
 $ git cloen https://github.com/haraisao/joy_twist
 $ cd ~/ros2_ws
 $ source /opt/ros/humble/setup.bash
 $ colcon build --symlink-install
 $ source install/setup.bash
~~~

### start by using lanch
~~~
 $ ros2 launch triorb_ros2 triorb_launch.py
~~~

### Operation
~~~
START Button： Send 'configure' message to '/triorb', lifecycle state of '/triorb' transits to inactive state.
　　　　　　　 This operation let '/triorb' initialize topic ports, parameters etc.

BACK Button： Send 'shutdown' message to '/triorb', lifecycle state of '/triorb' transits to finalized state.
              Terminate all programs.


LB Button：　Send 'activate' message to '/triorb', lifecycle state of '/triorb' transits to active state.
　　　　In only the active state, all topic ports and communication to the robot is avvaiable. 

RB Button：　Send 'deactivate' message to '/triorb', lifecycle state of '/triorb' transits to inactive state.
　　　　Stop the robot motion and motors, terminate all communication with external programs. 

Left analogue stick： Send translation velocitise(X, Y) to the robot

Right analogue stick(Horizontal motion): Send rotation velocity to the robot
~~~

