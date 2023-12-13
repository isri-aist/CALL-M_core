import subprocess
import keyboard
import time

# Construct the absolute paths to your launch files
master_launch =  ['call_m_supervisor', 'master.launch.py']
simu_launch = ['call_m_simulation', 'simu.launch.py']
display_launch = ['call_m_monitor', 'display.launch.py']
teleop_launch =  ['call_m_teleoperation', 'teleop.launch.py']

# Command to launch xterm and execute ROS 2 launch commands
cmd = ['xterm', '-fn', 'xft:fixed:size=12', '-geometry', '60x20', '-e', 'ros2', 'launch']

# Execute the ROS 2 launch commands using subprocess
p1 = subprocess.Popen(cmd + master_launch)
time.sleep(1)
p2 = subprocess.Popen(cmd + simu_launch)
time.sleep(1)
p3 = subprocess.Popen(cmd + display_launch)
time.sleep(1)
p4 = subprocess.Popen(cmd + teleop_launch)

# Wait for key press 'k' to terminate all processes
keyboard.wait('k')

# Terminate all processes
p1.terminate()
p2.terminate()
p3.terminate()
p4.terminate()