#!/usr/bin/env python3

from call_m_arm_pkg import robotiq_gripper_class

ip = "192.168.1.4"

def log_info(gripper):
    print(f"Pos: {str(gripper.get_current_position()): >3}  "
          f"Open: {gripper.is_open(): <2}  "
          f"Closed: {gripper.is_closed(): <2}  ")

print("Creating gripper...")
gripper = robotiq_gripper_class.RobotiqGripper()
print("Connecting to gripper...")
gripper.connect(ip, 63352)
print("Opening gripper")
gripper.get_open_position()
print("Closing gripper")
gripper.get_closed_position()
print("Moving gripper")
gripper.move_and_wait_for_pos(228,10,10)