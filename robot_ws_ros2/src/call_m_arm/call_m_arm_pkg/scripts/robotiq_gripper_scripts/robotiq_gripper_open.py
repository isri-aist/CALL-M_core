#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from call_m_custom_msgs.srv import GripperAction
from call_m_arm_pkg import robotiq_gripper_class


class GripperServiceNode(Node):
    def __init__(self):
        super().__init__('gripper_service_node')

        # Initialize the gripper
        self.gripper = robotiq_gripper_class.RobotiqGripper()
        self.ip = "192.168.1.4"  # Replace with your gripper's IP address
        self.port = 63352  # Default port

        self.get_logger().info("Connecting to gripper...")
        self.gripper.connect(self.ip, self.port)

        # Advertise the service
        self.srv = self.create_service(GripperAction, 'gripper_command', self.handle_gripper_command)
        self.get_logger().info("Gripper service ready to receive commands.")

    def handle_gripper_command(self, request, response):
        try:
            if request.close:
                self.get_logger().info("Closing the gripper...")
                self.gripper.get_closed_position()
                self.gripper.move_and_wait_for_pos(228, 10, 10)
            else:
                self.get_logger().info("Opening the gripper...")
                self.gripper.get_open_position()
                self.gripper.move_and_wait_for_pos(0, 10, 10)

            # Log current state
            current_position = self.gripper.get_current_position()
            self.get_logger().info(f"Gripper Position: {current_position}")

            response.success = True
        except Exception as e:
            self.get_logger().error(f"Failed to process command: {e}")
            response.success = False

        return response


def main(args=None):
    rclpy.init(args=args)
    node = GripperServiceNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
