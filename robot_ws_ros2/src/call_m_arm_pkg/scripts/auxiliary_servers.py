#!/usr/bin/env python3
from controller_manager_msgs.srv import SwitchController
from std_srvs.srv import Trigger
from call_m_custom_msgs.srv import ArmActivateServo  # Import your custom service type
import rclpy
from rclpy.node import Node

class ArmControlService(Node):
    def __init__(self):
        super().__init__('arm_control_service')

        # Service clients
        self.switch_controller_client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        self.start_servo_client = self.create_client(Trigger, '/servo_node/start_servo')
        self.stop_servo_client = self.create_client(Trigger,'/servo_node/stop_servo')

        # Advertise the custom service
        self.arm_activate_servo_service = self.create_service(
            ArmActivateServo,
            'arm_activate_servo',
            self.handle_arm_activate_servo
        )

        self.get_logger().info("ArmActivateServo service is ready.")

    def handle_arm_activate_servo(self, request, response):
        # Check if the 'activate' field is True to proceed with the operation
        self.call_start_servo(request.activate)
        self.call_switch_controller(request.activate)
        # Response is always successful (fire-and-forget)
        response.success = True
        return response

    def call_switch_controller(self,is_servo):
        # Ensure the service is available
        if not self.switch_controller_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("SwitchController service not available")
            return

        # Create the request
        request = SwitchController.Request()
        if is_servo:
            request.activate_controllers = ['forward_position_controller']
            request.deactivate_controllers = ['scaled_joint_trajectory_controller']
        else:
            request.deactivate_controllers = ['forward_position_controller']
            request.activate_controllers = ['scaled_joint_trajectory_controller']
        request.strictness = SwitchController.Request.BEST_EFFORT

        # Send the request asynchronously (fire-and-forget)
        self.get_logger().info("Sending SwitchController request...")
        self.switch_controller_client.call_async(request)

    def call_start_servo(self,is_start):
        # Ensure the service is available
        if not self.start_servo_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("StartServo service not available")
            return
        if not self.stop_servo_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("StoptServo service not available")
            return
        # Create and send the request asynchronously (fire-and-forget)
        request = Trigger.Request()
        self.get_logger().info("Sending Servo request...")
        if is_start:
            self.start_servo_client.call_async(request)
        else:
            self.stop_servo_client.call_async(request)

def main(args=None):
    rclpy.init(args=args)
    node = ArmControlService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
