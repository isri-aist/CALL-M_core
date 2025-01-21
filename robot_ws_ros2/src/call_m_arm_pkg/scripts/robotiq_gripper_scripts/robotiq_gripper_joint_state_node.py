#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from call_m_arm_pkg import robotiq_gripper_class

# Maximum gripper aperture from the joint perspective 
MAX_APERTURE_VALUE_IN_BYTE = 228

class RobotJointStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, '/call_m/joint_states', 10)

        # Subscribe to the arm's joint states
        self.arm_joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',  # Replace with the actual topic name if different
            self.assemble_joint_states_callback,
            50
        )

        self.gripper = robotiq_gripper_class.RobotiqGripper()
        self.robotiq_ip = "192.168.1.4"
        self.robotiq_port = 63352
        self.init_gripper()

        # Define joint names
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
            'robotiq_85_left_knuckle_joint'  # Gripper joint
        ]

        # Initialize joint positions
        self.joint_positions = [0.0] * len(self.joint_names)

        # Initialize gripper position
        self.robotiq_position = 0

        # The maximum frequency at which the controller operates is 200Hz, so operate the gripper at 100Hz
        joint_state_period = 0.01  # seconds
        self.create_timer(joint_state_period, self.publish_joint_states)

    def assemble_joint_states_callback(self, msg):
        # Log the received joint states
        # self.get_logger().info(f"Received joint states: {msg.name}, {msg.position}")

        # Update the arm joint positions
        for i, joint_name in enumerate(msg.name):
            if joint_name in self.joint_names:
                index = self.joint_names.index(joint_name)
                self.joint_positions[index] = msg.position[i]

        # Always read the latest gripper state when arm states are received
        self.get_robotiq_joint_state()

    def publish_joint_states(self):
        # Update finger joint position
        self.joint_positions[-1] = self.map_position_to_range(self.robotiq_position)

        # Prepare JointState message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = self.joint_positions

        # Log the published message for debugging
        # self.get_logger().info(f'Publishing joint states: {joint_state_msg.name}, {joint_state_msg.position}')

        # Publish the message
        self.publisher_.publish(joint_state_msg)

    def init_gripper(self):
        self.gripper.connect(self.robotiq_ip, self.robotiq_port)

    def get_robotiq_joint_state(self):
        self.robotiq_position = self.gripper.get_current_position()

    def map_position_to_range(self, position):
        # Linear mapping from position [0, 255] to [0, 0.7]
        return (position / MAX_APERTURE_VALUE_IN_BYTE) * 0.8

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = RobotJointStatePublisher()
    rclpy.spin(joint_state_publisher)
    joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
