#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Empty  # Import the Empty service
from pynput import keyboard

class PointCloudRepublisher(Node):
    def __init__(self):
        super().__init__('point_cloud_republisher')
        
        # Create a publisher for the new topic
        self.publisher = self.create_publisher(PointCloud2, '/call_m/camera/points', 10)

        # Create a subscription to the original topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/points',
            self.listener_callback,
            10  # QoS history depth
        )
        self.subscription  # prevent unused variable warning
        
        # Create a service client for the /clear_octomap service
        self.clear_octomap_client = self.create_client(Empty, '/clear_octomap')

        # Flag to control publishing
        self.is_publishing = False

        # Start keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def listener_callback(self, msg):
        # Publish the received message to the new topic if publishing is enabled
        if self.is_publishing:
            self.publisher.publish(msg)

    def on_press(self, key):
        try:
            if key.char == 'p':  # Press 'p' to toggle publishing
                self.is_publishing = not self.is_publishing
                state = "enabled" if self.is_publishing else "disabled"
                self.get_logger().info(f'Publishing {state}')

                # Clear the OctoMap if publishing is disabled
                if not self.is_publishing:
                    self.call_clear_octomap_service()

        except AttributeError:
            pass  # Handle special keys if needed

    def call_clear_octomap_service(self):
        # Wait until the service is available
        self.clear_octomap_client.wait_for_service()
        request = Empty.Request()  # Create a request object
        future = self.clear_octomap_client.call_async(request)

        # Handle the future response
        future.add_done_callback(self.clear_octomap_response)

    def clear_octomap_response(self, future):
        try:
            future.result()  # We can ignore the response since Empty service doesn't return anything
            self.get_logger().info('Cleared the OctoMap visualization in RViz.')
        except Exception as e:
            self.get_logger().error(f'Failed to call clear_octomap service: {e}')

def main(args=None):
    rclpy.init(args=args)
    republisher = PointCloudRepublisher()
    rclpy.spin(republisher)
    republisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
