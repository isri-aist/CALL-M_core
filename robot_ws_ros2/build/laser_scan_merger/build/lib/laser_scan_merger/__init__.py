import rclpy
from sensor_msgs.msg import LaserScan
from rclpy.node import Node

class LaserScanFusionNode(Node):
    def __init__(self):
        super().__init__('laser_scan_fusion_node')
        self.subscription1 = self.create_subscription(LaserScan, 'scan1', self.scan1_callback, 10)
        self.subscription2 = self.create_subscription(LaserScan, 'scan2', self.scan2_callback, 10)
        self.publisher = self.create_publisher(LaserScan, 'fused_scan', 10)

        # Initialize variables to store laser scan data
        self.scan1_data = None
        self.scan2_data = None

    def scan1_callback(self, msg):
        self.scan1_data = msg

    def scan2_callback(self, msg):
        self.scan2_data = msg
        self.fuse_and_publish()

    def fuse_and_publish(self):
        if self.scan1_data is not None and self.scan2_data is not None:
            # Implement your fusion logic here (e.g., averaging, minimum/maximum, weighted fusion)
            fused_data = self.average_scans(self.scan1_data, self.scan2_data)

            # Publish the fused laser scan
            self.publisher.publish(fused_data)

    def average_scans(self, scan1, scan2):
        # Example: Average the range values at each point
        fused_data = LaserScan()
        fused_data.header = scan1.header  # Assuming headers are the same
        fused_data.ranges = [(r1 + r2) / 2.0 for r1, r2 in zip(scan1.ranges, scan2.ranges)]
        # You may need to handle other fields such as intensities, angle_min, angle_max, etc.

        return fused_data

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanFusionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

