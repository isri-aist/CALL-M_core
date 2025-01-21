#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
import math

class EndEffectorPositionNode(Node):
    def __init__(self):
        super().__init__('end_effector_position_node')
        
        # Initialize the TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Set a timer to periodically check the transform
        self.timer = self.create_timer(1.0, self.get_end_effector_position)

    def quaternion_to_euler(self, x, y, z, w):
        """
        Convert quaternion (x, y, z, w) to roll, pitch, and yaw (RPY) angles in radians.
        """
        # Roll (x-axis rotation)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        # Pitch (y-axis rotation)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        # Yaw (z-axis rotation)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    def get_end_effector_position(self):
        try:
            # Look up the transform from 'base' to 'tool0'
            transform = self.tf_buffer.lookup_transform('base', 'tool0', rclpy.time.Time())
            
            # Extract translation (position) and rotation (orientation)
            position = transform.transform.translation
            orientation = transform.transform.rotation
            
            # Convert quaternion to RPY
            roll, pitch, yaw = self.quaternion_to_euler(
                orientation.x, orientation.y, orientation.z, orientation.w
            )
            
            self.get_logger().info(f"Position of tool0 relative to base:\n"
                                   f"x: {position.x}\n"
                                   f"y: {position.y}\n"
                                   f"z: {position.z}\n"
                                   f"Orientation (quaternion): x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}\n"
                                   f"Orientation (RPY): roll={math.degrees(roll)}, pitch={math.degrees(pitch)}, yaw={math.degrees(yaw)}")
        except Exception as e:
            self.get_logger().error(f"Could not get transform: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = EndEffectorPositionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
