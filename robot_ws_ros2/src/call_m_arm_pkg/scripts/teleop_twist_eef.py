#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import termios
import sys
import tty

# Instructions for keyboard controls
msg = """
Control the End-Effector using the keyboard:
--------------------------------------------
Moving the End-Effector:
   w: Forward (X+)
   s: Backward (X-)
   a: Left (Y+)
   d: Right (Y-)
   r: Up (Z+)
   f: Down (Z-)

Rotating the End-Effector:
   i: Roll+   , k: Roll-
   j: Pitch+  , l: Pitch-
   u: Yaw+    , o: Yaw-

Adjusting velocities:
   y: Increase linear velocity
   h: Decrease linear velocity
   t: Increase angular velocity
   g: Decrease angular velocity

q: Quit
--------------------------------------------
"""

class TerminalSettings:
    """Context manager for safely managing terminal settings."""
    def __enter__(self):
        self.old_attrs = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_attrs)

class TeleopTwistEEF(Node):
    def __init__(self):
        super().__init__('teleop_twist_eef')
        self.publisher_ = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.rate = self.create_rate(10)
        self.twist = TwistStamped()

        # Initial linear and angular velocities
        self.linear_velocity = 0.1
        self.angular_velocity = 0.1

        self.get_logger().info("TeleopTwistEEF Node Started")
        self.print_instructions()

    def print_instructions(self):
        print(msg)

    def run(self):
        try:
            with TerminalSettings():
                while rclpy.ok():
                    key = sys.stdin.read(1)
                    if key in ('q', 'Q'):  # Quit
                        self.get_logger().info("Exiting...")
                        break

                    # Reset velocities
                    self.twist.twist.linear.x = 0.0
                    self.twist.twist.linear.y = 0.0
                    self.twist.twist.linear.z = 0.0
                    self.twist.twist.angular.x = 0.0
                    self.twist.twist.angular.y = 0.0
                    self.twist.twist.angular.z = 0.0

                    # Map keys to actions
                    if key == 'w':
                        self.twist.twist.linear.x = self.linear_velocity
                    elif key == 's':
                        self.twist.twist.linear.x = -self.linear_velocity
                    elif key == 'a':
                        self.twist.twist.linear.y = self.linear_velocity
                    elif key == 'd':
                        self.twist.twist.linear.y = -self.linear_velocity
                    elif key == 'r':
                        self.twist.twist.linear.z = self.linear_velocity
                    elif key == 'f':
                        self.twist.twist.linear.z = -self.linear_velocity
                    elif key == 'i':
                        self.twist.twist.angular.x = self.angular_velocity
                    elif key == 'k':
                        self.twist.twist.angular.x = -self.angular_velocity
                    elif key == 'j':
                        self.twist.twist.angular.y = self.angular_velocity
                    elif key == 'l':
                        self.twist.twist.angular.y = -self.angular_velocity
                    elif key == 'u':
                        self.twist.twist.angular.z = self.angular_velocity
                    elif key == 'o':
                        self.twist.twist.angular.z = -self.angular_velocity

                    # Adjust linear velocity
                    elif key == 'y':
                        self.linear_velocity += 0.01
                        self.get_logger().info(f"Linear velocity increased: {self.linear_velocity}")
                    elif key == 'h':
                        self.linear_velocity = max(0.01, self.linear_velocity - 0.01)
                        self.get_logger().info(f"Linear velocity decreased: {self.linear_velocity}")

                    # Adjust angular velocity
                    elif key == 't':
                        self.angular_velocity += 0.01
                        self.get_logger().info(f"Angular velocity increased: {self.angular_velocity}")
                    elif key == 'g':
                        self.angular_velocity = max(0.01, self.angular_velocity - 0.01)
                        self.get_logger().info(f"Angular velocity decreased: {self.angular_velocity}")

                    # Set the timestamp and publish the twist command
                    self.twist.header.stamp = self.get_clock().now().to_msg()
                    self.twist.header.frame_id = "base"  # Change if necessary
                    self.publisher_.publish(self.twist)

        except Exception as e:
            self.get_logger().error(f"Exception in teleop node: {e}")
        finally:
            self.get_logger().info("Teleop Node Shutdown")

def main(args=None):
    rclpy.init(args=args)
    node = TeleopTwistEEF()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
