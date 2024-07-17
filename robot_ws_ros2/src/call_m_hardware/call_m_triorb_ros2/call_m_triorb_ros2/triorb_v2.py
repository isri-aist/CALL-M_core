#
# Copyright(C) 2024 RT-Coorp.
#  This program release with Apache License version 2.0
#
import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State, Publisher

from typing import Optional

import numpy as np
import quaternion

from rclpy.timer import Timer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import triorb_core

import std_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg

class TriOrb(LifecycleNode):
    def __init__(self):
        super().__init__('triorb')
        self.get_logger().info('Ready, Waiting to be configured...')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_configure')
        try:
          self.declare_parameter('triorb_port', '/dev/ttyACM0')
          self.declare_parameter('odom_duration', 1.0)
          self.declare_parameter('watchdog_timeout', 10.0)
          self.declare_parameter('world_frame', 'world')
          self.declare_parameter('robot_frame', 'triorb')
          self.declare_parameter('debug', False)
        except:
          pass

        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
      
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_activate() is called')
        # Activation code here
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_deactivate() is called')
        # Deactivation code here
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_cleanup() is called')
        # Cleanup code here
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_shutdown() is called')
        # Shutdown code here
        return TransitionCallbackReturn.SUCCESS

def main(args=None):
    rclpy.init(args=args)
    node = TriOrb()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()