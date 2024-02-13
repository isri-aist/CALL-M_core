#
# Copyright(C) 2024 RT-Coorp.
#  This program release with Apache License version 2.0
#
from typing import Optional

import numpy as np
import quaternion

import rclpy
from rclpy.lifecycle import Node
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import triorb_core

import std_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg

class TriOrb(Node):
  def __init__(self, node_name, **kargs):
    self._pub: Optional[Publisher] = None
    self._timer: Optional[Timer] = None
    self._watchdog: Optional[Timer] = None
    super().__init__(node_name, **kargs)

  ######### Lifecycle callbacks
  #
  # Configure
  def on_configure(self, state: State) -> TransitionCallbackReturn:
    self.get_logger().info('on_configure')

    self.declare_parameter('triorb_port', '/dev/ttyACM0')
    self.declare_parameter('odom_duration', 1.0)
    self.declare_parameter('watchdog_timeout', 10.0)
    self.declare_parameter('world_frame', 'world')
    self.declare_parameter('robot_frame', 'triorb')
    self.declare_parameter('debug', False)

    self.debug = self.get_parameter('debug').get_parameter_value().bool_value
   
    self._odom = nav_msgs.msg.Odometry()
    self._odom.header.frame_id = self.get_parameter('world_frame').get_parameter_value().string_value
    self._odom.child_frame_id = self.get_parameter('robot_frame').get_parameter_value().string_value
    self._odom.pose.pose.position.x = 0.0
    self._odom.pose.pose.position.y = 0.0
    self._odom.pose.pose.position.z = 0.0
    self._odom.pose.pose.orientation.x = 0.0
    self._odom.pose.pose.orientation.y = 0.0
    self._odom.pose.pose.orientation.z = 0.0
    self._odom.pose.pose.orientation.w = 1.0
    self.vx = 0.0
    self.vy = 0.0
    self.vw = 0.0
    self.port = self.get_parameter('triorb_port').get_parameter_value().string_value

    self.get_logger().info('open port: %s' % self.port)
    self._vehicle = triorb_core.robot(self.port)

    self._pub = self.create_lifecycle_publisher(nav_msgs.msg.Odometry, '/odom',  10)
    self._odom_duration = self.get_parameter('odom_duration').get_parameter_value().double_value
    #self._timer_callback = MutuallyExclusiveCallbackGroup()
    self._timer_callback = None
    self._timer = self.create_timer(self._odom_duration, self.publish_odometry, callback_group=self._timer_callback)

    self._watchdog_timeout = self.get_parameter('watchdog_timeout').get_parameter_value().double_value
    #self._watchdog_callback = MutuallyExclusiveCallbackGroup()
    self._watchdog_callback = None
    self._watchdog = self.create_timer(self._watchdog_timeout, self.cb_watchdog, callback_group=self._watchdog_callback)
    self._watchdog.cancel()

    self._sub_vel = self.create_subscription(geometry_msgs.msg.Twist, '/cmd_vel', self.cb_cmd_velocity, 10)
    return TransitionCallbackReturn.SUCCESS
  #
  # Activate
  def on_activate(self, state: State) -> TransitionCallbackReturn:
    self.get_logger().info('on_activate')
    self._vehicle.wakeup()
    self._watchdog.reset()
    self._pub.on_activate(state)

    return super().on_activate(state)
  #
  # Deactivate
  def on_deactivate(self, state: State) -> TransitionCallbackReturn:
    self.get_logger().info('on_deactivate')
    self._vehicle.sleep()
    self._watchdog.cancel()
    self._pub.on_deactivate(state)
    return super().on_deactivate(state)
  #
  # Cleanup
  def on_cleanup(self, state: State) -> TransitionCallbackReturn:
    self.stop()
    self.get_logger().info('on_cleanup')
    self.destroy_timer(self._timer)
    self.destroy_publisher(self._pub)
    del self._vehicle
    self._vehicle = None
    return TransitionCallbackReturn.SUCCESS
  #
  # Shutdown
  def on_shutdown(self, state: State) -> TransitionCallbackReturn:
    self.get_logger().info('on_shutdown') 
    self.stop()
    self.destroy_timer(self._timer)
    self.destroy_publisher(self._pub)
    del self._vehicle
    self._vehicle = None
    
    return TransitionCallbackReturn.SUCCESS

  #################### Callback functions and basic functions
  #
  # Callback of timer 
  def publish_odometry(self):
    if self._pub is not None and self._pub.is_activated:
      
      pose = self._vehicle.get_pos()[0]
      if pose is None and self.debug:
        import random
        pose = triorb_core.core_types.TriOrbDrive3Pose()
        pose.x = random.random()
        pose.w = random.random()
      self._odom.pose.pose.position.x = pose.x
      self._odom.pose.pose.position.y = pose.y

      quat=quaternion.from_euler_angles(0,0,pose.w)
      self._odom.pose.pose.orientation.x = quat.x
      self._odom.pose.pose.orientation.y = quat.y
      self._odom.pose.pose.orientation.z = quat.z
      self._odom.pose.pose.orientation.w = quat.w

      self._odom.twist.twist.linear.x = self.vx
      self._odom.twist.twist.linear.y = self.vy
      self._odom.twist.twist.angular.z = self.vw
      self._odom.header.stamp = self.get_clock().now().to_msg()
      self._pub.publish(self._odom)
    return
  #
  # Callback of watchdog timer
  def cb_watchdog(self):
    self.get_logger().info('watchdog')
    self.stop()
    self._watchdog.cancel()
    return
  #
  # Stop movement
  def stop(self):
    try:
      self._vehicle.brake()
      self._vehicle.join()
    except:
      pass
    self.vx = 0.0
    self.vy = 0.0
    self.vw = 0.0
    return
  #
  # Callback of subscriber
  def cb_cmd_velocity(self, msg):
    self.vx = msg.linear.x
    self.vy = msg.linear.y
    self.vw = msg.angular.z
    self._vehicle.set_vel_absolute(self.vx, self.vy, self.vw)
    self._watchdog.reset()
    return
  

#
#  M A I N
def main():
  rclpy.init()
  node = TriOrb('triorb')
  executor = MultiThreadedExecutor()
  executor.add_node(node)

  try:
    node.get_logger().info('Start control')
    executor.spin()
  except KeyboardInterrupt:
    node.stop()
    node.get_logger().info('Keyboad inperrupt, shutting down.')
  node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
    main()
