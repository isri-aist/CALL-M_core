#
# Dummy 
import numpy as np
import quaternion

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import tf_transformations
import tf2_ros

from geometry_msgs.msg import TransformStamped

import time

#
#
class Pose:
  def __init__(self):
    self.x=0.0
    self.y=0.0
    self.w=0.0
  def set(self, x=0.0, y=0.0, w=0.0):
    self.x=x
    self.y=y
    self.w=w
    return
  def __str__(self):
    return "x:%f, y:%f, w:%f" % (self.x, self.y, self.w)

#
#
class robot:
  def __init__(self, port=None, node=None):
    self.port=port
    self.vx=0
    self.vy=0
    self.vw=0
    self.current_pos = Pose()
    self.interval=0.2
    self.node=node
    self.timer_ = None
    self.timer_callback_group_ = None
    self.print_info("Dummy robot: port = %s" % self.port)
    self.tf_broadcaster = tf2_ros.TransformBroadcaster(self.node)
    self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self.node)

  def __del__(self):
    self.destroy_timer()
    return

  def create_timer(self):
    self.timer_callback_group_ = MutuallyExclusiveCallbackGroup()
    self.timer_ = self.node.create_timer(self.interval, self.timer_callback,
                      callback_group = self.timer_callback_group_)
    return

  def destroy_timer(self):
    if self.timer_ is not None:
      self.node.destroy_timer(self.timer_)
      self.timer_ = None
      self.timer_callback_group_ = None
    return

  def wakeup(self):
    self.broadcast_static_tf()
    self.create_timer()
    return

  def sleep(self):
    self.destroy_timer()
    return
    
  def brake(self):
    self.vx=0
    self.vy=0
    self.vw=0
    return
    
  def get_pos(self):
    self.print_info(str(self.current_pos))
    return [self.current_pos]

  def join(self):
    return
  
  def print_info(self, *args):
    self.node.get_logger().info(*args)
    return

  def set_pos_absolute(self, x, y, w):
    self.current_pos.x = x
    self.current_pos.y = y
    self.current_pos.w = w
    return

  def set_vel_absolute(self, vx, vy, vw):
    self.vx = vx
    self.vy = vy
    self.vw = vw
    return

  def timer_callback(self):
    self.current_pos.x += self.vx * self.interval 
    self.current_pos.y += self.vy * self.interval
    self.current_pos.w += self.vw * self.interval
    if self.current_pos.w > np.pi: self.current_pos.w -= np.pi * 2
    elif self.current_pos.w < -np.pi: self.current_pos.w += np.pi * 2

    self.broadcast_tf()
    return

  def broadcast_tf(self, frame_id="triorb"):
    trans_ = TransformStamped()
    trans_.header.stamp = self.node.get_clock().now().to_msg()
    trans_.header.frame_id = 'world'
    trans_.child_frame_id = frame_id
    trans_.transform.translation.x = self.current_pos.x
    trans_.transform.translation.y = self.current_pos.y
    trans_.transform.translation.z = 0.2

    quat = quaternion.from_euler_angles(0, 0, self.current_pos.w)
    trans_.transform.rotation.x = quat.x
    trans_.transform.rotation.y = quat.y
    trans_.transform.rotation.z = quat.z
    trans_.transform.rotation.w = quat.w
    self.tf_broadcaster.sendTransform(trans_)
    return

  def broadcast_static_tf(self, frame_id="world"):
    trans_ = TransformStamped()
    trans_.header.stamp = self.node.get_clock().now().to_msg()
    trans_.header.frame_id = ''
    trans_.child_frame_id = frame_id
    trans_.transform.translation.x = 0.0
    trans_.transform.translation.y = 0.0
    trans_.transform.translation.z = 0.0

    quat = quaternion.from_euler_angles(0, 0, 0.0)
    trans_.transform.rotation.x = quat.x
    trans_.transform.rotation.y = quat.y
    trans_.transform.rotation.z = quat.z
    trans_.transform.rotation.w = quat.w
    self.tf_static_broadcaster.sendTransform(trans_)
    return
