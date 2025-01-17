#
# Copyright(C) 2024 RT-Coorp.
#  This program release with Apache License version 2.0
#
import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State, Publisher
import lifecycle_msgs.msg

from typing import Optional

import numpy as np
import quaternion
import math

from rclpy.timer import Timer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import triorb_core

import std_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
import time

def transform_3D_frame(X,R):
    """
    X: Initial vector to rotate.
    R: 3D rotation matrix.
    """
    new_X = np.dot(R,X)
    return new_X

class TriOrb(LifecycleNode):
    def __init__(self):
        super().__init__('triorb')
        self.get_logger().info('Ready, Waiting to be configured...')

        self._state_pub = self.create_publisher(lifecycle_msgs.msg.State, '/triorb_node_state', 10)
        self.declare_parameter('state_pub_duration', 1.0)
        self._state_pub_duration = self.get_parameter('state_pub_duration').get_parameter_value().double_value
        self._state_pub_timer = self.create_timer(self._state_pub_duration, self.publish_state)

        #Frame rotation matrix
        self.R = np.array([
                    [0,-1,0],
                    [1,0,0],
                    [0,0,-1]
                    ])
        self.R_inv = np.array([
                    [0,1,0],
                    [-1,0,0],
                    [0,0,-1]
                    ])
        
    # Publish the state of the node
    def publish_state(self):
        try:
            current_state = self._state_machine.current_state
            msg = lifecycle_msgs.msg.State()
            msg.id = current_state[0]
            msg.label= current_state[1]
            self._state_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'Fail to publish node state: {str(e)}')
            pass

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
        self.get_logger().info('open port: '+self.port)
        
        try:
            self._vehicle = triorb_core.robot(self.port, self)
        except:
            self.get_logger().error('Fail to connect port: ' +self.port)
            return TransitionCallbackReturn.FAILURE

        self._pub = self.create_lifecycle_publisher(nav_msgs.msg.Odometry, '/odom_triorb',  10)
        self._odom_duration = self.get_parameter('odom_duration').get_parameter_value().double_value
        #self._timer_callback = MutuallyExclusiveCallbackGroup()
        self._timer_callback = None
        self._timer = self.create_timer(self._odom_duration, self.publish_odometry, callback_group=self._timer_callback)

        self._watchdog_timeout = self.get_parameter('watchdog_timeout').get_parameter_value().double_value
        #self._watchdog_callback = MutuallyExclusiveCallbackGroup()
        self._watchdog_callback = None
        self._watchdog = self.create_timer(self._watchdog_timeout, self.cb_watchdog, callback_group=self._watchdog_callback)
        self._watchdog.cancel()

        self.topic_in = '/cmd_vel_apply'
        self._sub_vel = self.create_subscription(geometry_msgs.msg.Twist, self.topic_in, self.cb_cmd_velocity, 10)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        try:
            self.get_logger().info('on_activate')
            self._vehicle.wakeup()
            self._watchdog.reset()
            self._pub.on_activate(state)
            self.get_logger().info('Waiting command on topic '+self.topic_in)
        except:
            #time.sleep(5) #tests
            self.get_logger().error('Failed to activate')
            return TransitionCallbackReturn.FAILURE
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        try:
            self.get_logger().info('on_deactivate')
            self._vehicle.sleep()
            self._watchdog.cancel()
            self._pub.on_deactivate(state)
        except:
            self.get_logger().error('Failed to deactivate')
            return TransitionCallbackReturn.FAILURE
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        try: 
            self.stop()
            self.get_logger().info('on_cleanup')
            self.destroy_timer(self._timer)
            self.destroy_publisher(self._pub)
            del self._vehicle
            self._vehicle = None
        except:
            self.get_logger().error('Failed to clean_up')
            return TransitionCallbackReturn.FAILURE
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_shutdown') 
        self.stop()
        self.destroy_timer(self._timer)
        self.destroy_publisher(self._pub)
        try:
            self._vehicle.sleep()
            del self._vehicle
            self._vehicle = None
        except:
            pass
        self.get_logger().info('on_shutdown finished') 
        return TransitionCallbackReturn.SUCCESS

    #################### Callback functions and basic functions
    #
    # Callback of timer 
    def publish_odometry(self):
        if self._pub is not None and self._pub.is_activated:
            try:
                pose = self._vehicle.get_pos()[0]
                #print(pose)
                
                """if pose is None and self.debug:
                import random
                pose = triorb_core.core_types.TriOrbDrive3Pose()
                pose.x = random.random()
                pose.w = random.random()"""
                
                #Unrotate position vector to match wanted frame
                pose_array = np.array([[pose.x],[pose.y],[pose.w]])
                new_pose_array = transform_3D_frame(pose_array,self.R_inv)

                self._odom.pose.pose.position.x = new_pose_array[0,0]
                self._odom.pose.pose.position.y = new_pose_array[1,0]
                quat=quaternion.from_euler_angles(0,0,new_pose_array[2,0])

                self._odom.pose.pose.orientation.x = quat.x
                self._odom.pose.pose.orientation.y = quat.y
                self._odom.pose.pose.orientation.z = quat.z
                self._odom.pose.pose.orientation.w = quat.w

                #Unrotate speed vector to match wanted frame
                speed_array = np.array([[self.vx],[self.vy],[self.vw]])
                new_speed_array = transform_3D_frame(speed_array,self.R_inv)

                self._odom.twist.twist.linear.x = new_speed_array[0,0]
                self._odom.twist.twist.linear.y = new_speed_array[1,0]
                self._odom.twist.twist.angular.z = new_speed_array[2,0]
                self._odom.header.stamp = self.get_clock().now().to_msg()
                self._pub.publish(self._odom)
            except:
                self.get_logger().warn("Couldn't publish odometry, robot disconnected? ") 
        return

    #
    # Callback of watchdog timer
    def cb_watchdog(self):
        self.get_logger().info('watchdog, robot stopped')
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

        #rotate commands to match wanted frame
        speed_array = np.array([[msg.linear.x],[msg.linear.y],[msg.angular.z]])
        new_speed_array = transform_3D_frame(speed_array,self.R)

        self.vx,self.vy,self.vw = new_speed_array.flatten()

        # self._vehicle.set_vel_relative(self.vx, self.vy, self.vw, acc=2000, dec=2000) #PArameters to work with acceleration
        self._vehicle.set_vel_relative(self.vx, self.vy, self.vw)
        self._watchdog.reset()
        return

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
