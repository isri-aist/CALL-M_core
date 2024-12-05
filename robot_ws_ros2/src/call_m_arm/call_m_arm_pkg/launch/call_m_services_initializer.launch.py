
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='call_m_arm_pkg',
            executable='arm_server',
            name='arm_server',
            output='screen',
            parameters=[{'use_sim_time': True}],  # Ensure necessary parameters are set
        ),
        Node(
            package='call_m_arm_pkg',
            executable='grasp_pose_detector_server',
            name="detector_server"
        ),
        Node(
            package="call_m_arm_pkg",
            executable='pointcloud_server',
            name="pointcloud_server"
        )
    ])