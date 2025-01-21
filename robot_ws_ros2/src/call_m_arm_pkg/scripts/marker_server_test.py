#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose
from call_m_custom_msgs.srv import SetGripperMarker
import tf_transformations


class MarkerServiceNode(Node):
    def __init__(self):
        super().__init__('marker_service_node')
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.service = self.create_service(SetGripperMarker, 'set_marker', self.handle_set_marker)
        self.get_logger().info('Marker service ready.')

    def handle_set_marker(self, request, response):
        # Create the MarkerArray message
        marker_array = MarkerArray()

        # Gripper dimensions (in meters)
        base_width = 0.1  # Width of the gripper base
        base_depth = 0.1  # Depth of the gripper base
        base_height = 0.02  # Height of the gripper base

        finger_width = 0.02  # Width of each finger
        finger_depth = 0.02  # Depth of each finger
        finger_length = 0.17  # Length of each finger

        finger_separation = 0.1  # Distance between the centers of the fingers

        # Common properties
        ns = "gripper"
        frame_id = request.frame_id

       # Compute rotation matrix from the orientation quaternion of the gripper base (root)
        pose_orientation = request.pose.orientation

        # Create a quaternion for 90-degree rotation around the Z-axis (yaw)
        yaw_rotation = tf_transformations.quaternion_from_euler(0, 0, 1.5708)  # 90 degrees in radians

        # Apply the 90-degree rotation to the received pose_orientation (root orientation)
        rotated_base_orientation = tf_transformations.quaternion_multiply(
            [pose_orientation.x, pose_orientation.y, pose_orientation.z, pose_orientation.w], yaw_rotation
        )

        # Position offsets (keeping relative offsets the same)
        half_finger_separation = finger_separation / 2
        half_base_height = base_height / 2
        base_offset = [0, 0, half_base_height, 1]  # Gripper base
        finger1_offset = [half_finger_separation, 0, base_height + finger_length / 2, 1]  # Finger 1
        finger2_offset = [-half_finger_separation, 0, base_height + finger_length / 2, 1]  # Finger 2

        # Transform the offsets (base and fingers) using the rotated base orientation
        rotation_matrix = tf_transformations.quaternion_matrix(rotated_base_orientation)
        base_position = rotation_matrix @ base_offset
        finger1_position = rotation_matrix @ finger1_offset
        finger2_position = rotation_matrix @ finger2_offset

        # Add the gripper position (translation) from the request
        gripper_position = request.pose.position
        base_position[:3] += [gripper_position.x, gripper_position.y, gripper_position.z]
        finger1_position[:3] += [gripper_position.x, gripper_position.y, gripper_position.z]
        finger2_position[:3] += [gripper_position.x, gripper_position.y, gripper_position.z]

        # Create markers for the gripper base and fingers with the rotated base orientation
        base_marker = self.create_marker(frame_id, ns, 0, Marker.CUBE, base_position, rotated_base_orientation,
                                        base_width, base_depth, base_height, 0.0, 0.0, 0.0, 1.0)
        finger1_marker = self.create_marker(frame_id, ns, 1, Marker.CUBE, finger1_position, rotated_base_orientation,
                                            finger_width, finger_depth, finger_length, 1.0, 0.0, 0.0, 1.0)
        finger2_marker = self.create_marker(frame_id, ns, 2, Marker.CUBE, finger2_position, rotated_base_orientation,
                                            finger_width, finger_depth, finger_length, 0.0, 0.0, 1.0, 1.0)

        # Add the markers to the MarkerArray
        marker_array.markers.append(base_marker)
        marker_array.markers.append(finger1_marker)
        marker_array.markers.append(finger2_marker)

        # Publish the MarkerArray
        self.marker_pub.publish(marker_array)
        self.get_logger().info(f"Published gripper marker at frame '{frame_id}' with pose {request.pose}")
        response.success = True
        return response

    def create_marker(self, frame_id, ns, marker_id, marker_type, position, orientation, scale_x, scale_y, scale_z,
                      color_r, color_g, color_b, color_a):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]
        marker.scale.x = scale_x
        marker.scale.y = scale_y
        marker.scale.z = scale_z
        marker.color.r = color_r
        marker.color.g = color_g
        marker.color.b = color_b
        marker.color.a = color_a
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = MarkerServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
