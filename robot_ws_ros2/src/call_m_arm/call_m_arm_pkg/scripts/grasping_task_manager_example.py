#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from call_m_custom_msgs.srv import ArmMoveToPose, ArmMoveToJoints, ArmMoveCartesian, GraspDetection, GenerateCollisionObject, RemoveCollisionObject, GetEndEffectorPose, GetJointValues, GripperAction, SavePointCloud
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose, PoseArray
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped, Quaternion
import math
import time
import copy

ARM_STAND_UP_BACK_POSE = [float(x) for x in [0, -1.57, 0, 0, 0, 3.14]]
ARM_STAND_UP_LEFT_POSE = [float(x) for x in [1.5708, -1.57, 0, 0, 0, 3.14]]
ARM_STAND_UP_FRONT_POSE = [float(x) for x in [3.14159, -1.57, 0, 0, 0, 3.14]]
ARM_STAND_UP_RIGHT_POSE = [float(x) for x in [4.71239, -1.57, 0, 0, 0, 3.14]]
ARM_OBJECT_VIEW_POSE = [float(x) for x in [0, -0.628319, 1.76278, -3.57792, -1.5708, -1.5708]]
GRIPPER_DISTANCE = 0.12

class ArmServer(Node):
    def __init__(self):
        super().__init__('arm_server')
        
        self.use_sim = True

        # Initialize clients
        self.arm_move_to_pose_client = self.create_client(ArmMoveToPose, 'arm_move_to_pose')
        self.arm_move_to_joints_client = self.create_client(ArmMoveToJoints, 'arm_move_to_joints')
        self.arm_move_cartesian_client = self.create_client(ArmMoveCartesian, 'arm_move_cartesian')
        self.grasp_detection_client = self.create_client(GraspDetection, 'detect_grasps')
        self.generate_collision_object_client = self.create_client(GenerateCollisionObject, 'generate_collision_object')
        self.remove_collision_object_client = self.create_client(RemoveCollisionObject, 'remove_collision_object')
        self.get_end_effector_pose_client = self.create_client(GetEndEffectorPose, 'get_end_effector_pose')
        self.get_joint_values_client = self.create_client(GetJointValues, "get_joint_values")
        self.save_point_cloud_client = self.create_client(SavePointCloud, '/save_point_cloud')
        self.clear_octomap_client = self.create_client(Empty, 'clear_octomap')

        if not self.use_sim:
            self.gripper_action_client = self.create_client(GripperAction,"gripper_command")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Wait for all services to become available
        self.wait_for_services()

    def wait_for_services(self):
        if self.use_sim:
            services = [
                ('arn_move_to_pose', self.arm_move_to_pose_client),
                ('arm_move_to_joints', self.arm_move_to_joints_client),
                ('arm_move_cartesian', self.arm_move_cartesian_client),
                ('detect_grasps', self.grasp_detection_client),
                ('generate_collision_object', self.generate_collision_object_client),
                ('save_point_cloud', self.save_point_cloud_client),
                ('remove_collision_object', self.remove_collision_object_client),
                ('clear_octomap', self.clear_octomap_client),
                ('get_end_effector_pose', self.get_end_effector_pose_client),
                ('get_joint_values', self.get_joint_values_client)
            ]
        else:
            services = [
                ('arn_move_to_pose', self.arm_move_to_pose_client),
                ('arm_move_to_joints', self.arm_move_to_joints_client),
                ('arm_move_cartesian', self.arm_move_cartesian_client),
                ('detect_grasps', self.grasp_detection_client),
                ('generate_collision_object', self.generate_collision_object_client),
                ('save_point_cloud', self.save_point_cloud_client),
                ('remove_collision_object', self.remove_collision_object_client),
                ('clear_octomap', self.clear_octomap_client),
                ('get_end_effector_pose', self.get_end_effector_pose_client),
                ('get_joint_values', self.get_joint_values_client),
                ('gripper_command',self.gripper_action_client)           
            ]
        for name, client in services:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for the {name} service to be available...')

    def save_point_cloud_request(self,path,topic):
        self.get_logger().info("Calling save_point_cloud service...")

        # Create a request object for the SavePointCloud service
        request = SavePointCloud.Request()
        request.path = path
        request.topic = topic

        # Call the service asynchronously
        save_future = self.save_point_cloud_client.call_async(request)
        return self.wait_for_future(save_future, "SavePointCloud")

    
    def clear_octomap_request(self):
        self.get_logger().info("Calling clear octomap service...")
        save_future = self.clear_octomap_client.call_async(Empty.Request())
        return self.wait_for_future(save_future, "ClearOctomap")

    def send_grasp_detection_request(self, cfg_path: str, pcd_path: str):
        """Sends a request to the GraspDetection service with the provided configuration and PCD paths."""
        self.get_logger().info("Calling detect_grasps service...")

        # Create the request object and populate the fields
        grasp_request = GraspDetection.Request()
        grasp_request.cfg_path = cfg_path
        grasp_request.pcd_path = pcd_path

        # Call the service asynchronously
        grasp_future = self.grasp_detection_client.call_async(grasp_request)
        
        # Wait for the future to complete
        self.wait_for_future(grasp_future, "GraspDetection")

        # Retrieve and process the response
        grasp_response = grasp_future.result()
        if grasp_response:
            if grasp_response.success:
                if grasp_response.grasp_poses.poses:
                    self.get_logger().info("Grasp detected successfully.")
                    return grasp_response.grasp_poses.poses[0]  # Return the first detected grasp pose
                else:
                    self.get_logger().warn("Grasp detection succeeded but no poses were found.")
            else:
                self.get_logger().warn(f"Grasp detection failed: {grasp_response.message}")
        else:
            self.get_logger().error("Failed to get a response from the GraspDetection service.")

        return None

    
    def get_end_effector_pose(self):
        self.get_logger().info("Calling get_end_effector_pose service...")
        pose_request = GetEndEffectorPose.Request()  # Request is empty
        pose_future = self.get_end_effector_pose_client.call_async(pose_request)
        result = self.wait_for_future(pose_future, "GetEndEffectorPose")
        if result:
            return result.pose  # Return the pose
        return None

    def get_joint_values(self):
        self.get_logger().info("Calling get_joint_values service...")
        joint_values_request = GetJointValues.Request()
        joint_values_future = self.get_joint_values_client.call_async(joint_values_request)
        result = self.wait_for_future(joint_values_future, "GetJointValues")
        if result:
            return result.joints #Return joint values
        return None

    def send_generate_collision_object_request(self, pose: Pose, length: float, width: float, depth: float, object_id: str):
        self.get_logger().info("Calling generate_collision_object service...")
        collision_request = GenerateCollisionObject.Request()
        collision_request.pose = pose
        collision_request.length = length
        collision_request.width = width
        collision_request.depth = depth
        collision_request.object_id = object_id
        collision_future = self.generate_collision_object_client.call_async(collision_request)
        self.wait_for_future(collision_future, "GenerateCollisionObject")
        return collision_future.result().success if collision_future.result() else False
    
    def send_remove_object_collision_object_request(self, object_id: str):
        self.get_logger().info("Calling remove_collision_object service...")
        remove_collision_object_request = RemoveCollisionObject.Request()
        remove_collision_object_request.object_id = object_id
        remove_collision_object_future = self.remove_collision_object_client.call_async(remove_collision_object_request)
        self.wait_for_future(remove_collision_object_future, "RemoveCollisionObject")
        return remove_collision_object_future.result().success if remove_collision_object_future.result() else False

    def send_move_to_pose_request(self, target_pose: Pose):
        self.get_logger().info("Calling move_to_pose service...")
        move_request = ArmMoveToPose.Request()
        move_request.target_pose = target_pose
        move_future = self.arm_move_to_pose_client.call_async(move_request)
        self.wait_for_future(move_future, "MoveToPose")
        return move_future.result().success if move_future.result() else False

    def send_move_to_joints_request(self, joint_positions: list):
        self.get_logger().info("Calling move_to_joints service...")
        move_to_joints_request = ArmMoveToJoints.Request()
        move_to_joints_request.joints = joint_positions
        joints_future = self.arm_move_to_joints_client.call_async(move_to_joints_request)
        self.wait_for_future(joints_future, "MoveToJoints")
        return joints_future.result().success if joints_future.result() else False
    
    def send_cartesian_move_request(self, waypoints):
        self.get_logger().info("Calling arm_move_cartesian service...")
        cartesian_request = ArmMoveCartesian.Request()
        cartesian_request.waypoints = waypoints
        cartesian_future = self.arm_move_cartesian_client.call_async(cartesian_request)
        self.wait_for_future(cartesian_future, "ArmMoveCartesian")
        return cartesian_future.result().success if cartesian_future.result() else False
    
    def send_gripper_command_request(self, command):
        self.get_logger().info("Calling gripper command service...")
        gripper_command_request = GripperAction.Request()
        gripper_command_request.close = command
        gripper_command_future = self.gripper_action_client.call_async(gripper_command_request)
        self.wait_for_future(gripper_command_future,"GripperAction")
        return gripper_command_future.result().success if gripper_command_future.result() else False
    
    def wait_for_future(self, future, service_name):
        """Utility function to block execution until the future completes."""
        while not future.done():
            rclpy.spin_once(self)
        try:
            result = future.result()
            self.get_logger().info(f"{service_name} service completed successfully.")
            return result
        except Exception as e:
            self.get_logger().error(f"{service_name} service call failed: {e}")
            return None
        
    def get_end_effector_position(self):
        try:
            # Loop until transform is available
            while not self.tf_buffer.can_transform('base', 'tool0', rclpy.time.Time()):
                self.get_logger().info("Waiting for transform from base to tool0...")
                rclpy.spin_once(self)

            # Once available, get the transform
            transform = self.tf_buffer.lookup_transform('base', 'tool0', rclpy.time.Time())
            position = transform.transform.translation
            orientation = transform.transform.rotation
            self.get_logger().info(f"Position of tool0 relative to base:\n"
                                f"x: {position.x}\n"
                                f"y: {position.y}\n"
                                f"z: {position.z}\n"
                                f"Orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}")

            eef_pose = Pose()
            eef_pose.position.x = position.x
            eef_pose.position.y = position.y
            eef_pose.position.z = position.z
            eef_pose.orientation.x = orientation.x
            eef_pose.orientation.y = orientation.y
            eef_pose.orientation.z = orientation.z 
            eef_pose.orientation.w = orientation.w

            return eef_pose
        except Exception as e:
            self.get_logger().error(f"Failed to get transform: {e}")
            return None
    
    def point_to_waypoints(self, eef_pose, target_pose):
        waypoints = PoseArray()

        # X-axis waypoint
        temp_waypoint_x = Pose()
        temp_waypoint_x.position.x = target_pose.position.x
        temp_waypoint_x.position.y = eef_pose.position.y
        temp_waypoint_x.position.z = eef_pose.position.z
        temp_waypoint_x.orientation = eef_pose.orientation
        waypoints.poses.append(temp_waypoint_x)

        # Y-axis waypoint
        temp_waypoint_y = Pose()
        temp_waypoint_y.position.x = target_pose.position.x
        temp_waypoint_y.position.y = target_pose.position.y
        temp_waypoint_y.position.z = eef_pose.position.z
        temp_waypoint_y.orientation = eef_pose.orientation
        waypoints.poses.append(temp_waypoint_y)

        # Z-axis waypoint
        temp_waypoint_z = Pose()
        temp_waypoint_z.position.x = target_pose.position.x
        temp_waypoint_z.position.y = target_pose.position.y
        temp_waypoint_z.position.z = target_pose.position.z
        temp_waypoint_z.orientation = eef_pose.orientation
        waypoints.poses.append(temp_waypoint_z)

        return waypoints
    
    def pose_to_waypoints(self, eef_pose, target_pose):
        waypoints = PoseArray()

        # X-axis waypoint
        temp_waypoint_x = Pose()
        temp_waypoint_x.position.x = target_pose.position.x
        temp_waypoint_x.position.y = eef_pose.position.y
        temp_waypoint_x.position.z = eef_pose.position.z
        temp_waypoint_x.orientation = target_pose.orientation
        waypoints.poses.append(temp_waypoint_x)

        # Y-axis waypoint
        temp_waypoint_y = Pose()
        temp_waypoint_y.position.x = target_pose.position.x
        temp_waypoint_y.position.y = target_pose.position.y
        temp_waypoint_y.position.z = eef_pose.position.z
        temp_waypoint_y.orientation = target_pose.orientation
        waypoints.poses.append(temp_waypoint_y)

        # Z-axis waypoint
        temp_waypoint_z = Pose()
        temp_waypoint_z.position.x = target_pose.position.x
        temp_waypoint_z.position.y = target_pose.position.y
        temp_waypoint_z.position.z = target_pose.position.z
        temp_waypoint_z.orientation = target_pose.orientation
        waypoints.poses.append(temp_waypoint_z)

        return waypoints
        
    def extract_yaw_from_pose(self,pose):
        # Convert the quaternion to Euler angles
        quat = pose.orientation
        roll, pitch, yaw = self.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return yaw
    
    def adjust_gripper_yaw(self,grasp_pose, current_joints):
        # Extract Yaw from the grasp pose
        desired_yaw = self.extract_yaw_from_pose(grasp_pose)
        
        # Assuming the last joint is the one that controls the gripper's rotation (e.g., wrist rotation)
        # Modify the last joint's angle to the desired Yaw while keeping other joint values intact
        current_joints[-1] = desired_yaw  # Set the last joint to the desired Yaw
        
        return current_joints
    
    def modify_grasp_pose_yaw(self,grasp_pose, new_yaw):
        # Get current orientation in Quaternion
        quat = grasp_pose.orientation
        
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        roll, pitch, current_yaw = self.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        
        # Modify the Yaw (heading)
        new_yaw = new_yaw  # Desired Yaw
        
        # Rebuild the quaternion from the modified Euler angles
        new_quat = self.quaternion_from_euler(roll, pitch, new_yaw)
        
        # Set the new quaternion to the grasp pose
        grasp_pose.orientation.x = new_quat[0]
        grasp_pose.orientation.y = new_quat[1]
        grasp_pose.orientation.z = new_quat[2]
        grasp_pose.orientation.w = new_quat[3]
        
        return grasp_pose
    
    def euler_from_quaternion(self,quat):
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        x, y, z, w = quat
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        roll_deg = math.degrees(roll_x)
        pitch_deg = math.degrees(pitch_y)
        yaw_deg = math.degrees(yaw_z)

        print(f"Roll: {roll_deg:.2f}°, Pitch: {pitch_deg:.2f}°, Yaw: {yaw_deg:.2f}°")

        return roll_x, pitch_y, yaw_z

    def quaternion_from_euler(self,roll, pitch, yaw):
        # Convert Euler angles to quaternion
        qx = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) - math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        
        return [qx, qy, qz, qw]
    
    def quaternion_multiply(self, q1, q2):
        """
        Multiply two quaternions q1 and q2.
        q1 = [x1, y1, z1, w1]
        q2 = [x2, y2, z2, w2]

        Returns: result quaternion [x, y, z, w]
        """
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2

        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2

        return [x, y, z, w]

    def scan_environment(self):
         if self.send_move_to_joints_request(ARM_STAND_UP_BACK_POSE):
            time.sleep(1)
            if self.send_move_to_joints_request(ARM_STAND_UP_LEFT_POSE):
                time.sleep(1)
                if self.send_move_to_joints_request(ARM_STAND_UP_FRONT_POSE):
                    time.sleep(1)
                    if self.send_move_to_joints_request(ARM_STAND_UP_RIGHT_POSE):
                        time.sleep(1)

def cartesian_demo(arm_server):
    arm_server.get_logger().info('Starting simple demo...')
    # arm_server.scan_environment()
    if arm_server.send_move_to_joints_request(ARM_OBJECT_VIEW_POSE):
        current_pose = arm_server.get_end_effector_pose()
        
        if arm_server.save_point_cloud_request('/workspace/ros2-workspace/call_m_arm_ws/src/call_m_arm/gpd/tutorials/output.pcd','/zed_eef_camera/points'):
            time.sleep(2)
            grasp_pose = arm_server.send_grasp_detection_request('/workspace/ros2-workspace/call_m_arm_ws/src/call_m_arm/gpd/cfg/eigen_params.cfg','/workspace/ros2-workspace/call_m_arm_ws/src/call_m_arm/gpd/tutorials/output.pcd')
            print("END EFFECTOR POSE")
            print(grasp_pose)
            quat = [
                grasp_pose.orientation.x,
                grasp_pose.orientation.y,
                grasp_pose.orientation.z,
                grasp_pose.orientation.w
            ]

            print(arm_server.euler_from_quaternion(quat))

        if grasp_pose:
            grasp_plane = copy.deepcopy(grasp_pose)
            grasp_plane.position.z -= 0.1
            grasp_plane.orientation.x = 0.0
            grasp_plane.orientation.y = 0.0
            grasp_plane.orientation.z = 1.0
            grasp_plane.orientation.w = 0.0
            
            # Generate collision object
            if arm_server.send_generate_collision_object_request(
                pose=grasp_plane,
                length=0.5,
                width=0.5,
                depth=0.05,
                object_id="temporary_collision_object"
            ):
                time.sleep(2)
                print(grasp_pose)

                # Convert grasp pose to waypoints
                waypoints_ = arm_server.pose_to_waypoints(current_pose, grasp_pose)
                for waypoint in waypoints_.poses:
                    print(waypoint)

                # Rotation quaternion for 90 degrees around the Y-axis
                rotation_quat = arm_server.quaternion_from_euler(0, math.pi / 2, 0)  # Rotate around Y-axis

                # Multiply the current orientation by the rotation quaternion
                rotated_orientation = arm_server.quaternion_multiply(
                    [grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z, grasp_pose.orientation.w],
                    rotation_quat
                )

                # Update the last waypoint to have the rotated orientation
                last_waypoint = waypoints_.poses[-1]
                last_waypoint.orientation.x = rotated_orientation[0]
                last_waypoint.orientation.y = rotated_orientation[1]
                last_waypoint.orientation.z = rotated_orientation[2]
                last_waypoint.orientation.w = rotated_orientation[3]
                
                # Leave a margin in Z axis for grasping
                last_waypoint.position.z += (GRIPPER_DISTANCE)

                # Pre-grasp position
                if arm_server.send_cartesian_move_request(waypoints_):
                    #arm_server.send_gripper_command_request(True)
                    time.sleep(5)
                    arm_server.send_move_to_joints_request(ARM_OBJECT_VIEW_POSE)
                    arm_server.send_remove_object_collision_object_request("temporary_collision_object")
                    #arm_server.send_gripper_command_request(False)

def joint_demo(arm_server):
    arm_server.get_logger().info('Starting simple demo...')
    # arm_server.scan_environment()
    if arm_server.send_move_to_joints_request(ARM_OBJECT_VIEW_POSE):
        current_pose = arm_server.get_end_effector_pose()
        
        if arm_server.save_point_cloud_request('/workspace/ros2-workspace/src/call_m_arm/gpd/tutorials/output.pcd','/zed_eef_camera/points'):
            time.sleep(2)
            grasp_pose = arm_server.send_grasp_detection_request()
            print("END EFFECTOR POSE")
            print(grasp_pose)
            quat = [
                grasp_pose.orientation.x,
                grasp_pose.orientation.y,
                grasp_pose.orientation.z,
                grasp_pose.orientation.w
            ]

            print(arm_server.euler_from_quaternion(quat))

        if grasp_pose:
            grasp_plane = copy.deepcopy(grasp_pose)
            grasp_plane.position.z -= 0.1
            grasp_plane.orientation.x = 0.0
            grasp_plane.orientation.y = 0.0
            grasp_plane.orientation.z = 1.0
            grasp_plane.orientation.w = 0.0
            
            # Generate collision object
            if arm_server.send_generate_collision_object_request(
                pose=grasp_plane,
                length=0.5,
                width=0.5,
                depth=0.05,
                object_id="temporary_collision_object"
            ):
                time.sleep(2)
                print(grasp_pose)

                # Convert grasp pose to waypoints
                waypoints_ = arm_server.pose_to_waypoints(current_pose, grasp_pose)
                for waypoint in waypoints_.poses:
                    print(waypoint)

                # Rotation quaternion for 90 degrees around the Y-axis
                rotation_quat = arm_server.quaternion_from_euler(0, math.pi / 2, 0)  # Rotate around Y-axis

                # Multiply the current orientation by the rotation quaternion
                rotated_orientation = arm_server.quaternion_multiply(
                    [grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z, grasp_pose.orientation.w],
                    rotation_quat
                )

                # Update the last waypoint to have the rotated orientation
                last_waypoint = waypoints_.poses[-1]
                last_waypoint.orientation.x = rotated_orientation[0]
                last_waypoint.orientation.y = rotated_orientation[1]
                last_waypoint.orientation.z = rotated_orientation[2]
                last_waypoint.orientation.w = rotated_orientation[3]
                
                # Leave a margin in Z axis for grasping
                last_waypoint.position.z += (GRIPPER_DISTANCE)

                # Pre-grasp position
                if arm_server.send_move_to_pose_request(waypoints_.poses[-1]):
                    #arm_server.send_gripper_command_request(True)
                    time.sleep(5)
                    arm_server.send_move_to_joints_request(ARM_OBJECT_VIEW_POSE)
                    arm_server.send_remove_object_collision_object_request("temporary_collision_object")
                    #arm_server.send_gripper_command_request(False)

def cartesian_and_joint_demo(arm_server):
    arm_server.get_logger().info('Starting simple demo...')
    # arm_server.scan_environment()
    if arm_server.send_move_to_joints_request(ARM_OBJECT_VIEW_POSE):        
        if arm_server.save_point_cloud_request('/workspace/ros2-workspace/src/call_m_arm/gpd/tutorials/output.pcd','/zed_eef_camera/points'):
            time.sleep(2)
            grasp_pose = arm_server.send_grasp_detection_request()
            print("END EFFECTOR POSE")
            print(grasp_pose)
            quat = [
                grasp_pose.orientation.x,
                grasp_pose.orientation.y,
                grasp_pose.orientation.z,
                grasp_pose.orientation.w
            ]

            print(arm_server.euler_from_quaternion(quat))

            # Rotation quaternion for 90 degrees around the Y-axis
            rotation_quat = arm_server.quaternion_from_euler(0, math.pi / 2, 0)  # Rotate around Y-axis

            # Multiply the current orientation by the rotation quaternion
            rotated_orientation = arm_server.quaternion_multiply(
                [grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z, grasp_pose.orientation.w],
                rotation_quat
            )

            #Corrected grasp pose
            grasp_pose.orientation.x = rotated_orientation[0]
            grasp_pose.orientation.y = rotated_orientation[1]
            grasp_pose.orientation.z = rotated_orientation[2]
            grasp_pose.orientation.w = rotated_orientation[3]

        if grasp_pose:
            grasp_plane = copy.deepcopy(grasp_pose)
            grasp_plane.position.z -= 0.1
            grasp_plane.orientation.x = 0.0
            grasp_plane.orientation.y = 0.0
            grasp_plane.orientation.z = 1.0
            grasp_plane.orientation.w = 0.0
            
            # Generate collision object
            if arm_server.send_generate_collision_object_request(
                pose=grasp_plane,
                length=0.5,
                width=0.5,
                depth=0.05,
                object_id="temporary_collision_object"
            ):
                time.sleep(2)
                print(grasp_pose)

                #Get pre-grasp position
                approach_pose = copy.deepcopy(grasp_pose)
                approach_pose.position.z += GRIPPER_DISTANCE+0.1
                arm_server.send_move_to_pose_request(approach_pose)
                
                current_pose = arm_server.get_end_effector_pose()
                
                # Convert grasp pose to waypoints
                waypoints_ = arm_server.pose_to_waypoints(current_pose, grasp_pose)
                for waypoint in waypoints_.poses:
                    print(waypoint)

                # Update the last waypoint to have the rotated orientation
                last_waypoint = waypoints_.poses[-1]

                
                # Leave a margin in Z axis for grasping
                last_waypoint.position.z += (GRIPPER_DISTANCE)

                # Pre-grasp position
                if arm_server.send_cartesian_move_request(waypoints_):
                    #arm_server.send_gripper_command_request(True)
                    time.sleep(5)
                    arm_server.send_move_to_joints_request(ARM_OBJECT_VIEW_POSE)
                    arm_server.send_remove_object_collision_object_request("temporary_collision_object")
                    #arm_server.send_gripper_command_request(False)

def main(args=None):
    rclpy.init(args=args)
    arm_server = ArmServer()
    arm_server.get_logger().info('ArmServer running')
    # arm_server.send_gripper_command_request(True)
    # arm_server.send_gripper_command_request(False)
    try:
        cartesian_demo(arm_server)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
