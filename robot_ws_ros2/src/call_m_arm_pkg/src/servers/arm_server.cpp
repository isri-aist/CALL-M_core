#include <rclcpp/rclcpp.hpp>
#include "std_srvs/srv/empty.hpp"  // Header for the Empty service
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include "call_m_custom_msgs/srv/arm_move_to_pose.hpp"
#include "call_m_custom_msgs/srv/arm_move_to_joints.hpp"
#include "call_m_custom_msgs/srv/generate_collision_object.hpp"
#include "call_m_custom_msgs/srv/remove_collision_object.hpp"
#include "call_m_custom_msgs/srv/arm_move_cartesian.hpp"
#include "call_m_custom_msgs/srv/get_end_effector_pose.hpp"
#include "call_m_custom_msgs/srv/get_joint_values.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <iostream>

// Combined ArmServerNode class
class ArmServerNode : public rclcpp::Node {
public:
  ArmServerNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("robot_control", options),
      node_(std::make_shared<rclcpp::Node>("move_group_interface",
                rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)}))),
      move_group_interface_(node_, "ur_manipulator"),
      executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()) {
        move_group_interface_.setPoseReferenceFrame("base");
    
        // Create the services
        arm_move_to_pose_service_ = this->create_service<call_m_custom_msgs::srv::ArmMoveToPose>(
            "arm_move_to_pose",
            std::bind(&ArmServerNode::handle_arm_move_to_pose_service, this, std::placeholders::_1, std::placeholders::_2)
        );

        arm_move_to_joints_service_ = this->create_service<call_m_custom_msgs::srv::ArmMoveToJoints>(
            "arm_move_to_joints",
            std::bind(&ArmServerNode::handle_arm_move_to_joints_service, this, std::placeholders::_1, std::placeholders::_2)
        );

        arm_move_cartesian_service_ = this->create_service<call_m_custom_msgs::srv::ArmMoveCartesian>(
            "arm_move_cartesian",
            std::bind(&ArmServerNode::handle_arm_move_cartesian_service_, this, std::placeholders::_1, std::placeholders::_2)
        );

        generate_collision_object_service_ = this->create_service<call_m_custom_msgs::srv::GenerateCollisionObject>(
            "generate_collision_object",
            std::bind(&ArmServerNode::handle_generate_collision_object_service, this, std::placeholders::_1, std::placeholders::_2)
        );

        remove_collision_object_service = this->create_service<call_m_custom_msgs::srv::RemoveCollisionObject>(
            "remove_collision_object",
            std::bind(&ArmServerNode::handle_remove_collision_object_service, this, std::placeholders::_1, std::placeholders::_2)
        );
        
        get_end_effector_pose_service_ = this->create_service<call_m_custom_msgs::srv::GetEndEffectorPose>(
            "get_end_effector_pose",
            std::bind(&ArmServerNode::handle_get_end_effector_pose_service, this, std::placeholders::_1, std::placeholders::_2)
        );

        get_joint_values_service_ = this->create_service<call_m_custom_msgs::srv::GetJointValues>(
            "get_joint_values",
            std::bind(&ArmServerNode::handle_get_joint_values_service, this, std::placeholders::_1, std::placeholders::_2)
        );

        // Start the executor thread
        executor_->add_node(node_);
        executor_thread_ = std::thread([this]() {
        RCLCPP_INFO(node_->get_logger(), "Starting executor thread");
        executor_->spin();
        });
    }     

  ~ArmServerNode() {
    executor_->cancel();
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
  }

private:
    void handle_arm_move_to_pose_service(
        const std::shared_ptr<call_m_custom_msgs::srv::ArmMoveToPose::Request> request,
        std::shared_ptr<call_m_custom_msgs::srv::ArmMoveToPose::Response> response
    ) {
        // Set the target pose from the service request
        move_group_interface_.setPoseTarget(request->target_pose);

        // Plan and execute the motion
        auto success = move_group_interface_.move();
        response->success = (success == moveit::core::MoveItErrorCode::SUCCESS);

        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Motion to target pose succeeded.");
        } else {
            RCLCPP_WARN(this->get_logger(), "Motion to target pose failed.");
        }
    }

    void handle_arm_move_to_joints_service(
        const std::shared_ptr<call_m_custom_msgs::srv::ArmMoveToJoints::Request> request,
        std::shared_ptr<call_m_custom_msgs::srv::ArmMoveToJoints::Response> response
    ) {
        // Set the joint target values from the service request
        if (request->joints.size() == 6) {  // Ensure there are 6 joint values
            move_group_interface_.setJointValueTarget(request->joints);

            // Plan and execute the motion
            auto success = move_group_interface_.move();
            response->success = (success == moveit::core::MoveItErrorCode::SUCCESS);

            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Motion to joint target succeeded.");
            } else {
                RCLCPP_WARN(this->get_logger(), "Motion to joint target failed.");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Received joint array of incorrect size. Expected 6 joint values.");
            response->success = false;
        }
    }

     void handle_arm_move_cartesian_service_(
        const std::shared_ptr<call_m_custom_msgs::srv::ArmMoveCartesian::Request> request,
        std::shared_ptr<call_m_custom_msgs::srv::ArmMoveCartesian::Response> response
    ) {
        std::vector<geometry_msgs::msg::Pose> waypoints;
        for (const auto& pose : request->waypoints.poses) {
            waypoints.push_back(pose);
        }

        // Plan Cartesian path with waypoints
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.01;      // Step size in meters
        const double jump_threshold = 0.0; // Prevent large jumps
        double fraction = move_group_interface_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        // Check if the Cartesian path was planned successfully
        if (fraction > 0.9) {
            // Scale trajectory time to slow down motion
            double slow_down_factor = 5.0; // Adjust this factor to control the speed (0.5 = half speed)
            scale_trajectory_time(trajectory, slow_down_factor);

            // Execute the trajectory
            moveit::core::MoveItErrorCode success = move_group_interface_.execute(trajectory);
            response->success = (success == moveit::core::MoveItErrorCode::SUCCESS);
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Cartesian path executed successfully.");
            } else {
                RCLCPP_WARN(this->get_logger(), "Execution of Cartesian path failed.");
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to plan complete Cartesian path. Only achieved %.2f%% of the path.", fraction * 100.0);
            response->success = false;
        }
    }

    // Function to scale trajectory time
    void scale_trajectory_time(moveit_msgs::msg::RobotTrajectory &trajectory, double scale_factor) {
        for (auto &point : trajectory.joint_trajectory.points) {
            // Scale time
            rclcpp::Duration original_time(point.time_from_start.sec, point.time_from_start.nanosec);
            auto scaled_time = rclcpp::Duration::from_seconds(original_time.seconds() * scale_factor);
            point.time_from_start.sec = scaled_time.seconds();
            point.time_from_start.nanosec = scaled_time.nanoseconds() % 1'000'000'000;

            // Optionally scale velocities and accelerations for smoother execution
            for (auto &vel : point.velocities) {
                vel /= scale_factor;
            }
            for (auto &acc : point.accelerations) {
                acc /= scale_factor;
            }
        }
    }

    void handle_get_joint_values_service(const std::shared_ptr<call_m_custom_msgs::srv::GetJointValues::Request> request,
    std::shared_ptr<call_m_custom_msgs::srv::GetJointValues::Response> response){
          // Retrieve the current joint values from MoveIt
        std::vector<double> joint_values;
        move_group_interface_.getCurrentState()->copyJointGroupPositions(move_group_interface_.getCurrentState()->getRobotModel()->getJointModelGroup("ur_manipulator"), joint_values);

        // Fill the response with the joint values
        response->joints = joint_values;

        // Log the joint values for debugging
        RCLCPP_INFO(this->get_logger(), "Current Joint Values: ");
        for (size_t i = 0; i < joint_values.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "Joint %zu: %f", i, joint_values[i]);
        }
    }


    void handle_generate_collision_object_service(
    const std::shared_ptr<call_m_custom_msgs::srv::GenerateCollisionObject::Request> request,
    std::shared_ptr<call_m_custom_msgs::srv::GenerateCollisionObject::Response> response
    ) {
        // Create a CollisionObject
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.id = request->object_id;
        collision_object.header.frame_id = "base";

        // Define the shape of the object as a box with specified dimensions
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        primitive.dimensions.resize(3); // Length, width, and depth
        primitive.dimensions[0] = request->length; // Length
        primitive.dimensions[1] = request->width;  // Width
        primitive.dimensions[2] = request->depth;  // Depth

        // Assign the pose received in the request
        geometry_msgs::msg::Pose object_pose = request->pose;

        // Add the primitive shape and pose to the collision object
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(object_pose);
        collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

        // Apply the collision object to the planning scene
        planning_scene_interface_.applyCollisionObject(collision_object);

        // Confirm successful addition of the object
        response->success = true;
        RCLCPP_INFO(this->get_logger(), 
            "Collision object [%s] added with dimensions [L: %.2f, W: %.2f, D: %.2f].", 
            request->object_id.c_str(), 
            request->length, 
            request->width, 
            request->depth
        );
    }


    void handle_remove_collision_object_service(
        const std::shared_ptr<call_m_custom_msgs::srv::RemoveCollisionObject::Request> request,
        std::shared_ptr<call_m_custom_msgs::srv::RemoveCollisionObject::Response> response
    ) {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.id = request->object_id;  // Remove based on the object id

        // Set operation to REMOVE
        collision_object.operation = collision_object.REMOVE;

        // Apply the collision object to the planning scene
        planning_scene_interface_.applyCollisionObject(collision_object);

        // Confirm successful removal of the object
        response->success = true;
        RCLCPP_INFO(this->get_logger(), "Collision object '%s' removed from planning scene.", request->object_id.c_str());
    }

    void handle_get_end_effector_pose_service(
    const std::shared_ptr<call_m_custom_msgs::srv::GetEndEffectorPose::Request> request,
    std::shared_ptr<call_m_custom_msgs::srv::GetEndEffectorPose::Response> response
) {
    // Create a tf2 buffer and listener with the required clock (shared pointer)
    static rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    static tf2_ros::Buffer tf_buffer(clock);
    static tf2_ros::TransformListener tf_listener(tf_buffer);

    try {
        // Wait for the transform from 'base' to 'tool0'
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped = tf_buffer.lookupTransform(
            "base", "tool0", tf2::TimePointZero, std::chrono::seconds(1)
        );

        // Extract position and orientation
        auto translation = transform_stamped.transform.translation;
        auto rotation = transform_stamped.transform.rotation;

        // Log the transform for debugging
        RCLCPP_INFO(this->get_logger(), "Transform from 'base' to 'tool0':");
        RCLCPP_INFO(this->get_logger(), "Position: x=%.3f, y=%.3f, z=%.3f", 
                    translation.x, translation.y, translation.z);
        RCLCPP_INFO(this->get_logger(), "Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f", 
                    rotation.x, rotation.y, rotation.z, rotation.w);

        // Populate the response
        response->pose.position.x = translation.x;
        response->pose.position.y = translation.y;
        response->pose.position.z = translation.z;
        response->pose.orientation.x = rotation.x;
        response->pose.orientation.y = rotation.y;
        response->pose.orientation.z = rotation.z;
        response->pose.orientation.w = rotation.w;

        RCLCPP_INFO(this->get_logger(), "Successfully retrieved end effector pose.");
    } catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Could not transform 'base' to 'tool0': %s", ex.what());
    }
}


    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface move_group_interface_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_;
    rclcpp::Service<call_m_custom_msgs::srv::ArmMoveToPose>::SharedPtr arm_move_to_pose_service_;
    rclcpp::Service<call_m_custom_msgs::srv::ArmMoveToJoints>::SharedPtr arm_move_to_joints_service_;
    rclcpp::Service<call_m_custom_msgs::srv::GenerateCollisionObject>::SharedPtr generate_collision_object_service_;
    rclcpp::Service<call_m_custom_msgs::srv::RemoveCollisionObject>::SharedPtr remove_collision_object_service;
    rclcpp::Service<call_m_custom_msgs::srv::ArmMoveCartesian>::SharedPtr arm_move_cartesian_service_;
    rclcpp::Service<call_m_custom_msgs::srv::GetEndEffectorPose>::SharedPtr get_end_effector_pose_service_;
    rclcpp::Service<call_m_custom_msgs::srv::GetJointValues>::SharedPtr get_joint_values_service_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread executor_thread_;
};

// Main function
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.use_intra_process_comms(false);

  auto node = std::make_shared<ArmServerNode>(node_options);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
