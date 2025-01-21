#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"  // Use PoseArray for multiple grasp poses
#include "call_m_custom_msgs/srv/grasp_detection.hpp"  // Include your custom service
#include <gpd/grasp_detector.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

// Include util for Cloud
#include <gpd/util/cloud.h>

namespace gpd_ros2 {

class GraspDetectionNode : public rclcpp::Node {
public:
  GraspDetectionNode() : Node("grasp_detection_node") {
    // Create the service
    service_ = this->create_service<call_m_custom_msgs::srv::GraspDetection>(
      "detect_grasps", std::bind(&GraspDetectionNode::detect_grasps_callback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Grasp pose detection server ready. Call /detect_grasps and give the path towards the PCD file to detect grasps, otherwise default path will be used.");

  }

private:
  bool checkFileExists(const std::string &file_name) {
    std::ifstream file(file_name);
    return file.is_open();
  }

  void detect_grasps_callback(
    const std::shared_ptr<call_m_custom_msgs::srv::GraspDetection::Request> request,
    std::shared_ptr<call_m_custom_msgs::srv::GraspDetection::Response> response) {
    
    // Retrieve the configuration and PCD file paths from the request
    std::string cfg_path = request->cfg_path;
    std::string pcd_filename = request->pcd_path;

    // Check if the configuration file exists
    if (!checkFileExists(cfg_path)) {
        RCLCPP_ERROR(this->get_logger(), "Configuration file not found: %s", cfg_path.c_str());
        response->success = false;
        response->message = "Configuration file not found";
        return;
    }

    // Check if the PCD file exists
    if (!checkFileExists(pcd_filename)) {
        RCLCPP_ERROR(this->get_logger(), "PCD file not found: %s", pcd_filename.c_str());
        response->success = false;
        response->message = "PCD file not found";
        return;
    }
    // Load the point cloud using the util::Cloud class
    gpd::util::Cloud cloud(pcd_filename, Eigen::Matrix3Xd::Zero(3, 1));
    if (cloud.getCloudOriginal()->size() == 0) {
        RCLCPP_ERROR(this->get_logger(), "Point cloud is empty or could not be loaded");
        response->success = false;
        response->message = "Point cloud is empty or invalid";
        return;
    }

    // Load configuration and initialize GraspDetector
    gpd::GraspDetector detector(cfg_path);
    detector.preprocessPointCloud(cloud);

    // Convert detected grasp poses to ROS2 message
    geometry_msgs::msg::PoseArray grasp_poses_msg;
    auto grasps = detector.detectGrasps(cloud); // Retrieve grasps
    
    if (grasps.empty()) {
    RCLCPP_WARN(this->get_logger(), "No grasp poses detected.");
    response->success = false;
    response->message = "No grasp poses found.";
    return;
}

for (const auto &grasp : grasps) {
    geometry_msgs::msg::Pose pose;

    // Position of the grasp
    Eigen::Vector3d position = grasp->getPosition();
    pose.position.x = position.x();
    pose.position.y = position.y();
    pose.position.z = position.z();

    // Orientation of the grasp (rotation matrix to quaternion)
    Eigen::Matrix3d orientation_matrix = grasp->getOrientation();

    // // Extract the current yaw from the GPD's orientation matrix
    // double current_yaw = atan2(orientation_matrix(1, 0), orientation_matrix(0, 0));

    // // Desired roll and pitch
    // Eigen::AngleAxisd roll(M_PI, Eigen::Vector3d::UnitX()); // 180°
    // Eigen::AngleAxisd pitch(-50 * M_PI / 180.0, Eigen::Vector3d::UnitY()); // -50°
    // Eigen::AngleAxisd yaw(current_yaw, Eigen::Vector3d::UnitZ()); // Keep the detected yaw

    // // Combine roll, pitch, and yaw into a single rotation matrix
    // Eigen::Matrix3d desired_orientation = (yaw * pitch * roll).toRotationMatrix();

    // Update the orientation matrix
    // orientation_matrix = desired_orientation;

    // Print the orientation matrix
    RCLCPP_INFO(this->get_logger(), "Orientation matrix:");
    for (int i = 0; i < 3; ++i) {
        std::stringstream row;
        row << orientation_matrix(i, 0) << " " 
            << orientation_matrix(i, 1) << " " 
            << orientation_matrix(i, 2);
        RCLCPP_INFO(this->get_logger(), row.str().c_str());
    }

    // Convert the updated rotation matrix to quaternion for the Pose message
    Eigen::Quaterniond orientation(orientation_matrix);
    pose.orientation.x = orientation.x();
    pose.orientation.y = orientation.y();
    pose.orientation.z = orientation.z();
    pose.orientation.w = orientation.w();

    // Add the pose to the array
    grasp_poses_msg.poses.push_back(pose);
}




    // Set response
    response->success = true;
    response->message = "Grasp poses detected successfully";
    response->grasp_poses = grasp_poses_msg; // Return PoseArray of grasp poses
  }

  rclcpp::Service<call_m_custom_msgs::srv::GraspDetection>::SharedPtr service_;
  std::string config_file_;
};

}  // namespace gpd_ros2

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gpd_ros2::GraspDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
