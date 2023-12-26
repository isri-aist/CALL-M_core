#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "nav_msgs/msg/odometry.hpp"

class SimuOdometry : public rclcpp::Node
{
public:
  SimuOdometry()
    : Node("simu_odometry_node")
  {
    // Subscribe to the model_states topic
    subscription_ = this->create_subscription<gazebo_msgs::msg::ModelStates>("gazebo/model_states", 10, std::bind(&SimuOdometry::TopicCallback, this, std::placeholders::_1));
    // Initialize publisher
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_simu", 10);
  }

private:
  void TopicCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
  {
    // Find the index of the call_m_bot model
    auto it = std::find(msg->name.begin(), msg->name.end(), "call_m_bot");
    if (it != msg->name.end()) {
      size_t index = std::distance(msg->name.begin(), it);

      // Extract pose and twist data for call_m_bot
      geometry_msgs::msg::Pose pose = msg->pose[index];
      geometry_msgs::msg::Twist twist = msg->twist[index];

      //Print or process the extracted data as needed
      //RCLCPP_INFO(this->get_logger(), "call_m_bot Pose: (%f, %f, %f)", pose.position.x, pose.position.y, pose.position.z);
      //RCLCPP_INFO(this->get_logger(), "call_m_bot Twist: (%f, %f, %f)", twist.linear.x, twist.linear.y, twist.angular.z);
      
      //Speeds are in world frame, so we can not use them for ekf filter, we need them in robot's frame
      //so we use positions directly instead

      // Publish Odometry message
      publishOdometry(pose, twist);
    }
  }

  void publishOdometry(const geometry_msgs::msg::Pose &pose, const geometry_msgs::msg::Twist &twist)
  {
    // Create and populate the Odometry message
    auto odometry_msg = nav_msgs::msg::Odometry();
    odometry_msg.header.stamp = this->now();
    odometry_msg.header.frame_id = "odom";
    odometry_msg.child_frame_id = "base_link";

    // Set pose information
    odometry_msg.pose.pose = pose;
    odometry_msg.twist.twist = twist;

    // Publish the Odometry message
    odometry_publisher_->publish(odometry_msg);
  }

  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimuOdometry>());
  rclcpp::shutdown();
  return 0;
}