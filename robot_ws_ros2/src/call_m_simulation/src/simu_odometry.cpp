#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rosgraph_msgs/msg/clock.hpp"  // Add the clock message

class SimuOdometry : public rclcpp::Node
{
public:
  SimuOdometry()
    : Node("simu_odometry_node")
  {
    // Subscribe to the model_states topic
    subscription_ = this->create_subscription<gazebo_msgs::msg::ModelStates>("gazebo/model_states", 10, std::bind(&SimuOdometry::TopicCallback, this, std::placeholders::_1));

    // Subscribe to the clock topic
    clock_subscription_ = this->create_subscription<rosgraph_msgs::msg::Clock>("/clock", 10, std::bind(&SimuOdometry::ClockCallback, this, std::placeholders::_1));

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

      // Publish Odometry message with the received timestamp
      publishOdometry(pose, twist, current_timestamp_);
    }
  }

  void ClockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
  {
    // Update the current timestamp when the clock message is received
    current_timestamp_ = msg->clock;
  }

  void publishOdometry(const geometry_msgs::msg::Pose &pose, const geometry_msgs::msg::Twist &twist, const builtin_interfaces::msg::Time &timestamp)
  {
    // Create and populate the Odometry message
    auto odometry_msg = nav_msgs::msg::Odometry();
    odometry_msg.header.stamp = timestamp;
    odometry_msg.header.frame_id = "odom";
    odometry_msg.child_frame_id = "base_link";

    // Set pose information
    odometry_msg.pose.pose = pose;
    odometry_msg.twist.twist = twist;

    // Publish the Odometry message
    odometry_publisher_->publish(odometry_msg);
  }

  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr subscription_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  builtin_interfaces::msg::Time current_timestamp_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimuOdometry>());
  rclcpp::shutdown();
  return 0;
}
