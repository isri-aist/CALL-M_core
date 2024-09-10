#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rosgraph_msgs/msg/clock.hpp"  // Add the clock message

void rotate_vect(float &vx, float &vy, float alpha){
    float temp_vx = vx;
    float temp_vy = vy;
    vx = cos(alpha)*temp_vx+sin(alpha)*temp_vy;
    vy = -sin(alpha)*temp_vx+cos(alpha)*temp_vy;
    //RCLCPP_INFO(this->get_logger(),"temp: vx=%.2f, vy = %.2f",temp_vx,temp_vy);
    //RCLCPP_INFO(this->get_logger(),"set: vx=%.2f, vy = %.2f",vx,vy);
}


class SimuOdometry : public rclcpp::Node
{
public:
  SimuOdometry()
    : Node("simu_odometry_node")
  {
    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());

    // Subscribe to the model_states topic
    subscription_ = this->create_subscription<gazebo_msgs::msg::ModelStates>("gazebo/model_states", sensor_qos, std::bind(&SimuOdometry::TopicCallback, this, std::placeholders::_1));

    // Subscribe to the clock topic
    clock_subscription_ = this->create_subscription<rosgraph_msgs::msg::Clock>("/clock", sensor_qos, std::bind(&SimuOdometry::ClockCallback, this, std::placeholders::_1));

    // Initialize publisher
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_triorb", default_qos);
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

      /*// put twist in robot frame, because they are in world frame initially
      float alpha = quaternion_to_euler(pose.orientation).z;
      float new_vx = twist.linear.x;
      float new_vy = twist.linear.y;
      rotate_vect(new_vx, new_vy, -alpha); //we adjust rotation
      twist.linear.x = new_vx; 
      twist.linear.y = new_vy;*/

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

    double covariance_value = 0.0;
    for (size_t i = 0; i < 36; ++i) {
        odometry_msg.pose.covariance[i] = covariance_value;
        odometry_msg.twist.covariance[i] = covariance_value;
    }

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
