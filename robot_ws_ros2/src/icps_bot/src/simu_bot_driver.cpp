#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "geometry_msgs/msg/twist.hpp"

bool is_same(geometry_msgs::msg::Twist prev_msg,geometry_msgs::msg::Twist new_msg){
    bool result = prev_msg.linear.x == new_msg.linear.x && prev_msg.linear.y == new_msg.linear.y && prev_msg.angular.z == new_msg.angular.z;
    return result;
}

class SimuBotDriver : public rclcpp::Node {
public:
  SimuBotDriver() : Node("simu_bot_driver_node") {
    publisher_cmd = create_publisher<trajectory_msgs::msg::JointTrajectory>("/set_joint_trajectory", 10);
    subscriber_cmd = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_apply", 10, std::bind(&SimuBotDriver::twistCallback, this, std::placeholders::_1));


    timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&SimuBotDriver::publishJointTrajectory, this));

  }

private:
  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if(!is_same(cmd_vel,*msg)){cmd_vel = *msg;}
  }

  void publishJointTrajectory() {
    auto message = std::make_unique<trajectory_msgs::msg::JointTrajectory>();
    message->header.frame_id = "base_link";
    message->joint_names = {"wheel_f_sup_joint", "wheel_bl_sup_joint", "wheel_br_sup_joint", "wheel_f_joint", "wheel_bl_joint", "wheel_br_joint"};
    trajectory_msgs::msg::JointTrajectoryPoint point;
    //convert linear and angular command into ones adapted to robot
    point.positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    //cmd_vel.linear.x
    //cmd_vel.linear.y
    //cmd_vel.angular.z

    message->points.push_back(point);

    publisher_cmd->publish(std::move(message));
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_cmd; 
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_cmd;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Twist cmd_vel;

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimuBotDriver>());
  rclcpp::shutdown();
  return 0;
}
