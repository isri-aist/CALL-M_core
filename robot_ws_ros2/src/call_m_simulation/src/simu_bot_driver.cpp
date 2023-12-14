#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <math.h>
#include <cmath>

double pi = 3.14159265359;

bool is_same(geometry_msgs::msg::Twist prev_msg,geometry_msgs::msg::Twist new_msg){
    bool result = prev_msg.linear.x == new_msg.linear.x && prev_msg.linear.y == new_msg.linear.y && prev_msg.angular.z == new_msg.angular.z;
    return result;
}

double alpha(double a, double w ,double vx, double vy ,bool linear,double k){
    if(!linear){
      if (w != 0){
        return (abs(w)/w)*(a+pi/2); //just rotation speed wanted
      }
      else{
        return 0.0; //nothing wanted
      }
      
    }
    else{
      //RCLCPP_INFO(this->get_logger(),"k*w*(a+pi/2) = %f", k*w*(a+pi/2));
      //RCLCPP_INFO(this->get_logger(),"atan2(vy,vx) = %f", atan2(vy,vx));
      //RCLCPP_INFO(this->get_logger(),"final = %f", (k*w*(a+pi/2)+atan2(vy,vx)));
      return (k*w*(a+pi/2)+atan2(vy,vx)); //linear + rotation movement wanted
    }
}

double euler(double var, double speed, double dt){
  double result = std::fmod(var + speed * dt, 2.0 * pi);
  return result;
}

class SimuBotDriver : public rclcpp::Node {
public:
  SimuBotDriver() : Node("simu_bot_driver_node") {
    publisher_cmd = create_publisher<trajectory_msgs::msg::JointTrajectory>("/set_joint_trajectory", 10);
    subscriber_cmd = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_apply", 10, std::bind(&SimuBotDriver::twistCallback, this, std::placeholders::_1));


    timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&SimuBotDriver::publishJointTrajectory, this));

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

    //datas
    double a1 = 0;
    double a2 = 2*pi/3;
    double a3 = 4*pi/3;
    double r = 0.05;
    double triangle_lenght = 0.32;
    double R = sqrt(((3*(pow(triangle_lenght,4)))/16)+(pow(triangle_lenght,2))/4);
    double k = 0.1;

    //commands
    double vx = cmd_vel.linear.x;
    double vy = cmd_vel.linear.y;
    double w = cmd_vel.angular.z;
    bool linear = sqrt((pow(vx,2)) + (pow(vy,2))) != 0;

    //compute
    double alpha1 = alpha(a1,w,vx,vy ,linear,k);
    double alpha2 = alpha(a2,w,vx,vy ,linear,k);
    double alpha3 = alpha(a3,w,vx,vy ,linear,k);

    double V = 0;
    if (linear){
      V = sqrt((pow(vx,2)) + (pow(vy,2)));
    }
    else{
      V = R*abs(w);
    }

    double w2 = V/r;

    //simulate speed with positions evolution for sphere wheels, EULER Integration
    double dt = (this->now() - last_time).seconds();
    last_pos1 = euler(last_pos1,w2,dt);
    last_pos2 = euler(last_pos2,w2,dt);
    last_pos3 = euler(last_pos3,w2,dt);
    last_time = this->now();

    point.positions = {alpha1, alpha2, alpha3, last_pos1, last_pos2, last_pos3};

    message->points.push_back(point);

    publisher_cmd->publish(std::move(message));
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_cmd; 
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_cmd;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Twist cmd_vel;
  double last_pos1 = 0;
  double last_pos2 = 0;
  double last_pos3 = 0;
  rclcpp::Time last_time = this->now();

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimuBotDriver>());
  rclcpp::shutdown();
  return 0;
}