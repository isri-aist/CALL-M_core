#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <math.h>
#include <cmath>

bool is_same(geometry_msgs::msg::Twist prev_msg,geometry_msgs::msg::Twist new_msg){
    bool result = prev_msg.linear.x == new_msg.linear.x && prev_msg.linear.y == new_msg.linear.y && prev_msg.angular.z == new_msg.angular.z;
    return result;
}

double val_sign(double val){
  if(val<0.0){return -1.0;}
  else if(val>=0.0){return 1.0;}
}

double alpha(double a, double w ,double vx, double vy ,bool linear,double k){
    if(!linear){
      if (w != 0){
        return a+val_sign(w)*M_PI/2; //just rotation speed wanted
      }
      else{
        return 0.0; //nothing wanted
      }
      
    }
    else{
      //RCLCPP_INFO(this->get_logger(),"k*w*(a+M_PI/2) = %f", k*w*(a+M_PI/2));
      //RCLCPP_INFO(this->get_logger(),"atan2(vy,vx) = %f", atan2(vy,vx));
      //RCLCPP_INFO(this->get_logger(),"final = %f", (k*w*(a+M_PI/2)+atan2(vy,vx)));
      return (k*abs(w)*(a+val_sign(w)*M_PI/2)+atan2(vy,vx)); //linear + rotation movement wanted
    }
}

class SimuBotDriver : public rclcpp::Node {
public:
  SimuBotDriver() : Node("simu_bot_driver_node") {
    publisher_cmd_wheels = create_publisher<std_msgs::msg::Float64MultiArray>("/wheels_cont/commands", 10);
    publisher_cmd_wheels_sup = create_publisher<std_msgs::msg::Float64MultiArray>("/wheels_sup_cont/commands", 10);
    subscriber_cmd = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_apply", 10, std::bind(&SimuBotDriver::twistCallback, this, std::placeholders::_1));

    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;

    timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&SimuBotDriver::publish_cmds, this));

  }

private:
  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    //RCLCPP_INFO(this->get_logger(),"\nticked...");
    if(!is_same(cmd_vel,*msg)){
      cmd_vel = *msg;
      //RCLCPP_INFO(this->get_logger(),"\nCMD = %.2f,%.2f,%.2f",cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.angular.z);
      }
  }

  void publish_cmds(){
    // Create a messages
    auto msg_wheels = std_msgs::msg::Float64MultiArray();
    auto msg_wheels_sup = std_msgs::msg::Float64MultiArray();

    //convert linear and angular command into ones adapted to robot

    //datas
    double a1 = (4*M_PI/3)-M_PI/2; //wheel and support 1 position
    double a2 = -M_PI/2;
    double a3 = (2*M_PI/3)-M_PI/2;
    double r = 0.05;
    double triangle_lenght = 0.32;
    double R = sqrt(((3*(pow(triangle_lenght,4)))/16)+(pow(triangle_lenght,2))/4);
    double k = 0.07;

    //commands
    double vx = cmd_vel.linear.x*MAX_VX;
    double vy = cmd_vel.linear.y*MAX_VY;
    double w = cmd_vel.angular.z*MAX_W;
    bool linear = sqrt((pow(vx,2)) + (pow(vy,2))) != 0;

    //compute angles wanted
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

    double w2 = V/r; //rotation speed wanted

    // Set the data values
    msg_wheels.data = {w2, w2, w2}; 
    msg_wheels_sup.data = {alpha1, alpha2, alpha3};

    //RCLCPP_INFO(this->get_logger(),"\nspeeds = %.2f,%.2f,%.2f\nangles = %.2f,%.2f,%.2f", msg_wheels.data[0],msg_wheels.data[1],msg_wheels.data[2],msg_wheels_sup.data[0],msg_wheels_sup.data[1],msg_wheels_sup.data[2]);

    // Publish the messages
    publisher_cmd_wheels->publish(msg_wheels);
    publisher_cmd_wheels_sup->publish(msg_wheels_sup);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_cmd; 
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_cmd_wheels;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_cmd_wheels_sup;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Twist cmd_vel;
  float MAX_VX = 1.0;
  float MAX_VY = 1.0;
  float MAX_W = 1.5 * M_PI;

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimuBotDriver>());
  rclcpp::shutdown();
  return 0;
}
