#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"

#include <math.h>
#include <cmath>
#include <string.h>

bool is_same(geometry_msgs::msg::Twist prev_msg,geometry_msgs::msg::Twist new_msg){
    bool result = prev_msg.linear.x == new_msg.linear.x && prev_msg.linear.y == new_msg.linear.y && prev_msg.angular.z == new_msg.angular.z;
    return result;
}

double val_sign(double val){
  if(val<0.0){return -1.0;}
  else{return 1.0;}
}

// Sawtooth function with values between -pi and pi
double sawtooth(double x) {
    double smaller_angle;
    double new_x = fmod(x,2*M_PI);
    if(abs(new_x)>M_PI){
      smaller_angle = -val_sign(new_x)*(2*M_PI-abs(new_x));
    }
    else{
      smaller_angle = new_x;
    }
    return smaller_angle;
}

double to_angle(double rad){
  return rad*180/M_PI;
}

double error_cmd(double prev_val,double wanted_val, double max_acc, double max_dcc,double dt){
  double error = wanted_val-prev_val;
  bool acceleration = (val_sign(wanted_val)==val_sign(prev_val) && abs(wanted_val)>abs(prev_val));
  if(acceleration){
    error = val_sign(error)*std::min(abs(error),dt*max_acc);
  }
  else{
    error = val_sign(error)*std::min(abs(error),dt*max_dcc);
  }
  return error;
}

double alpha(double a, double w ,double vx, double vy ,bool linear,double k){
  double angle;
    if(!linear){
      if (w != 0){
        angle= a+val_sign(w)*M_PI/2; //just rotation speed wanted
      }
      else{
      angle = 0.0; //nothing wanted
      }
      
    }
    else{
      //RCLCPP_INFO(this->get_logger(),"a = %f",to_angle(a));
      double rot_part = a+val_sign(w)*M_PI/2; //wanted vector for rotation part
      //RCLCPP_INFO(this->get_logger(),"target_vect = %f",to_angle(rot_part));
      double lin_part = atan2(vy,vx); //wanted vector for linear part
      //RCLCPP_INFO(this->get_logger(),"current_vect = %f",to_angle(lin_part));
      double error = sawtooth(rot_part-lin_part); //angle correction needed to pass from linear to rotation mouvement
      //RCLCPP_INFO(this->get_logger(),"error = %f",to_angle(error));

      angle = lin_part + k*abs(w)*error; //we apply the rotation part proportinonnaly to w command
      //RCLCPP_INFO(this->get_logger(),"final = %f",to_angle(angle));
    }
    return angle;
}

class SimuBotDriver : public rclcpp::Node {
public:
  SimuBotDriver() : Node("simu_bot_driver_node") {
    publisher_cmd_wheels = create_publisher<std_msgs::msg::Float64MultiArray>("/wheels_cont/commands", 10);
    publisher_cmd_wheels_sup = create_publisher<std_msgs::msg::Float64MultiArray>("/wheels_sup_cont/commands", 10);
    publisher_cmd_cams = create_publisher<std_msgs::msg::Float64MultiArray>("/cams_cont/commands", 10);
    subscriber_cmd = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_apply", 10, std::bind(&SimuBotDriver::twistCallback, this, std::placeholders::_1));
    subscriber_cams_cmd = create_subscription<dynamixel_sdk_custom_interfaces::msg::SetPosition>("/set_position", 10, std::bind(&SimuBotDriver::cams_callback, this, std::placeholders::_1));

    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;
    clock = this->get_clock();
    t0 = clock->now();

    //timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&SimuBotDriver::publish_cmds, this));
    RCLCPP_INFO(this->get_logger(), "\033[%dm\033[2J\033[1;1f",0);
    RCLCPP_INFO(this->get_logger(), "SIMULATED ROBOT DRIVER:");
    RCLCPP_INFO(this->get_logger(), "Waiting commands (vx,vy,w) on /cmd_vel_apply...");

  }

private:

  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    tf = clock->now();
    //RCLCPP_INFO(this->get_logger(),"\nticked...");
    if(!is_same(cmd_vel,*msg)){
      double former_vals[3] = {cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.angular.z};
      double wanted_vals[3] = {msg->linear.x,msg->linear.y,msg->angular.z};

      //managing accelerations limits
      double dt = (tf-t0).seconds();
      double errors_vals[3];

      //max acc and decc are converted to percentage equivalence
      errors_vals[0] = error_cmd(former_vals[0],wanted_vals[0],MAX_AX/MAX_VX,MAX_DX/MAX_VX,dt);
      errors_vals[1] = error_cmd(former_vals[1],wanted_vals[1],MAX_AY/MAX_VY,MAX_DY/MAX_VY,dt);
      errors_vals[2] = error_cmd(former_vals[2],wanted_vals[2],MAX_AW/MAX_W,MAX_DW/MAX_W,dt);

      cmd_vel = *msg;
      cmd_vel.linear.x = former_vals[0]+errors_vals[0];
      cmd_vel.linear.y = former_vals[1]+errors_vals[1];
      cmd_vel.angular.z = former_vals[2]+errors_vals[2];
      RCLCPP_INFO(this->get_logger(), "\033[%dm\033[2J\033[1;1f",0);
      RCLCPP_INFO(this->get_logger(), "SIMULATED ROBOT DRIVER:");
      printf("\nWanted = %.2f,%.2f,%.2f",wanted_vals[0],wanted_vals[1],wanted_vals[2]);
      printf("\nFormer = %.2f,%.2f,%.2f",former_vals[0],former_vals[1],former_vals[2]);
      printf("\nErrors cliped = %.2f,%.2f,%.2f, dt = %.2f",errors_vals[0],errors_vals[1],errors_vals[2],dt);
      printf("\nCMD = %.2f,%.2f,%.2f",cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.angular.z);

      //publish new commands
      publish_cmds();
    }
    t0 = clock->now();
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
    double k = 1/MAX_W;

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

    //RCLCPP_INFO(this->get_logger(),"\nrot speeds = %.2f,%.2f,%.2f\nsup angles = %.2f,%.2f,%.2f", msg_wheels.data[0],msg_wheels.data[1],msg_wheels.data[2],msg_wheels_sup.data[0],msg_wheels_sup.data[1],msg_wheels_sup.data[2]);

    // Publish the messages
    publisher_cmd_wheels->publish(msg_wheels);
    publisher_cmd_wheels_sup->publish(msg_wheels_sup);
  }

  void cams_callback(const dynamixel_sdk_custom_interfaces::msg::SetPosition::SharedPtr msg){
    //msg->position //Between 1000 and 3000, 3000 = angle of pi/2 and 1000 = angle of -pi/2
    //msg->id //id = 2 : camera1 and id = 3 : camera2
    bool changed = false;
    if(msg->id == 2){
      changed = msg->position != cam1_angle;
      cam1_angle = ((msg->position-1000)*M_PI/2000)-M_PI/2;
    }
    else if(msg->id == 3){
      changed = msg->position != cam2_angle;
      cam2_angle = ((msg->position-1000)*M_PI/2000)-M_PI/2;;
    }
    if(changed){
      publish_cmds_cams();
    }
  }

  void publish_cmds_cams(){
    auto msg_cams = std_msgs::msg::Float64MultiArray();
    msg_cams.data = {cam1_angle, cam2_angle};
    publisher_cmd_cams->publish(msg_cams);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_cmd; 
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_cmd_wheels;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_cmd_wheels_sup;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_cmd_cams;
  rclcpp::Subscription<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr subscriber_cams_cmd;

  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Twist cmd_vel;
  float MAX_VX = 1.5; //m.s-1
  float MAX_VY = 2.0;
  float MAX_W = 1.5 * M_PI; //rad.s-1
  double MAX_AX= 0.5*MAX_VX; //should be >0 m.s-2 Acceleration
  double MAX_AY= 0.5*MAX_VY; //should be >0 m.s-2
  double MAX_AW= 0.5*MAX_W; //should be >0 rad.s-2
  double MAX_DX= 1*MAX_VX; //should be >0 m.s-2  Deceleration
  double MAX_DY= 1*MAX_VY; //should be >0 m.s-2
  double MAX_DW= 1*MAX_W; //should be >0 rad.s-2
  double cam1_angle = 0.0;
  double cam2_angle = 0.0;
  rclcpp::Clock::SharedPtr clock;
  rclcpp::Time t0;
  rclcpp::Time tf;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimuBotDriver>());
  rclcpp::shutdown();
  return 0;
}
