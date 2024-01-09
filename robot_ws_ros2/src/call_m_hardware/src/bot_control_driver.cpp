#include <chrono>
#include <memory>
#include<unistd.h>
#include<math.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "servo3moog.h"
#include "kinema.h"

using std::placeholders::_1;

double constrain(double value, double min, double MAX)
{
    if (MAX >= value && value >= min) {
	return value;
    } else if (value > MAX) {
	return MAX;
    } else if (value < min) {
	return min;
    }
    return min;
}

void rotate_vect(double &vx, double &vy, double alpha){
    double temp_vx = vx;
    double temp_vy = vy;
    vx = cos(alpha)*temp_vx+sin(alpha)*temp_vy;
    vy = -sin(alpha)*temp_vx+cos(alpha)*temp_vy;
    //RCLCPP_INFO(this->get_logger(),"temp: vx=%.2f, vy = %.2f",temp_vx,temp_vy);
    //RCLCPP_INFO(this->get_logger(),"set: vx=%.2f, vy = %.2f",vx,vy);
}

class Bot_control_driver : public rclcpp::Node
{
  public:
    Bot_control_driver(): Node("bot_control_driver_node")
    {
        initialize_params();
        refresh_params();
        RCLCPP_INFO(this->get_logger(), "Sarted, connecting to motors...");
        rotor_rad_p_sec[0] = 0.0;
        rotor_rad_p_sec[1] = 0.0;
        rotor_rad_p_sec[2] = 0.0;
        while(servomotor_setup(device_name.c_str()) <0){
            //RCLCPP_ERROR(this->get_logger(), "Port tested: "+device_name);
            RCLCPP_ERROR(this->get_logger(), "Please plug servomotors and restart..");
            sleep(1);
        }
        RCLCPP_INFO(this->get_logger(), "Set up finished.");
        subscription= this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_apply", 10, std::bind(&Bot_control_driver::topic_callback, this, _1));
        timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&Bot_control_driver::timeout_secu, this));
        RCLCPP_INFO(this->get_logger(), "\033[%dm\033[2J\033[1;1f",0);
        RCLCPP_INFO(this->get_logger(), "ROBOT DRIVER:");
        RCLCPP_INFO(this->get_logger(), "Waiting commands...");
    }

  private:
    //functions
    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        callback_active = 0; //update the security variable
        //MAIN CODE to get commands
        if(msg->linear.x != this->vx || msg->linear.y != this->vy || msg->angular.z != this->w)
        {
            //RCLCPP_INFO(this->get_logger(), "Keyboard new commands: vx:'%f' vy:'%f' w:'%f' ", msg->vx, msg->vy, msg->w);
            this->vx = msg->linear.x;
            this->vy = msg->linear.y;
            this->w = msg->angular.z;
            apply_commands(true);
        }
    }

    void timeout_secu(){
        callback_active ++;
        if(callback_active >= iteration_to_reset){
            apply_commands(false);
            callback_active = 0;//just to avoid to hit int values limit on computer, we restart the count so the bot is reset continiously each amount of time
        }
    }

    void apply_commands(bool armed)
    {
        float des_velx = 0.0;
        float des_vely = 0.0;
        float des_velw = 0.0;

        RCLCPP_INFO(this->get_logger(), "\033[%dm\033[2J\033[1;1f",0);
        RCLCPP_INFO(this->get_logger(), "ROBOT DRIVER:");

        bool apply_cmd = callback_active < iteration_to_reset;

        //received commands should be percentage (0->1)
        if(apply_cmd && armed)
        {
            des_velx = this->vx*MAX_VX;
            des_vely = this->vy*MAX_VY;
            des_velw = this->w*MAX_W;
            RCLCPP_INFO(this->get_logger(), "Command applied");
        }
        
        //security
        des_velx = constrain(des_velx, -MAX_VX, MAX_VX);
        des_vely = constrain(des_vely, -MAX_VY, MAX_VY);
        des_velw = constrain(des_velw, -MAX_W, MAX_W);

        if(!armed){
            RCLCPP_INFO(this->get_logger(), "Unarmed, Waiting commands...");
        }

        RCLCPP_INFO(this->get_logger(), "Vx:%.2f Vy:%.2f Vw:%.2f ",des_velx,des_vely,des_velw);

        //we rotate the speed vector to match wanted frame for the robot
        double new_des_velx = -des_velx; //we flip x axis
        double new_des_vely = des_vely;
        double alpha = M_PI/3;//5*M_PI/6;
        rotate_vect(new_des_velx, new_des_vely, alpha);

        //RCLCPP_INFO(this->get_logger(), "Vx:%.2f Vy:%.2f Vw:%.2f ",new_des_velx,new_des_vely,des_velw);

        // convert vehicle vel. -> rotor vel.
        vel2rotor(this->rotor_rad_p_sec, new_des_velx, new_des_vely, des_velw);

        // move motor
        this->rotor_rad_per_sec_gain = servo3motor_loop (this->rotor_rad_p_sec, this->target_motor_rpm, this->real_motor_rpm);
    }

    void initialize_params(){
      this->declare_parameter("device_name"); 
    }

    void refresh_params(){
        this->get_parameter_or<std::string>("device_name",device_name,"/dev/ttyUSB0");
    }

    //global variables
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription; 
    float rotor_rad_p_sec[3] = {0.0, 0.0, 0.0}; // rev. per sec
    float vx = 0;
    float vy = 0;
    int iteration_to_reset = 100;
    float w = 0;
    int callback_active = 0;
    float rotor_rad_per_sec_gain = 1.0;
    float target_motor_rpm[6];
    float real_motor_rpm[6];
    rclcpp::TimerBase::SharedPtr timer_;
    std::string device_name;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Bot_control_driver>());
  rclcpp::shutdown();
  return 0;
}