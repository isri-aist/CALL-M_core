#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "servo3moog.h"
#include "kinema.h"

using std::placeholders::_1;

class Bot_control_driver : public rclcpp::Node
{
  public:
    Bot_control_driver(): Node("bot_control_driver_node")
    {
        RCLCPP_INFO(this->get_logger(), "Sarted, connecting motor...");
        servomotor_setup();
        rotor_rad_p_sec[0] = 0.0;
        rotor_rad_p_sec[1] = 0.0;
        rotor_rad_p_sec[2] = 0.0;
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

        if(apply_cmd && armed)
        {
            des_velx = this->vx;
            des_vely = this->vy;
            des_velw = this->w;
            RCLCPP_INFO(this->get_logger(), "Command applied");
        }
        
        if(!armed){
            RCLCPP_INFO(this->get_logger(), "Unarmed, Waiting commands...");
        }

        RCLCPP_INFO(this->get_logger(), "Vx:%.2f Vy:%.2f Vw:%.2f ",des_velx,des_vely,des_velw);

        // convert vehicle vel. -> rotor vel.
        vel2rotor (this->rotor_rad_p_sec, des_velx, des_vely, des_velw);

        // move motor
        this->rotor_rad_per_sec_gain = servo3motor_loop (this->rotor_rad_p_sec, this->target_motor_rpm, this->real_motor_rpm);
    }

    //global variables
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription; 
    float rotor_rad_p_sec[3] = {0.0, 0.0, 0.0}; // rev. per sec
    float vx = 0;
    float vy = 0;
    int iteration_to_reset = 200;
    float w = 0;
    int callback_active = 0;
    float rotor_rad_per_sec_gain = 1.0;
    float target_motor_rpm[6];
    float real_motor_rpm[6];
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Bot_control_driver>());
  rclcpp::shutdown();
  return 0;
}