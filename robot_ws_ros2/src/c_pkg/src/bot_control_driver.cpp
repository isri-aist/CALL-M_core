#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "c_pkg/msg/state_vector.hpp"

#include "servo3moog.h"
#include "kinema.h"
#include "func.h"

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
        subscription_keyboard= this->create_subscription<c_pkg::msg::StateVector>("keyboard_bot_command", 10, std::bind(&Bot_control_driver::topic_callback_keyboard, this, _1));
    }

  private:
    //functions
    void topic_callback_keyboard(const c_pkg::msg::StateVector::SharedPtr msg)
    {
        //MAIN CODE to get commands from keyboard
        if(msg->vx != this->keyboard_vx || msg->vy != this->keyboard_vy || msg->w != this->keyboard_w)
        {
            RCLCPP_INFO(this->get_logger(), "Keyboard new commands: vx:'%f' vy:'%f' w:'%f' ", msg->vx, msg->vy, msg->w);
            this->keyboard_vx = msg->vx;
            this->keyboard_vy = msg->vy;
            this->keyboard_w = msg->w;
            apply_commands();
        }
    }

    void apply_commands()
    {
        bool command_keyboard = this->keyboard_vx+this->keyboard_vy+this->keyboard_w != 0;
        bool command_joystick = this->joystick_vx+this->joystick_vy+this->joystick_w != 0;

        float des_velx = 0.0;
        float des_vely = 0.0;
        float des_velw = 0.0;

        //if we receive non nul commands from keyboard, we use keyboard commands whatever any other joysticks
        if(command_keyboard)
        {
            des_velx = this->keyboard_vx;
            des_vely = this->keyboard_vy;
            des_velw = this->keyboard_w;
        }
        else if(!command_keyboard && command_joystick)
        {
            des_velx = this->joystick_vx;
            des_vely = this->joystick_vy;
            des_velw = this->joystick_w;
        }

        des_velx = constrain(des_velx, -MAX_VX, MAX_VX);
        des_vely = constrain (des_vely, -MAX_VY, MAX_VY);
        des_velw = constrain (des_velw, -MAX_W, MAX_W);

        // convert vehicle vel. -> rotor vel.
        vel2rotor (this->rotor_rad_p_sec, des_velx, des_vely, des_velw);

        // move motor
        this->rotor_rad_per_sec_gain = servo3motor_loop (this->rotor_rad_p_sec, this->target_motor_rpm, this->real_motor_rpm);
    }

    //global variables
    rclcpp::Subscription<c_pkg::msg::StateVector>::SharedPtr subscription_keyboard;
    float rotor_rad_p_sec[3] = {0.0, 0.0, 0.0}; // rev. per sec
    float keyboard_vx = 0;
    float keyboard_vy = 0;
    float keyboard_w = 0;
    float joystick_vx = 0;
    float joystick_vy = 0;
    float joystick_w = 0;
    float rotor_rad_per_sec_gain = 1.0;
    float target_motor_rpm[6];
    float real_motor_rpm[6];
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Bot_control_driver>());
  rclcpp::shutdown();
  return 0;
}