#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"

#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <sstream>
#include <termios.h>
#include "conio.h"

using namespace std;
using namespace std::chrono_literals;

/****************************/
/*       motor      ball    */
/* ball(3)--[2]--------(2)  */
/*       \             /    */
/*        \     motor[1]    */
/*         \         /      */
/*          \       / Y     */
/*           \     /  ^     */
/*      motor[3]  /   |     */
/*             \ /    |     */
/*         ball(1)    +-> X */
/****************************/

void flush_input_buffer() {
    struct termios term;
    tcgetattr(STDIN_FILENO, &term);
    while (_kbhit()) {
        char c = _getche();
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &term);
}

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

class Keyboard_control:public rclcpp::Node
{
    public:
        Keyboard_control():Node("keyboard_control_node")
        {
            //create pubisher that will publish message of type [vx,vy,w]
            publisher_motor = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_teleop_key", 10);
            //publisher for camera servos
            publisher_servo_cam = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>("/set_position", 10);
            //create timer that will call repetitively the function timer_callback
            timer_ = this->create_wall_timer(dt, std::bind(&Keyboard_control::timer_callback, this));
            RCLCPP_INFO(this->get_logger(),"\nkeyboard_control_node started...");
            show_msg();
        } 

    private:
        //global variables    
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_motor;
        rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr publisher_servo_cam;
        dynamixel_sdk_custom_interfaces::msg::SetPosition servo_cam_msg;
        std::chrono::milliseconds dt = 100ms;
        int key_code = 0;
        float linear_vel = 0.1;
        float rotatio_vel = 0.1;
        double max_lin_sp = 1; //we are working with values between -1 and 1 = percentage
        double max_rot_sp = 1; //we are working with values between -1 and 1 = percentage
        double lin_incr = 0.1;
        double rot_incr = 0.1;
        bool auto_mode = false;
        float chrono = 0.0;
        int step = 0;
        float sec_dt = 0.1; //loop dt in seconds
        float distance_t = 20*sec_dt;
        int servo_cam1_id=2;
        int servo_cam2_id=3;
        int servo_cam1_pos = 2000; //1000 to 3000 (1000 = looking down)
        int servo_cam2_pos = 2000;
        int servo_cam_incr = 200;

        //Functions
        void show_msg(){
        printf( "\033[%dm\033[2J\033[1;1f",0);
        printf("KEYBOARD CONTROLS:\n\n");
        printf("  Q  W  E       T                U  I\n");
        printf("  A  S  D    F  G  H    SPACE    J  K\n\n");
        printf("Current max linear speed : %.2f  %",(this->linear_vel/max_lin_sp)*100);
        printf("\nCurrent max rotationnal speed: %.2f %",(this->rotatio_vel/max_rot_sp)*100);
        printf("\n\nQ/E: Rotation\n");
        printf("W/S: Y axis move\n");
        printf("A/D: X axis move\n");
        printf("T/G: Set linear speed\n");
        printf("F/H: Set rotational speed\n");
        printf("SPACE: Switch to auto mode\n");
        printf("U/J: Front Camera angle\n");
        printf("I/K: Back Camera angle\n");
        printf("\nKeyboard controls will overwrite any joysticks ones.\n");
        }

        void check_keyboard(){
        if ( _kbhit() ){
            this->key_code = _getche();
            this->auto_mode = false;
            flush_input_buffer(); //empty tampon memory
            show_msg();
        }
        else{
            this->key_code = -1;
        }
        /*stringstream strs;
        strs << this->key_code;
        string temp_str = strs.str();
        char* char_type = (char*) temp_str.c_str();
        printf(char_type);*/
        }

        void timer_callback()
        {   
            geometry_msgs::msg::Twist commands;

            float des_velx = 0.0;
            float des_vely = 0.0;
            float des_velw = 0.0;

            //MAIN PROGRAM TO COMPUTE SPEEDS

            check_keyboard();
            switch (this->key_code)
            {
            case 119: //W
                des_vely = -this->linear_vel;
                break;
            case 97: //A
                des_velx = -this->linear_vel;
                break;
            case 115: //S
                des_vely = this->linear_vel;
                break;
            case 100: //D
                des_velx = this->linear_vel;
                break;
            case 113: //Q
                des_velw = this->rotatio_vel;
                break;
            case 101: //E
                des_velw = -this->rotatio_vel;
                break;
            case 32: //space
                this->auto_mode = true;
                printf( "\033[%dm\033[2J\033[1;1f",0);
                printf("Press any key to stop Automatic mode...");
                break;
            case 116: //T
                this->linear_vel += lin_incr;        
                //show_msg();
                //printf("\b");
                break;
            case 102: //F
                this->rotatio_vel -= rot_incr;
                //show_msg();
                //printf("\b");
                break;
            case 104: //H
                this->rotatio_vel += rot_incr;
                //show_msg();
                //printf("\b");
                break;
            case 103: //G
                this->linear_vel -= lin_incr;
                //show_msg();
                //printf("\b");
                break;
            case 117: //U
                servo_cam1_pos = constrain(servo_cam1_pos-servo_cam_incr,1000,3000);
                servo_cam_msg.id = servo_cam1_id;
                servo_cam_msg.position = servo_cam1_pos;
                publish_cam_cmd();
                break;
            case 106: //J
                servo_cam1_pos = constrain(servo_cam1_pos+servo_cam_incr,1000,3000);
                servo_cam_msg.id = servo_cam1_id;
                servo_cam_msg.position = servo_cam1_pos;
                publish_cam_cmd();
                break;
            case 105: //I
                servo_cam2_pos = constrain(servo_cam2_pos-servo_cam_incr,1000,3000);
                servo_cam_msg.id = servo_cam2_id;
                servo_cam_msg.position = servo_cam2_pos;
                publish_cam_cmd();
                break;
            case 107: //K
                servo_cam2_pos = constrain(servo_cam2_pos+servo_cam_incr,1000,3000);
                servo_cam_msg.id = servo_cam2_id;
                servo_cam_msg.position = servo_cam2_pos;
                publish_cam_cmd();
                break;
            default:
                des_velx = 0;
                des_vely = 0;
                des_velw = 0;
                break;
            }
            
            if (auto_mode){
                switch (step)
                {
                case 0:
                des_velx = 0;
                des_vely = -0.1;
                des_velw = 0;
                this->chrono += this->sec_dt;
                if(this->chrono > this->distance_t){
                    this->chrono = 0;
                    this->step ++;
                }
                break;
                case 1:
                des_velx = 0.1;
                des_vely = 0;
                des_velw = 0;
                this->chrono += this->sec_dt;
                if(this->chrono > this->distance_t){
                    this->chrono = 0;
                    this->step ++;
                }
                break;
                case 2:
                des_velx = 0;
                des_vely = 0.1;
                des_velw = 0;
                this->chrono += this->sec_dt;
                if(this->chrono > this->distance_t){
                    this->chrono = 0;
                    this->step ++;
                }
                break;
                case 3:
                des_velx = -0.1;
                des_vely = 0;
                des_velw = 0;
                this->chrono += this->sec_dt;
                if(this->chrono > this->distance_t){
                    this->chrono = 0;
                    this->step = 0;
                }
                break;        
                default:
                break;
                }
            }
            else{
                this->chrono = 0;
                this->step = 0;
            }

            //END OF MAIN
            this->linear_vel = constrain (this->linear_vel, 0, max_lin_sp);
            this->rotatio_vel = constrain (this->rotatio_vel, 0, max_rot_sp);
            /*if(!auto_mode){
                show_msg();
            }*/

            //filling StateVector Message
            if (this->linear_vel > 0){
                commands.linear.x = des_velx;
                commands.linear.y = des_vely;
            }
            else{
                commands.linear.x = 0.0;
                commands.linear.y = 0.0;
            }
            if (this->rotatio_vel > 0){
                commands.angular.z = des_velw;
            }
            else{
                commands.angular.z = 0.0;
            }

            //publish commands
            publisher_motor->publish(commands);
        }

        void publish_cam_cmd()
        {
            //printf("Publish: id: %d, pos: %d",servo_cam_msg.id,servo_cam_msg.position);
            publisher_servo_cam->publish(servo_cam_msg);
        }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Keyboard_control>());
    rclcpp::shutdown();
    return 0;
}