#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "c_pkg/msg/state_vector.hpp"

#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <sstream>
#include <termios.h>
#include "conio.h"
#include "func.h"
#include "kinema.h"

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

class Keyboard_control:public rclcpp::Node
{
    public:
        Keyboard_control():Node("keyboard_control_node")
        {
            //create pubisher that will publish message of type [vx,vy,w]
            publisher_ = this->create_publisher<c_pkg::msg::StateVector>("keyboard_bot_command", 10);
            //create timer that will call repetitively the function timer_callback
            timer_ = this->create_wall_timer(dt, std::bind(&Keyboard_control::timer_callback, this));
            RCLCPP_INFO(this->get_logger(),"\nkeyboard_control_node started...");
            show_msg();
        } 

    private:
        //global variables    
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<c_pkg::msg::StateVector>::SharedPtr publisher_;
        std::chrono::milliseconds dt = 100ms;
        int key_code = 0;
        float linear_vel = 0.1;
        float rotatio_vel = 0.1 * M_PI;
        bool auto_mode = false;
        float chrono = 0.0;
        int step = 0;
        float sec_dt = 0.1; //loop dt in seconds
        float distance_t = 20*sec_dt;

        //Functions
        void show_msg(){
        printf( "\033[%dm\033[2J\033[1;1f",0);
        printf("KEYBOARD CONTROLS:\n\n");
        printf("  Q  W  E       T\n");
        printf("  A  S  D    F  G  H    SPACE\n\n");
        printf("Current linear speed: %f",this->linear_vel);
        printf("\nCurrent rotationnal speed: %f",this->rotatio_vel);
        printf("\n\nQ/E: Rotation\n");
        printf("W/S: Y axis move\n");
        printf("A/D: X axis move\n");
        printf("T/G: Set linear speed\n");
        printf("F/H: Set rotational speed\n");
        printf("SPACE: Switch to auto mode\n");
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
            auto commands=c_pkg::msg::StateVector();

            float vx=0.0;
            float vy=0.0;
            float w=0.0;
            float des_velx = 0.0;
            float des_vely = 0.0;
            float des_velw = 0.0;

            //MAIN PROGRAM TO COMPUTE SPEEDS
            this->linear_vel = constrain (this->linear_vel, 0, MAX_VX);
            this->rotatio_vel = constrain (this->rotatio_vel, 0, MAX_W);

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
                this->linear_vel += 0.1;        
                show_msg();
                printf("\b");
                break;
            case 102: //F
                this->rotatio_vel -= 0.1* M_PI;
                show_msg();
                printf("\b");
                break;
            case 104: //H
                this->rotatio_vel += 0.1* M_PI;
                show_msg();
                printf("\b");
                break;
            case 103: //G
                this->linear_vel -= 0.1;
                show_msg();
                printf("\b");
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

            des_velx = constrain(des_velx, -MAX_VX, MAX_VX);
            des_vely = constrain (des_vely, -MAX_VY, MAX_VY);
            des_velw = constrain (des_velw, -MAX_W, MAX_W);
            
            vx = des_velx;
            vy = des_vely;
            w = des_velw;

            //END OF MAIN
            

            //filling StateVector Message
            commands.vx = vx;
            commands.vy = vy;
            commands.w = w;

            //publish commands
            publisher_->publish(commands);
        }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Keyboard_control>());
    rclcpp::shutdown();
    return 0;
}