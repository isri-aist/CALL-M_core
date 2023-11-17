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
#include <linux/joystick.h>
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

/**
 * Reads a joystick event from the joystick device.
 *
 * Returns 0 on success. Otherwise -1 is returned.
 */
int read_event(int fd, struct js_event *event)
{
    ssize_t bytes;

    bytes = read(fd, event, sizeof(*event));

    if (bytes == sizeof(*event))
        return 0;

    /* Error, could not read full event. */
    return -1;
}

/**
 * Current state of an axis.
 */
struct axis_state {
    short x, y;
};

/**
 * Keeps track of the current axis state.
 *
 * NOTE: This function assumes that axes are numbered starting from 0, and that
 * the X axis is an even number, and the Y axis is an odd number. However, this
 * is usually a safe assumption.
 *
 * Returns the axis that the event indicated.
 */
size_t get_axis_state(struct js_event *event, struct axis_state axes[3])
{
    size_t axis = event->number / 2;

    if (axis < 3)
    {
        if (event->number % 2 == 0)
            axes[axis].x = event->value;
        else
            axes[axis].y = event->value;
    }

    return axis;
}

class Joystick_control:public rclcpp::Node
{
    public:
        Joystick_control():Node("joystick_control_node")
        {
            //create pubisher that will publish message of type [vx,vy,w]
            publisher_ = this->create_publisher<c_pkg::msg::StateVector>("joystick_bot_command", 10);
            //create timer that will call repetitively the function timer_callback
            timer_ = this->create_wall_timer(dt, std::bind(&Joystick_control::timer_callback, this));
            RCLCPP_INFO(this->get_logger(),"joystick_control_node started...");
            setup_js();
            show_msg(0.0,0.0,0.0);
        } 

    private:
        //global variables    
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<c_pkg::msg::StateVector>::SharedPtr publisher_;
        std::chrono::milliseconds dt = 10ms;

        int js;
        struct js_event event;
        struct axis_state axes[3] = {0};
        size_t axis;

        float linear_vel = 0.1;
        float rotatio_vel = 0.1 * M_PI;
        float lin_incr = 0.1;
        float rot_incr = 0.1* M_PI;

        //Functions
        void show_msg(float des_velx,float des_vely,float des_velw){
            if(this->js != -1){
                printf( "\033[%dm\033[2J\033[1;1f",0);
                printf("JOYSTICK CONTROLS:\n\n");
                printf("   --7--                      --9--\n\n");
                printf("   --6--                      --8--\n\n");
                printf("                                   4\n");
                printf("     ^                          2    5\n");
                printf("  <     >                    3     0\n");
                printf("     v                          1\n\n");
                printf("        Left Pad     Right Pad\n\n");
                printf("Current max linear speed: %f",this->linear_vel);
                printf("\nCurrent max rotationnal speed: %f",this->rotatio_vel);
                printf("\n\nLeft Pad: Linear movements\n");
                printf("Right Pad: Rotation\n");
                printf("6/8: Set linear speed\n");
                printf("7/9: Set rotational speed\n");
                printf("\nChange the joystick mode if not corresponding.\n\n");
                printf("Current commands: (%.2f,%.2f,%.2f)\n",des_velx,des_vely,des_velw);
                if(this->event.type == JS_EVENT_BUTTON){
                    printf("Button %u %s\n", this->event.number, this->event.value ? "pressed" : "released");
                }
                else if(this->event.type == JS_EVENT_AXIS){
                    printf("Axis %zu at (%6d, %6d)\n", this->axis, this->axes[axis].x, this->axes[axis].y);
                } 
            }
            else{
                printf("\033[%dm\033[2J\033[1;1f",0);
                printf("Waiting for joystick...");
            }

        }

        void timer_callback()
        {   
            auto commands=c_pkg::msg::StateVector();

            float des_velx = 0.0;
            float des_vely = 0.0;
            float des_velw = 0.0;

            //MAIN PROGRAM TO COMPUTE SPEEDS
            this->linear_vel = constrain (this->linear_vel, 0, MAX_VX);
            this->rotatio_vel = constrain (this->rotatio_vel, 0, MAX_W);

            if(this->js == -1) //if no joystick is connected, we try to connect one
            {
                setup_js();
            }
            else//otherwise we can start to use it
            {
                if (read_event(this->js, &(this->event)) == 0)
                {
                    switch (this->event.type)
                    {
                        case JS_EVENT_BUTTON:
                            switch (this->event.number)
                            {
                            case 6:
                                if(this->event.value){this->linear_vel-=this->lin_incr;} //if button 6 pressed we add linear speed
                                break;
                            case 7:
                                if(this->event.value){this->rotatio_vel-=this->rot_incr;} 
                                break;
                            case 8:
                                if(this->event.value){this->linear_vel+=this->lin_incr;}
                                break;
                            case 9:
                                if(this->event.value){this->rotatio_vel+=this->rot_incr;} 
                                break;                                                                                            
                            default:
                                break;
                            }
                            break;
                        case JS_EVENT_AXIS:
                            this->axis = get_axis_state(&this->event, this->axes);
                            if (this->axis < 3)
                                switch (axis)
                                {
                                case 0:
                                    des_velx = ((axes[axis].x)/32767.0)*linear_vel;
                                    des_vely = ((axes[axis].y)/32767.0)*linear_vel;
                                    break;
                                case 1:
                                    des_velw = -((axes[axis].y)/32767.0)*rotatio_vel;
                                    break;                                
                                default:
                                    break;
                                }
                            break;
                        default:
                            /* Ignore init events. */
                            break;
                    }
                    
                    fflush(stdout);
                }
                else{
                    close(this->js);
                    this->js = -1;
                }
            }
            
        
            des_velx = constrain(des_velx, -MAX_VX, MAX_VX);
            des_vely = constrain (des_vely, -MAX_VY, MAX_VY);
            des_velw = constrain (des_velw, -MAX_W, MAX_W);

            show_msg(des_velx,des_vely,des_velw);
            
            //END OF MAIN CODE

            //filling StateVector Message
            commands.vx = des_velx;
            commands.vy = des_vely;
            commands.w = des_velw;

            //publish commands
            publisher_->publish(commands);
        }

        void setup_js()
        {
            const char *device;
            device = "/dev/input/js0";
            this->js = open(device, O_RDONLY); // js=-1 means that could not open joystick
        }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Joystick_control>());
    rclcpp::shutdown();
    return 0;
}