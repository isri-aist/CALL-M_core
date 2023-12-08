#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <sstream>
#include <termios.h>
#include <linux/joystick.h>
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

int sign(double val){
    if (val>=0){
        return 1;
    }
    else{
        return -1;
    }
}


class Joystick_control:public rclcpp::Node
{
    public:
        Joystick_control():Node("joystick_control_node")
        {
            //create pubisher that will publish message of type [vx,vy,w]
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_teleop_joy", 10);
            //create timer that will call repetitively the function timer_callback
            timer_ = this->create_wall_timer(dt, std::bind(&Joystick_control::timer_callback, this));
            RCLCPP_INFO(this->get_logger(),"joystick_control_node started...");
            setup_js();
            show_msg(0.0,0.0,0.0);
        } 

    private:
        //global variables    
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        std::chrono::milliseconds dt = 10ms;

        int js;
        struct js_event event;
        struct axis_state axes[3] = {0};
        size_t axis;

        double linear_vel = 0.1;
        double rotatio_vel = 0.1;
        double lin_incr = 0.1;
        double rot_incr = 0.1;
        double max_lin_sp = 1; //we are working with values between -1 and 1 = percentage
        double max_rot_sp = 1; //we are working with values between -1 and 1 = percentage
        double max_joystick_value = 32767.0; //maximal value send by the joystick's pads
        double des_velx = 0.0;
        double des_vely = 0.0;
        double des_velw = 0.0;

        //Functions
        void show_msg(double des_velx,double des_vely,double des_velw){
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
                printf("Current max linear speed : %.2f%",(this->linear_vel)*100);
                printf("\nCurrent max rotationnal speed: %.2f%",(this->rotatio_vel)*100);
                printf("\n\nLeft Pad: Linear movements\n");
                printf("Right Pad: Rotation\n");
                printf("6/8: Set linear speed\n");
                printf("7/9: Set rotational speed\n");
                printf("\nChange the joystick mode if not corresponding.\n\n");
                printf("Current commands: (%.2f,%.2f,%.2f)\n",des_velx,des_vely,des_velw);
                printf("Total linear speed command: %.2f%\n",sqrt(pow(des_velx,2)+pow(des_vely,2))*100);
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
            geometry_msgs::msg::Twist commands;

            //double des_velx = 0.0;
            //double des_vely = 0.0;
            //double des_velw = 0.0;

            //MAIN PROGRAM TO COMPUTE SPEEDS
            this->linear_vel = constrain (this->linear_vel, 0, max_lin_sp);
            this->rotatio_vel = constrain (this->rotatio_vel, 0, max_rot_sp);

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
                                    des_velx = ((axes[axis].x)/32767.0); //percentage
                                    des_vely = ((axes[axis].y)/32767.0); //percentage
                                    break;
                                case 1:
                                    des_velw = -((axes[axis].y)/32767.0); //percentage
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
            
            //We want the norm of (vx,vy) to be bounded, we remap the values that were between -1 and 1 to of the norm wanted
            if (des_vely!=0 && des_velx!=0){
                double a = abs(des_velx/des_vely);
                double L= sqrt(pow(des_velx,2.0)+pow(des_vely,2.0)); 
                double new_vx = abs(des_velx)*linear_vel;
                double new_vy = abs(des_vely)*linear_vel;
                double new_L = sqrt(pow(new_vx,2.0)+pow(new_vy,2.0));
                double max_vy = abs(des_vely/des_velx);
                double max_new_vy = abs(new_vy*linear_vel/new_vx);
                double max_L = sqrt(1+pow(max_vy,2));
                double max_new_L = sqrt(pow(linear_vel,2)+pow(max_new_vy,2));
                double map_L= linear_vel*new_L/max_new_L;
                
                des_vely = sign(des_vely)*map_L/sqrt(1+pow(a,2));
                des_velx = sign(des_velx)*a*abs(des_vely);
            }
            else{
                des_velx =des_velx*this->linear_vel;
                des_vely =des_vely*this->linear_vel;
            }

            //rotation speed have direct bounds
            des_velw = des_velw*this->rotatio_vel; //naturally bounds because rotatio_vel is bound and des_velw is a percentage of it

            show_msg(des_velx,des_vely,des_velw);
            
            //END OF MAIN CODE

            //filling StateVector Message
            commands.linear.x = des_velx;
            commands.linear.y = des_vely;
            commands.angular.z = des_velw;

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