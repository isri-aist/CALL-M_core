#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <lifecycle_msgs/srv/get_available_states.hpp>
#include <lifecycle_msgs/srv/get_available_transitions.hpp>
#include <lifecycle_msgs/msg/state.hpp>

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
            initialize_common_params();
            refresh_common_params();

            status_details = "No messages received on " + input_node_state_topic_name + " yet";

            // Create a lifecycle node for the /controlled_managed_node_name node
            RCLCPP_INFO(this->get_logger(),"\njoystick_control_node started...");
            RCLCPP_INFO(this->get_logger(),"\nTrying to connect to '%s' node...",controlled_managed_node_name.c_str());
            create_lifecycleclient();
            RCLCPP_INFO(this->get_logger(),"\nGetting %s state...",controlled_managed_node_name.c_str());
            stat_id = get_state(); 
            update_msg_status(stat_id);

            auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

            //create pubisher that will publish message of type [vx,vy,w]
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(topic_main_cmd, default_qos);
            publisher_assisted = this->create_publisher<geometry_msgs::msg::Twist>(topic_assissted_cmd, default_qos);
            //publisher for camera servos
            publisher_servo_cam = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>("/set_position", default_qos);
            //create timer that will call repetitively the function timer_callback
            timer_ = this->create_wall_timer(std::chrono::milliseconds(int(1000/rate)), std::bind(&Joystick_control::timer_callback, this));
            setup_js();
            show_msg(0.0,0.0,0.0);

            sub_stat_notification_ = this->create_subscription<lifecycle_msgs::msg::State>(input_node_state_topic_name,10,std::bind(&Joystick_control::stat_notification_callback, this, std::placeholders::_1));

        } 

        //parameters
        int linear_speed_axis;
        int angular_speed_axis;
        int max_linear_speed_up_button;
        int max_linear_speed_down_button;
        int max_angular_speed_up_button;
        int max_angular_speed_down_button;
        int front_camera_up_button;
        int front_camera_down_button;
        int back_camera_up_button;
        int back_camera_down_button;
        int assissted_teleoperation_mode_button;
        int emergency_button;
        int arming_axis; 
        std::string configure_state_direction;
        std::string activate_state_direction; 
        std::string deactivate_state_direction; 
        std::string reconnect_direction;
        double max_linear_speed_axis_value_input; 
        int linear_speed_direction;
        double max_angular_speed_axis_value_input;
        int angular_speed_direction;
        double max_linear_speed_axis_value_wanted;
        double max_angular_speed_axis_value_wanted;
        double linear_speed_increment;
        double angular_speed_increment;
        double camera_angle_increment;
        double rate;
        std::string topic_main_cmd;
        std::string topic_assissted_cmd;
        std::string controlled_managed_node_name;
        std::string input_node_state_topic_name;

        //global variables    
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_assisted;

        int js;
        struct js_event event;
        struct axis_state axes[3] = {0};
        size_t axis;
        int version;

        double linear_vel = 0.2;
        double rotatio_vel = 0.1;
        double joy_axe0_x = 0.0;
        double joy_axe0_y = 0.0;
        double joy_axe1_y = 0.0;
        double des_velx = 0.0;
        double des_vely = 0.0;
        double des_velw = 0.0;

        rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr publisher_servo_cam;
        dynamixel_sdk_custom_interfaces::msg::SetPosition servo_cam_msg;
        int servo_cam1_id=2;
        int servo_cam2_id=3;
        int servo_cam1_pos = 2000; //1000 to 3000 (1000 = looking down)
        int servo_cam2_pos = 2000;
        int servo_cam_incr = camera_angle_increment*(2000/180);

        //auto mode, security
        bool auto_mode = false;
        float chrono = 0.0;
        int step = 0;
        float sec_dt = 0.1; //loop dt in seconds
        float distance_t = 1*sec_dt;

        //assissted mode
        bool assissted = false;

        //arming controlled_managed_node_name
        bool lifecycle_ready = false;
        int stat_id;
        std::string current_status = "None"; //current status information
        std::string status_details = "None"; //used for error management
        std::string msg_status = "None"; //instructions for current stat to start robot
        rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_client_;
        std::shared_ptr<rclcpp::Subscription<lifecycle_msgs::msg::State>> sub_stat_notification_; 

        void stat_notification_callback(lifecycle_msgs::msg::State::ConstSharedPtr msg)
        {
            stat_id = msg->id;
            status_details = "Stat "+ msg->label + " with ID: "+to_string(msg->id);
            update_msg_status(stat_id);
        }

        void update_msg_status(int stat_id_val){
            if (stat_id_val == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED){
                msg_status = "PRESS '" + configure_state_direction +  "' TO CONFIGURE";
                current_status = "Unconfigured"; //could be updated by stat_notification_callback, but we want custom labels
            }
            else if(stat_id_val == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE){
                msg_status = "PRESS '" + activate_state_direction +  "' TO ACTIVATE";
                current_status = "Configured, Inactive";
            }
            else if(stat_id_val == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE){
                msg_status = "PRESS '" + deactivate_state_direction +  "' TO DEACTIVATE";
                current_status = "Active";
            }
            else if(stat_id_val == -1){
                current_status = "OFF: "+status_details;
                msg_status = "No response, please start or restart the "+ controlled_managed_node_name +" node and press '"+ reconnect_direction +"' to reconnect";
            }
            else{
                current_status = "Unmanaged Status";
                msg_status = "stat_id = "+to_string(stat_id) + ", Detail: " + status_details;
            }
        }

        //Functions
        void show_msg(double des_velx,double des_vely,double des_velw){
            if(ioctl(this->js, JSIOCGVERSION, &version) >= 0){
                printf( "\033[%dm\033[2J\033[1;1f",0);
                printf("JOYSTICK CONTROLS(example):\n\n");
                printf("   --7--                      --9--\n\n");
                printf("   --6--                      --8--\n\n");
                printf("                                   4\n");
                printf("       ^                        2    5\n");
                printf("  <  Axis2  >                3     0\n");
                printf("       v                        1\n\n");
                printf("        Axis0     Axis1\n\n");
                printf("MOBILE BASE CONTROL\n\n");
                printf("Current max linear speed : %.2f%.",(this->linear_vel/max_linear_speed_axis_value_wanted)*100);
                printf("\nCurrent max rotationnal speed: %.2f%.",(this->rotatio_vel/max_angular_speed_axis_value_wanted)*100);
                printf("\n\nAxis %d: Linear movements\n",linear_speed_axis);
                printf("Axis %d: Rotation\n",angular_speed_axis);
                printf("%d: Toggle assissted mode\n",assissted_teleoperation_mode_button);
                printf("%d: Toggle lock cmds mode\n",emergency_button);
                printf("%d/%d: Set linear speed\n",max_linear_speed_down_button,max_linear_speed_up_button);
                printf("%d/%d: Set rotational speed\n", max_angular_speed_down_button,max_angular_speed_up_button);
                printf("%d/%d: Front Camera angle\n",front_camera_down_button,front_camera_up_button);
                printf("%d/%d: Back Camera angle\n",back_camera_down_button,back_camera_up_button);
                printf("%s|%s|%s|%s: configure|activate|deactivate|update",configure_state_direction.c_str(),activate_state_direction.c_str(),deactivate_state_direction.c_str(),reconnect_direction.c_str());
                printf("\nChange the joystick mode if not corresponding.\n\n");
                printf("%s status: %s",controlled_managed_node_name.c_str(),current_status.c_str());
                printf("\n%s instructions: %s\n\n",controlled_managed_node_name.c_str(),msg_status.c_str());
                printf("Current commands: (%.2f,%.2f,%.2f)\n",des_velx,des_vely,des_velw);
                printf("Total linear speed command: %.2f%\n",sqrt(pow(des_velx,2)+pow(des_vely,2))*100);
                if(this->event.type == JS_EVENT_BUTTON){
                    printf("Button %u %s\n", this->event.number, this->event.value ? "pressed" : "released");
                }
                else if(this->event.type == JS_EVENT_AXIS){
                    printf("Axis %zu at (%6d, %6d)\n", this->axis, this->axes[axis].x, this->axes[axis].y);
                } 
                if(this->assissted){
                    printf("Assissted teleoperation activated.\n");
                }
                if(this->auto_mode){
                    printf("LOCK MODE...\n");
                }
            }
            else{
                printf("\033[%dm\033[2J\033[1;1f",0);
                printf("Waiting for joystick...");
            }

        }

        void refresh_speeds(){
            //linear speeds
            if(des_velx!=linear_speed_direction*joy_axe0_y || des_vely!=linear_speed_direction*joy_axe0_x){
                des_velx=linear_speed_direction*joy_axe0_y;
                des_vely=linear_speed_direction*joy_axe0_x;
                //We want the norm of (vx,vy) to be bounded, we remap the values that were between -1 and 1 to of the norm wanted (for square joysticks)
                /*if (des_vely!=0 && des_velx!=0){
                    double a = abs(des_velx/des_vely);
                    //double L= sqrt(pow(des_velx,2.0)+pow(des_vely,2.0)); 
                    double new_vx = abs(des_velx)*linear_vel;
                    double new_vy = abs(des_vely)*linear_vel;
                    double new_L = sqrt(pow(new_vx,2.0)+pow(new_vy,2.0));
                    //double max_vy = abs(des_vely/des_velx);
                    double max_new_vy = abs(new_vy*linear_vel/new_vx);
                    //double max_L = sqrt(1+pow(max_vy,2));
                    double max_new_L = sqrt(pow(linear_vel,2)+pow(max_new_vy,2));
                    double map_L= linear_vel*new_L/max_new_L;
                    
                    des_vely = sign(des_vely)*map_L/sqrt(1+pow(a,2));
                    des_velx = sign(des_velx)*a*abs(des_vely);
                }
                else{
                    des_velx =des_velx*this->linear_vel;
                    des_vely =des_vely*this->linear_vel;
                }*/
                
                des_velx =des_velx*this->linear_vel;
                des_vely =des_vely*this->linear_vel;
            
                double N_max= this->linear_vel;
                double N = sqrt(pow(des_velx,2.0)+pow(des_vely,2.0));
                if (N > N_max){
                    des_velx =des_velx*(N_max/N);
                    des_vely =des_vely*(N_max/N);
                }
            }

            //rotation speed
            if(des_velw!=angular_speed_direction*joy_axe1_y){
                des_velw=angular_speed_direction*joy_axe1_y;
                //rotation speed have direct bounds
                des_velw = des_velw*this->rotatio_vel; //naturally bounds because rotatio_vel is bound and des_velw is a percentage of it 
            } 

        }

        void timer_callback()
        {   
            geometry_msgs::msg::Twist commands;

            //MAIN PROGRAM TO COMPUTE SPEEDS

            if(ioctl(this->js, JSIOCGVERSION, &version) < 0) //if no joystick is connected, we try to connect one
            {
                //RCLCPP_INFO(this->get_logger(),"RESET...");
                des_velx = 0.0;
                des_vely = 0.0;
                des_velw = 0.0;
                setup_js();
            }
            else//otherwise we can start to use it
            {   
                //RCLCPP_INFO(this->get_logger(),"Connected...%d",version);
                if (read_event(this->js, &(this->event)) == 0) //check if there is an event
                {
                    //RCLCPP_INFO(this->get_logger(),"event...");
                    switch (this->event.type)
                    {
                        case JS_EVENT_BUTTON: //we ca not use switch for the following code, because we use variables
                            if (this->event.number == this->max_linear_speed_down_button) {
                                if(this->event.value){this->linear_vel-=this->linear_speed_increment;}
                            } else if (this->event.number == this->max_angular_speed_down_button) {
                                if(this->event.value){this->rotatio_vel-=this->angular_speed_increment;}
                            } else if (this->event.number == this->max_linear_speed_up_button) {
                                if(this->event.value){this->linear_vel+=this->linear_speed_increment;}
                            } else if (this->event.number == this->max_angular_speed_up_button) {
                                if(this->event.value){this->rotatio_vel+=this->angular_speed_increment;}
                            } else if (this->event.number == this->front_camera_up_button) {
                                if(this->event.value){
                                    servo_cam1_pos = constrain(servo_cam1_pos-servo_cam_incr,1000,3000);
                                    servo_cam_msg.id = servo_cam1_id;
                                    servo_cam_msg.position = servo_cam1_pos;
                                    publish_cam_cmd();
                                }
                            } else if (this->event.number == this->front_camera_down_button) {
                                if(this->event.value){
                                    servo_cam1_pos = constrain(servo_cam1_pos+servo_cam_incr,1000,3000);
                                    servo_cam_msg.id = servo_cam1_id;
                                    servo_cam_msg.position = servo_cam1_pos;
                                    publish_cam_cmd();
                                }
                            } else if (this->event.number == this->back_camera_up_button) {
                                if(this->event.value){
                                    servo_cam2_pos = constrain(servo_cam2_pos-servo_cam_incr,1000,3000);
                                    servo_cam_msg.id = servo_cam2_id;
                                    servo_cam_msg.position = servo_cam2_pos;
                                    publish_cam_cmd();
                                }
                            } else if (this->event.number == this->back_camera_down_button) {
                                if(this->event.value){
                                    servo_cam2_pos = constrain(servo_cam2_pos+servo_cam_incr,1000,3000);
                                    servo_cam_msg.id = servo_cam2_id;
                                    servo_cam_msg.position = servo_cam2_pos;
                                    publish_cam_cmd();
                                }
                            } else if (this->event.number == this->emergency_button) {
                                if(this->event.value){this->auto_mode = !this->auto_mode;}
                            } else if (this->event.number == this->assissted_teleoperation_mode_button) {
                                if(this->event.value){this->assissted = !this->assissted;}
                            }
                        case JS_EVENT_AXIS:
                            this->axis = get_axis_state(&this->event, this->axes);
                            break;
                        default:
                            /* Ignore init events. */
                            break;
                    }
                    if (this->axis < 3){
                        if (this->axis == this->linear_speed_axis) {
                            joy_axe0_x = ((axes[axis].x)/max_linear_speed_axis_value_input); //percentage
                            joy_axe0_y = ((axes[axis].y)/max_linear_speed_axis_value_input); //percentage
                        } else if (this->axis == this->angular_speed_axis) {
                            joy_axe1_y = ((axes[axis].y)/max_angular_speed_axis_value_input);
                        }
                        else if(this->axis == this->arming_axis){
                            std::string direction = "none";
                            if(axes[axis].y < 0.0){ //"up"
                                direction = "up";
                            }
                            else if(axes[axis].x < 0.0){ //"left"
                                direction = "left";
                            }
                            else if(axes[axis].y > 0.0){ //"down"
                                direction = "down";
                            }
                            else if(axes[axis].x > 0.0){ //"right"
                                direction = "right";
                            }

                            if(direction == configure_state_direction){ 
                                RCLCPP_INFO(this->get_logger(),"\033[%dm\033[2J\033[1;1f");
                                RCLCPP_INFO(this->get_logger(),"\nConfiguring %s...",controlled_managed_node_name.c_str());
                                stat_id = get_state(); //we don't put it above because we want to compute that only when clicked (would slow down node otherwise)
                                if (stat_id == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) { 
                                    change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);  
                                }
                            }
                            else if(direction == activate_state_direction){ 
                                RCLCPP_INFO(this->get_logger(),"\033[%dm\033[2J\033[1;1f");
                                RCLCPP_INFO(this->get_logger(),"\nActivating %s...",controlled_managed_node_name.c_str());
                                stat_id = get_state(); //we don't put it above because we want to compute that only when clicked (would slow down node otherwise)
                                if (stat_id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) { 
                                    change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);   
                                }
                            }
                            else if(direction == deactivate_state_direction){ 
                                RCLCPP_INFO(this->get_logger(),"\033[%dm\033[2J\033[1;1f");
                                RCLCPP_INFO(this->get_logger(),"\nDeactivating %s...",controlled_managed_node_name.c_str());
                                stat_id = get_state(); //we don't put it above because we want to compute that only when clicked (would slow down node otherwise) 
                                if (stat_id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                                    change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);      
                                }
                            }
                            else if(direction == reconnect_direction){ 
                                if(true){
                                    RCLCPP_INFO(this->get_logger(),"\033[%dm\033[2J\033[1;1f");
                                    RCLCPP_INFO(this->get_logger(),"\nTrying to connect to '%s' node...",controlled_managed_node_name.c_str());
                                    create_lifecycleclient();
                                    stat_id = get_state();
                                }
                            }
                            update_msg_status(stat_id); //in case of errors
                        }
                    }
                    refresh_speeds(); //we update sent speeds
                    fflush(stdout);
                }
            }

            /*
            AUTO SECURITY MODE
            */
            if (this->auto_mode){
                switch (step)
                {
                case 0:
                des_velx = 0;
                des_vely = 0;
                des_velw = 0.05;
                this->chrono += this->sec_dt;
                if(this->chrono > this->distance_t){
                    this->chrono = 0;
                    this->step ++;
                }
                break;
                case 1:
                des_velx = 0;
                des_vely = 0;
                des_velw = -0.05;
                this->chrono += this->sec_dt;
                if(this->chrono > this->distance_t){
                    this->chrono = 0;
                    this->step ++;
                }
                break;
                case 2:
                des_velx = 0;
                des_vely = 0;
                des_velw = 0.05;
                this->chrono += this->sec_dt;
                if(this->chrono > this->distance_t){
                    this->chrono = 0;
                    this->step ++;
                }
                break;
                case 3:
                des_velx = 0;
                des_vely = 0;
                des_velw = -0.05;
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
            /*
            END of AUTO SECURITY MODE
            */
            
            //RCLCPP_INFO(this->get_logger(),"vx,vy,w:2 (%.2f,%.2f,%.2f)",des_velx,des_vely,des_velw);

            //END OF MAIN CODE

            this->linear_vel = constrain (this->linear_vel, 0, max_linear_speed_axis_value_wanted);
            this->rotatio_vel = constrain (this->rotatio_vel, 0, max_angular_speed_axis_value_wanted);

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

            show_msg(commands.linear.x,commands.linear.y,commands.angular.z);

            //publish commands
            if(this->assissted){
                publisher_assisted->publish(commands);
            }
            else{
                publisher_->publish(commands);
            }
        }

        void setup_js()
        {
            const char *device;
            device = "/dev/input/js0";
            this->js = open(device, O_RDONLY | O_NONBLOCK); // js=-1 means that could not open joystick
        }

        void publish_cam_cmd()
        {
            //printf("Publish: id: %d, pos: %d",servo_cam_msg.id,servo_cam_msg.position);
            publisher_servo_cam->publish(servo_cam_msg);
        }

        void initialize_common_params(){
            this->declare_parameter("linear_speed_axis", 0);
            this->declare_parameter("angular_speed_axis", 1);
            this->declare_parameter("max_linear_speed_up_button", 8);
            this->declare_parameter("max_linear_speed_down_button", 6);
            this->declare_parameter("max_angular_speed_up_button", 9);
            this->declare_parameter("max_angular_speed_down_button", 7);
            this->declare_parameter("front_camera_up_button", 2);
            this->declare_parameter("front_camera_down_button", 0);
            this->declare_parameter("back_camera_up_button", 3);
            this->declare_parameter("back_camera_down_button", 1);
            this->declare_parameter("assissted_teleoperation_mode_button", 4);
            this->declare_parameter("emergency_button", 5);
            this->declare_parameter("arming_axis", 2);
            this->declare_parameter("configure_state_direction", "left");
            this->declare_parameter("activate_state_direction", "up");
            this->declare_parameter("deactivate_state_direction", "down");
            this->declare_parameter("reconnect_direction", "right");
            this->declare_parameter("max_linear_speed_axis_value_input", 32767.0);
            this->declare_parameter("linear_speed_direction", -1);
            this->declare_parameter("max_angular_speed_axis_value_input", 32767.0);
            this->declare_parameter("angular_speed_direction", -1);
            this->declare_parameter("max_linear_speed_axis_value_wanted", 1.0);
            this->declare_parameter("max_angular_speed_axis_value_wanted", 1.0);
            this->declare_parameter("linear_speed_increment", 0.2);
            this->declare_parameter("angular_speed_increment", 0.1);
            this->declare_parameter("camera_angle_increment", 18.0);
            this->declare_parameter("rate", 30.0);
            this->declare_parameter("topic_main_cmd", "cmd_vel_teleop_joy");
            this->declare_parameter("topic_assissted_cmd", "cmd_vel_teleop_assist");
            this->declare_parameter("controlled_managed_node_name", "/managed_node");
            this->declare_parameter("input_node_state_topic_name", "/managed_node_state");
            
        }

        void refresh_common_params(){
            this->get_parameter("linear_speed_axis", linear_speed_axis);
            this->get_parameter("angular_speed_axis", angular_speed_axis);
            this->get_parameter("max_linear_speed_up_button", max_linear_speed_up_button);
            this->get_parameter("max_linear_speed_down_button", max_linear_speed_down_button);
            this->get_parameter("max_angular_speed_up_button", max_angular_speed_up_button);
            this->get_parameter("max_angular_speed_down_button", max_angular_speed_down_button);
            this->get_parameter("front_camera_up_button", front_camera_up_button);
            this->get_parameter("front_camera_down_button", front_camera_down_button);
            this->get_parameter("back_camera_up_button", back_camera_up_button);
            this->get_parameter("back_camera_down_button", back_camera_down_button);
            this->get_parameter("assissted_teleoperation_mode_button", assissted_teleoperation_mode_button);
            this->get_parameter("emergency_button", emergency_button);
            this->get_parameter("arming_axis", arming_axis);
            this->get_parameter("configure_state_direction", configure_state_direction);
            this->get_parameter("activate_state_direction", activate_state_direction);
            this->get_parameter("deactivate_state_direction", deactivate_state_direction);
            this->get_parameter("reconnect_direction", reconnect_direction);
            this->get_parameter("max_linear_speed_axis_value_input", max_linear_speed_axis_value_input);
            this->get_parameter("linear_speed_direction", linear_speed_direction);
            this->get_parameter("max_angular_speed_axis_value_input", max_angular_speed_axis_value_input);
            this->get_parameter("angular_speed_direction", angular_speed_direction);
            this->get_parameter("max_linear_speed_axis_value_wanted", max_linear_speed_axis_value_wanted);
            this->get_parameter("max_angular_speed_axis_value_wanted", max_angular_speed_axis_value_wanted);
            this->get_parameter("linear_speed_increment", linear_speed_increment);
            this->get_parameter("angular_speed_increment", angular_speed_increment);
            this->get_parameter("camera_angle_increment", camera_angle_increment);
            this->get_parameter("rate", rate);
            this->get_parameter("topic_main_cmd", topic_main_cmd);
            this->get_parameter("topic_assissted_cmd", topic_assissted_cmd);
            this->get_parameter("controlled_managed_node_name", controlled_managed_node_name);
            this->get_parameter("input_node_state_topic_name", input_node_state_topic_name);
            
        }

        /**
        * Create ROS 2 service client for lifecycle transition
        */
        void create_lifecycleclient() {
            //auto service_callback_group = nullptr;
            //std::string ch_state = name+"/change_state";
            this->change_state_client_ = this->create_client<lifecycle_msgs::srv::ChangeState>(controlled_managed_node_name+"/change_state",rmw_qos_profile_services_default);

            /*while(!change_state_client_->wait_for_service(5s)){
            if(!rclcpp::ok()){
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
            return ;
            }
            RCLCPP_INFO(this->get_logger(), "waiting for lifecycle service...");
            }*/

            if(!change_state_client_->wait_for_service(5s)){
                status_details = "Failed to create client for '"+controlled_managed_node_name+"/change_state'";
                return;
            }

            lifecycle_ready = true;

            return;
        }

        /**
        *  ROS 2 service call to get the lifecycle node
        *     Return: Lifecycle state ID
        */
        int get_state() {
            
            //Check if working from terminal: ros2 service call /controlled_managed_node_name/get_state lifecycle_msgs/srv/GetState "{}"

            if(lifecycle_ready){
                if(!change_state_client_->wait_for_service(5s)){ //we check if the triorb node is still alive
                    status_details = "Service '"+controlled_managed_node_name+"/change_state' disconnected?";
                    lifecycle_ready = false;
                    return -1;
                }
                //otherwise we just return current state
                return stat_id;
            }
            else{
                //detail of error updated if init client function 'create_lifecyleclient'
                return -1;
            }
        }

        /**
        *  ROS 2 service call to change the lifecycle node's state
        */
        void change_state(int id) {
            /*if (!this->change_state_client_->service_is_ready()) {
                RCLCPP_INFO(this->get_logger(),"srv not ready");
                return;
            }*/
            auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
            request->transition.id=id;
            auto result = this->change_state_client_->async_send_request(request);
            return;
        }


};

/*void kill(std::shared_ptr<Joystick_control> node) {
  auto joystick_control_node = std::dynamic_pointer_cast<Joystick_control>(node);
  if (joystick_control_node) {
    if(joystick_control_node->stat_id==lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE){
        joystick_control_node->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
    }
    else if (joystick_control_node->stat_id==lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
        joystick_control_node->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
        joystick_control_node->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
    }
  }
}*/

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Joystick_control>();

  //Register the kill() function to be called on shutdown, passing the node object as an argument
  //using namespace std::placeholders;
  //rclcpp::on_shutdown(std::bind(static_cast<void(*)(std::shared_ptr<Joystick_control>)>(kill), node));

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}