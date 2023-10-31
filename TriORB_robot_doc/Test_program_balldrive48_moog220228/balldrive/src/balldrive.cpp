#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <sstream>
#include "func.h"
#include "kinema.h"
#include "servo3motor.h"
#include "ps3con.h"

//////ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
using namespace std;
//////

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

float target_motor_rpm[6];
float real_motor_rpm[6];

///////////////////////////////////////////////////////////////////////
// control parameters
float rotor_rad_p_sec[] = {0.0, 0.0, 0.0}; // rev. per sec
float des_velx = 0.0;
float des_vely = 0.0;
float des_velw = 0.0;
float des_velarm = 0.0;
float cur_velx = 0.0;
float cur_vely = 0.0;
float cur_velw = 0.0;
float cur_velarm = 0.0;
float rea_velx = 0.0;
float rea_vely = 0.0;
float rea_velw = 0.0;
float rea_angarm = 0.0;

///////////////////////////////////////////////////////////////////////
//ROS
int recording_flag = 0;

void setup() {
  printf("Ball Wheel Drive `");
  open_js();
  servomotor_setup();
  rotor_rad_p_sec[0] = 0.0;
  rotor_rad_p_sec[1] = 0.0;
  rotor_rad_p_sec[2] = 0.0;
  printf("' start\n");
}

void loop(ros::Publisher rec_flag_pub, ros::Publisher button_state_pub) 
{
    unsigned long current_time = millis();
    static unsigned long last_time = current_time;
    
    float rotor_rad_per_sec_gain = 1.0;

    ////////////////////////////////////////////////////////////
    // control interval
    if (current_time - last_time > CONTROL_INTERVAL) {
      last_time = current_time;
      
      std_msgs::Bool msg;
      std_msgs::String msg2;
      std::stringstream ss;

      joystick_loop(&des_velx, &des_vely, &des_velw, &des_velarm, &recording_flag); // one 3-axes
      des_velarm *= M_PI_2;
      switch (recording_flag) {
      case 2:
        ss << "Run automatically";
        msg2.data = ss.str();
        msg.data = true;
        break;
      case 1:
        ss << "Pushed Button";
        msg2.data = ss.str();
        msg.data = true;
        break;
      case 0:
        ss << "No Pushed...";
        msg2.data = ss.str();
        msg.data = false;
        break;
      }
      msg2.data = ss.str();
    
      rec_flag_pub.publish(msg);
      button_state_pub.publish(msg2);
      ros::spinOnce();
      
      constrain (des_velx, -MAX_VX, MAX_VX);
      constrain (des_vely, -MAX_VY, MAX_VY);
      constrain (des_velw, -MAX_W, MAX_W);
      
      cur_velx = des_velx;
      cur_vely = des_vely;
      cur_velw = des_velw;
      cur_velarm = des_velarm;

      // convert vehicle vel. -> rotor vel.
      vel2rotor (rotor_rad_p_sec, cur_velx, cur_vely, cur_velw);

      // move motor
      rotor_rad_per_sec_gain = servo3motor_loop (rotor_rad_p_sec, target_motor_rpm, real_motor_rpm);
    } // if (current_time - last_time > CONTROL_INTERVAL)

} // loop

int main(int argc, char **argv)
{
  ros::init(argc, argv, "BallDrive");
  ros::NodeHandle n;
  ros::NodeHandle n2;
  ros::Publisher rec_flag_pub = n.advertise<std_msgs::Bool>("rec_flag", 1000);
  ros::Publisher button_state_pub = n2.advertise<std_msgs::String>("button_state", 1000);
  setup();
  while (ros::ok()) {
    ros::spinOnce();
    loop (rec_flag_pub, button_state_pub);
  }
  return 0;
}
