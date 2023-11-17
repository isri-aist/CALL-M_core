#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <sstream>
#include "conio.h"
#include "func.h"
#include "kinema.h"
#include "servo3moog.h"
#include "ps3con.h"

using namespace std;

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

void loop() 
{
    unsigned long current_time = millis();
    static unsigned long last_time = current_time;
    
    float rotor_rad_per_sec_gain = 1.0;

    ////////////////////////////////////////////////////////////
    // control interval
    if (current_time - last_time > CONTROL_INTERVAL) {
      last_time = current_time;

      joystick_loop(&des_velx, &des_vely, &des_velw, &des_velarm, &recording_flag); // one 3-axes
      des_velarm *= M_PI_2;
      
      switch (recording_flag) {
      case 2:
        printf("Run automatically");
        break;
      case 1:
        printf("Pushed Button");
        break;
      case 0:
        //printf("No push");
        break;
      }

      des_velx = constrain(des_velx, -MAX_VX, MAX_VX);
      des_vely = constrain (des_vely, -MAX_VY, MAX_VY);
      des_velw = constrain (des_velw, -MAX_W, MAX_W);
      
      cur_velx = des_velx;
      cur_vely = des_vely;
      cur_velw = des_velw;

      // convert vehicle vel. -> rotor vel.
      vel2rotor (rotor_rad_p_sec, cur_velx, cur_vely, cur_velw);

      // move motor
      rotor_rad_per_sec_gain = servo3motor_loop (rotor_rad_p_sec, target_motor_rpm, real_motor_rpm);
    } // if (current_time - last_time > CONTROL_INTERVAL)

} // loop

int main(int argc, char **argv)
{
  printf("Setup START\n");
  setup();
  while (1) {
    loop ();
  }
  return 0;
}

