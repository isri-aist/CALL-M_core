#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <sstream>
#include "conio.h"
#include "func.h"
#include "kinema.h"
#include "servo3moog.h"

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
float cur_velx = 0.0;
float cur_vely = 0.0;
float cur_velw = 0.0;
float rea_velx = 0.0;
float rea_vely = 0.0;
float rea_velw = 0.0;

int key_code = 0;
float linear_vel = 0.5;
float rotatio_vel = 0.75 * M_PI;
bool auto_mode = false;
float chrono = 0.0;
int step = 0;
float distance_t = 20*CONTROL_INTERVAL;

void show_msg(){
  printf( "\033[%dm\033[2J\033[1;1f",0);
  printf("CONTROLS:\n\n");
  printf("  Q  W  E           up\n");
  printf("  A  S  D    left  down  right    SPACE\n\n");
  printf("Current linear speed: %f",linear_vel);
  printf("\nCurrent rotationnal speed: %f",rotatio_vel);
  printf("\n\nQ/E: Rotation\n");
  printf("W/S: Y axis move\n");
  printf("A/D: X axis move\n");
  printf("up/down: Set linear speed\n");
  printf("left/right: Set rotational speed\n");
  printf("SPACE: Switch to auto mode\n");
}

void check_keyboard(int &KEY_CODE){
  if ( _kbhit() ){
    KEY_CODE = _getche();
    auto_mode = false;
    show_msg();
    //printf("_");
    /*stringstream strs;
    strs << KEY_CODE;
    string temp_str = strs.str();
    char* char_type = (char*) temp_str.c_str();
    printf(char_type);*/
  }
  else{
    KEY_CODE = -1;
    //printf("-");
  }
}

void setup() {
  printf("Ball Wheel Drive `");
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

      linear_vel = constrain (linear_vel, -MAX_VX, MAX_VX);
      rotatio_vel = constrain (rotatio_vel, -MAX_W, MAX_W);

      check_keyboard(key_code);
      switch (key_code)
      {
      case 119: //W
        des_vely = -linear_vel;
        break;
      case 97: //A
        des_velx = -linear_vel;
        break;
      case 115: //S
        des_vely = linear_vel;
        break;
      case 100: //D
        des_velx = linear_vel;
        break;
      case 113: //Q
        des_velw = rotatio_vel;
        break;
      case 101: //E
        des_velw = -rotatio_vel;
        break;
      case 32: //space
        auto_mode = true;
        printf( "\033[%dm\033[2J\033[1;1f",0);
        printf("Press any key to stop Automatic mode...");
        break;
      case 65: //up
        linear_vel += 0.1;        
        show_msg();
        printf("\b");
        break;
      case 68: //left
        rotatio_vel -= 0.1* M_PI;
        show_msg();
        printf("\b");
        break;
      case 67: //right
        rotatio_vel += 0.1* M_PI;
        show_msg();
        printf("\b");
        break;
      case 66: //down
        linear_vel -= 0.1;
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
          des_vely = -0.3;
          des_velw = 0;
          chrono += CONTROL_INTERVAL;
          if(chrono > distance_t){
            chrono = 0;
            step ++;
          }
          break;
        case 1:
          des_velx = 0.3;
          des_vely = 0;
          des_velw = 0;
          chrono += CONTROL_INTERVAL;
          if(chrono > distance_t){
            chrono = 0;
            step ++;
          }
          break;
        case 2:
          des_velx = 0;
          des_vely = 0.3;
          des_velw = 0;
          chrono += CONTROL_INTERVAL;
          if(chrono > distance_t){
            chrono = 0;
            step ++;
          }
          break;
        case 3:
          des_velx = -0.3;
          des_vely = 0;
          des_velw = 0;
          chrono += CONTROL_INTERVAL;
          if(chrono > distance_t){
            chrono = 0;
            step = 0;
          }
          break;        
        default:
          break;
        }
      }
      else{
        chrono = 0;
        step = 0;
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
  printf("Setup FINISHED, press any key to start...");
  //show_msg();
  while (1) {
    loop ();
  }
  return 0;
}

/*
int fd;
int main() {
    fd = open("/dev/ttyUSB0", O_RDWR |O_NONBLOCK);
    if(fd < 0){
        printf("servo3motor: serial can't open, errno: %d : '%s'\n", errno,strerror(errno)); //PERSO MODIF (Error codes: https://man7.org/linux/man-pages/man2/fcntl.2.html)
        return -1;
    }
    else{
        printf("Port connected");
        return 0;
    }
}*/
