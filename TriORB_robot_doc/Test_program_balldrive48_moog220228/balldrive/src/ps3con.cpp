#include <stdio.h>
#include <math.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include <cstdio>
#include <sstream>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <linux/joystick.h>
#include "func.h"
#include "kinema.h"
#include "servo3motor.h"

#define JOY_DEV "/dev/input/js0"

//////////////////////////////////////////////////////////
// JA_L_UD        JA_R_UD 
//   ^              ^
// <   > JA_L_LR  <   > JA_R_LR
//   v              v
#define JA_L_LR 0 // -32767 ~ 0 ~ 32767 Left joy stick
#define JA_L_UD 1 // -32767 ~ 0 ~ 32767 Left joy stick
#define JA_LT 2 // -32767 ~ 32767 Left trigger
#define JA_R_LR 3 // -32767 ~ 0 ~ 32767 Right joy stick
#define JA_R_UD 4 // -32767 ~ 0 ~ 32767 Right joy stick
#define JA_RT 5 // -32767 ~ 32767 Right trigger
#define JA_C_LR 6 // -32767, 0, 32767 Left cross key
#define JA_C_UD 7 // -32767, 0, 32767 Left cross key

//      JB_Y
//  JB_X    JB_B
//      JB_A
#define JB_A 0
#define JB_B 1
#define JB_X 2
#define JB_Y 3
#define JB_L1 4
#define JB_R1 5
#define JB_BACK 6
#define JB_START 7
#define JB_GUIDE 8
#define JB_L_PUSH 9 // push left joystick
#define JB_R_PUSH 10 // push right joystick

/////////////////////////////////////////////////////////
#define sumofsqr(a, b, c) ((a)*(a)+(b)*(b)+(c)*(c))

void check_state_zengo (int button, float *velx, float *vely, float *velw, int *flag);
void check_state_maru (int button, float *velx, float *vely, float *velw,int *flag);
void check_state_sikaku (int button, float *velx, float *vely, float *velw,int *flag);
void check_state_sankaku (int button, float *velx, float *vely, float *velw,int *flag);

/////////////////////////////////////////////////////
using namespace std;
int num_of_axis(0), num_of_buttons(0);
char name_of_joystick[80];
vector<char> joy_button;
vector<int> joy_axis;

int joy_fd;

void open_js(void) {
  if((joy_fd = open(JOY_DEV,O_RDONLY)) < 0) {
    cerr<<"Failed to open "<<JOY_DEV<<endl;
    return;
  }
  ioctl(joy_fd, JSIOCGAXES, &num_of_axis);
  ioctl(joy_fd, JSIOCGBUTTONS, &num_of_buttons);
  ioctl(joy_fd, JSIOCGNAME(80), &name_of_joystick);
  joy_button.resize(num_of_buttons, 0);
  joy_axis.resize(num_of_axis, 0);
  cout<<"Joystick: "<<name_of_joystick<<endl;
   //<<"  axis: "<<num_of_axis<<endl;
   //<<"  buttons: "<<num_of_buttons<<endl;

  fcntl(joy_fd, F_SETFL, O_NONBLOCK);   // using non-blocking mode
}

/////////////////////////////////////////////////////
char length_label[] = {"m"};
char speed_label[] = {"m/s"};
char accel_label[] = {"m/s^2"};

char moving_length_label[] = {"moving_length"};
int moving_length_num = 0, max_moving_length_num = 7;
float moving_length_array[] = {0.005, 0.01, 0.05, 0.1, 0.2, 0.5, 1.0};
float moving_length = moving_length_array[0];

char moving_speed_label[] = {"moving_speed"};
int moving_speed_num = 0, max_moving_speed_num = 6;
float moving_speed_array[] = {0.01, 0.05, 0.1, 0.3, 0.5, 1.0}; // [m/s]
float moving_speed = moving_speed_array[0];

float moving_accel = 0.75;

void printmag (int n, char c)
{
  for (int i = 0; i < n; i++) {
    printf("%c", c);
  }
  printf("\n");
}

class Joymenu {
  public:
  char *label;
  char *unit;
  char digic;
  float va;
  int vnum = 0;
  int max_vnum;
  float *varr;

  unsigned long wait_time = 150, wait_time2 = 2000;
  unsigned long last_t = millis ();

  Joymenu(char *_label, char *_unit, char _digic, float _va, int _max_vnum, float *_varr)
  {
    label = _label;
    unit = _unit;
    max_vnum = _max_vnum;
    va = _va;
    varr = _varr;
    digic = _digic;
  }

  float read(int ja, unsigned long curr_t) {
    unsigned long elasped = curr_t - last_t;
    if (elasped > wait_time) {
      if (ja != 0) {
        int num = vnum;
        // if (elasped < wait_time2) 
        {
          if (ja < -100) {
            num--;
            if (num < 0) {
              num = 0;
            }
          }
          if (ja > 100) {
            num++;
            if (num >= max_vnum) {
              num = max_vnum - 1;
            }
          }
        }
        if (num != vnum /* || elasped > wait_time2 */) {
          last_t = curr_t;
          // if (elasped < wait_time2) 
          {
              vnum = num;
              va = varr[num];
          }
          printf("%s[%d/%d] = %5.3f [%s] ", label, vnum + 1, max_vnum, va, unit);
          printmag (vnum + 1, digic);
        }
      }
    }
    return va;
  }
};

class Joymag {
  public:
  char *vlabel;
  char *unit;
  char digic;
  float va = 0.05;
  float vstep = 0.05;
  float min_va = 0.05;
  float max_va = MAX_VY;

  unsigned long wait_time = 150, wait_time2 = 2000;
  unsigned long last_t = millis ();

  Joymag (char *_vlabel, char *_unit, char _digic, float _va, float _vstep, float _min_va, float _max_va) {
    vlabel = _vlabel;
    unit = _unit;
    digic = _digic;
    va = _va;
    vstep = _vstep;
    min_va = _min_va;
    max_va = _max_va;
  }

  float read(int ja, unsigned long curr_t) {
    unsigned long elasped = curr_t - last_t;
    if (ja != 0) {
      if (elasped > wait_time) {
        last_t = curr_t;
        if (elasped < wait_time2) {
          if (ja > 100) {
            va += (float) vstep;
          }
          if (ja < -100) {
            va -= (float) vstep;
          }
          va = constrain(va, min_va, max_va);
          printf("%s %6.4f / %6.4f [%s] ", vlabel, va, MAX_VY, unit);
          printmag (int(va / min_va), digic);
        }
      }
    }
    return va;
  }
};

void joystick_loop(float *velx, float *vely, float *velw, float *velarm, int *flag)
{
  js_event js;
  std::stringstream ss;

  read(joy_fd, &js, sizeof(js_event));

  switch (js.type & ~JS_EVENT_INIT) {
  case JS_EVENT_AXIS:
    if ((int)js.number >= joy_axis.size()) {cerr<<"err:"<<(int)js.number<<endl;}
    joy_axis[(int)js.number] = js.value;
    // printf("Joy_axis %d %d\n", (int)js.number, js.value);
    break;
  case JS_EVENT_BUTTON:
    if ((int)js.number >= joy_button.size())  {cerr<<"err:"<<(int)js.number<<endl;}
    joy_button[(int)js.number] = js.value;
    // printf("Joy_but %d %d\n", (int)js.number, js.value);
    break;
  }

  ///////
  unsigned long curr_time = millis();
  unsigned long wait_time = 150, wait_time2 = 2000;

  static Joymenu jm_mvlen
    = Joymenu(moving_length_label, length_label, 'M', moving_length, max_moving_length_num, moving_length_array);
  moving_length = jm_mvlen.read(-joy_axis[JA_C_UD] * joy_button[JB_START], curr_time);

  static Joymenu jm_mvspd
    = Joymenu(moving_speed_label, speed_label, 'S', moving_speed, max_moving_speed_num, moving_speed_array);
  moving_speed = jm_mvspd.read(joy_axis[JA_C_LR] * joy_button[JB_START], curr_time);

  ////////
  if (joy_button[JB_START] + joy_button[JB_BACK] == 0) {
    static float joyspeed = 0.05;
    const float joystep = 0.05, min_joyspeed = 0.05, max_joyspeed = MAX_VY;
    char joyspd_label[] = {"joyspeed"};
    static Joymag jm_joyspd
    = Joymag (joyspd_label, speed_label, '>', joyspeed, joystep, min_joyspeed, max_joyspeed);
    joyspeed = jm_joyspd.read(joy_axis[JA_C_LR], curr_time);

    float mag = joyspeed;
    if (joy_button[JB_L_PUSH] + joy_button[JB_R_PUSH] > 0) {
      mag = min_joyspeed;
    }
    mag /= 32767.0;
    *velx =  (float) joy_axis[JA_L_LR] * mag;
    // *velx += (float) (joy_axis[JA_RT] - joy_axis[JA_LT]) / 32767.0 * mag;
    *vely = -(float) joy_axis[JA_L_UD] * mag;
    *velw = -(float) joy_axis[JA_R_LR] * mag / MAX_VX * MAX_W;
    *velarm = (float) joy_axis[JA_R_UD] / 32767.0;
  }

  //if ((joy_button[JB_START] > 0) && (joy_button[JB_BACK] > 0))
  if (joy_button[JB_GUIDE] > 0) {
    wake_up_motors ();
  }

  check_state_zengo (joy_button[JB_A], velx, vely, velw, flag); //auto drive
  check_state_maru (joy_button[JB_B], velx, vely, velw, flag); //auto drive
  check_state_sikaku (joy_button[JB_X], velx, vely, velw, flag); //auto drive
  check_state_sankaku (joy_button[JB_Y], velx, vely, velw, flag); //auto drive
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define STATE_STOP 0
#define STATE_READY 1
#define STATE_GO_1 2
#define STATE_GO_2 3
#define STATE_GO_3 4
#define STATE_GO_4 5
#define STATE_GO_5 6
#define STATE_GO_6 7
#define STATE_GO_7 8
#define STATE_GO_8 9
#define STATE_GO_9 10
#define STATE_GO_10 11
#define STATE_GO_11 12
#define STATE_GO_12 13
#define STATE_GO_13 14
#define STATE_GO_14 15

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// vmax +----+           
//     /      \      vt /\
//    /        \       /  \
// --+          +-- --+    +--
//   0  t1   t2 t3    0 t1 t3
//
// t1 = vmax / acc
// len = t1 * vmax + (t2 - t1) * vmax = (t1 + (t3 - 2*t1)) * vmax = (t3 - t1) * vmax
// t3 = t1 + len / vmax
//
//         
//   vt /\
//     /  \
//  --+    +--
//    0 t1 t3
//
// t1 = vt / acc
// len = vt * t1 = vt^2 / acc
// vt = sqrt(len * acc)
// t1 = vt / acc

float sqr_time (float len, float vmax)
{
  float t3 = len / vmax;
  return t3;
}

float sqr_vel (float len, float vmax, float t)
{
  float vel = 0.0;
  float t3 = len / vmax;
  if (t < t3) {
    vel = vmax;
  } else {
    vel = 0.0; // STOP
  }
  return vel;
}

#define TRAPEZOID_VEL 0.1

float trapezoid_time (float len, float vmax, float acc)
{
  float t3;
  if (vmax > TRAPEZOID_VEL) {
    float t1 = vmax / acc;
    t3 = t1 + len / vmax;
    if (t1 * vmax > len) {
      float vt = sqrt(len * acc);
      t1 = vt / acc;
      t3 = t1 * 2.0;
    }
  } else {
    t3 = sqr_time (len, vmax);
  }
  return t3;
}

float trapezoid_vel (float len, float vmax, float acc, float t)
{
  float vel = 0.0;
  if (vmax > TRAPEZOID_VEL) {
    float t1 = vmax / acc;
    float t2 = len / vmax;
    float t3 = t2 + t1;
    if (t1 * vmax < len) {
      if (t < t1) {
        vel = vmax * t / t1;
      } else if (t < t2) {
        vel = vmax;
      } else if (t < t3) {
        vel = vmax - vmax * (t - t2) / t1;
      } else {
        vel = 0.0; // STOP
      }
    } else {
      float vt = sqrt(len * acc);
      t1 = vt / acc;
      t3 = t1 * 2.0;
      if (t < t1) {
        vel = vt * t / t1;
      } else if (t < t3) {
        vel = vt - vt * (t - t1) / t1;
      } else {
        vel = 0.0; // STOP
      }
    }
    if (0) {
      static int itvl = 0;
      if(++itvl >= 50) {
        itvl = 0;
        printf("t=%f, vel=%f, t1=%f, t2=%f, t3=%f\n", t, vel, t1, t2, t3);
      }
    }
  } else {
    vel = sqr_vel (len, vmax, t);
  }
  return vel;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void check_state (char *label, int button, int *last_button, int *state,
 unsigned long curr_t, unsigned long *start_t,
 int *duration_arr, float *velx, float *vely, float *velw, int *flag) {
  if (button > 0) { // joystick is center and button is pressed
    if (*last_button == 0) { // button is pressed just now
      *start_t = curr_t;
      *state = STATE_STOP; // cancel if moving
      *flag = 0;
      // printf("stop\n");
      // return;
    }
    if (*last_button > 0) { // start after pressed long
      if (*state == STATE_STOP) {
        if (curr_t - *start_t > (unsigned long) duration_arr[STATE_STOP]) {
          printf("START %s %f [m], %f [m/s]\n", label, moving_length, moving_speed);
          *flag = 2;
          *start_t = curr_t;
          (*state)++;
        }
      }
    }
    *last_button = button;
  }
  if (*state != STATE_STOP) {
    int duration = (int) (duration_arr[*state]);
    int moving_time = (int) (curr_t - *start_t);
    if (sumofsqr(*velx, *vely, *velw) > 0) {
      *state = STATE_STOP;
      *flag = 0;
      *last_button = 0;
      printf("INTERRUPTTED ");
      duration = -1;
    }
    if (duration < 0) { // work is finished
        *state = STATE_STOP;
        *flag = 0;
        *last_button = 0;
        printf("STOP %s\n", label);
    } else 
    if (moving_time >= duration) { // next step
      *start_t = curr_t;
      (*state)++;
    }
  }
}

void move_state (int *state, unsigned long curr_t, unsigned long *start_t,
 int *duration_arr, float *vx_arr, float *vy_arr, float *vw_arr, float *velx, float *vely, float *velw,
 int *flag) {
  static int last_state = STATE_STOP;
  int st = *state;
  if (st != STATE_STOP) { // move
    if (st != last_state) {
      last_state = st;
      printf("state %d: %f [sec]", st, duration_arr[st] * 1e-3);
      printf(" %f %f %f\n", vx_arr[st], vy_arr[st], vw_arr[st]);
    }
    int moving_time = (int) (curr_t - *start_t);
    float t = float(moving_time) * 1e-3;
    float v = trapezoid_vel (moving_length, moving_speed, moving_accel, t);
    float vx = vx_arr[st] * v;
    float vy = vy_arr[st] * v;
    float vw = vw_arr[st] * v;
    vx = constrain(vx, -2.0, 2.0);
    vy = constrain(vy, -2.0, 2.0);
    vw = constrain(vw, -2.0, 2.0);
    if (0) {
      static int itvl = 0;
      if(++itvl >= 50) {
        itvl = 0;
        printf("%d/%d: %f %f %f\n", moving_time, duration_arr[st], vx, vy, vw);
      }
    }
    *velx = vx;
    *vely = vy;
    *velw = vw;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Draw GO and BACK
void check_state_zengo (int button, float *velx, float *vely, float *velw, int *flag)
{
  const int restt = 1000;
  int movet = int(1e3 * trapezoid_time(moving_length, moving_speed, moving_accel));
  int duration_array[] = {1000, 0, movet, restt, movet, -1, -1};
  float vx_array[] = {0, 0,  0.0, 0,  0.0, 0, 0, 0, 0, 0, 0};
  float vy_array[] = {0, 0, +1.0, 0, -1.0, 0, 0, 0, 0, 0, 0};
  float vw_array[] = {0, 0,  0.0, 0,  0.0, 0, 0, 0, 0, 0, 0};
  static int last_button = 0;
  static int moving_state = STATE_STOP;
  static unsigned long start_moving_time;
  unsigned long current_time = millis();

  char label[] = "GO&BACK";
  check_state (label, button, &last_button, &moving_state, current_time, &start_moving_time,
    duration_array, velx, vely, velw, flag);

  move_state (&moving_state, current_time, &start_moving_time,
    duration_array, vx_array, vy_array, vw_array, velx, vely, velw, flag);
} // check_state_zengo

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Draw CIRCLE
void check_state_maru (int button, float *velx, float *vely, float *velw, int *flag)
{
  const int restt = 1000;
  int movet = int(1e3 * trapezoid_time(moving_length * M_PI, moving_speed, moving_accel));
  int duration_array[] = {1000, 0, movet, -1, -1, -1, -1, -1, -1};
  static int last_button = 0;
  static int moving_state = STATE_STOP;
  static unsigned long start_moving_time;
  unsigned long current_time = millis();
  int duration = (int) (duration_array[moving_state]);
  int moving_time = (int) (current_time - start_moving_time);

  char label[] = "CIRCLE";
  check_state (label, button, &last_button, &moving_state, current_time, &start_moving_time,
    duration_array, velx, vely, velw, flag);

  if (moving_state == STATE_GO_1) {
    float theta = (float) moving_time / (float) duration;
    theta = constrain (theta, 0.0, 1.00);
    theta *= 2.0 * M_PI;
    float t = float(moving_time) * 1e-3;
    float v = trapezoid_vel (moving_length * M_PI, moving_speed, moving_accel, t);
    float vx = -cos(theta) * v;
    float vy = +sin(theta) * v;
    float vw = 0.0;
    vx = constrain(vx, -2.0, 2.0);
    vy = constrain(vy, -2.0, 2.0);
    vw = constrain(vw, -2.0, 2.0);
    if (0) {
      static int itvl = 0;
      if(++itvl >= 100) {
        itvl = 0;
        printf("%d/%d %d: %f %f %f\n", moving_time, duration, int(theta / M_PI * 180.0), vx, vy, vw);
      }
    }
    *velx = vx;
    *vely = vy;
    *velw = vw;
  }
} // check_state_maru

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Draw SQUARE
void check_state_sikaku (int button, float *velx, float *vely, float *velw, int *flag)
{
  const int restt = 1000;
  int movet = int(1e3 * trapezoid_time(moving_length, moving_speed, moving_accel));
  int duration_array[] = {1000, 0, movet, restt, movet, restt, movet, restt, movet, -1, -1};
  float vx_array[] = {0, 0,  0.0, 0, +1.0, 0,  0.0, 0, -1.0, 0.0, 0.0};
  float vy_array[] = {0, 0, +1.0, 0,  0.0, 0, -1.0, 0,  0.0, 0.0, 0.0};
  float vw_array[] = {0, 0,  0.0, 0,  0.0, 0,  0.0, 0,  0.0, 0.0, 0.0};
  static int last_button = 0;
  static int moving_state = STATE_STOP;
  static unsigned long start_moving_time;
  unsigned long current_time = millis();

  char label[] = "SQURE";
  check_state (label, button, &last_button, &moving_state, current_time, &start_moving_time,
    duration_array, velx, vely, velw, flag);

  move_state (&moving_state, current_time, &start_moving_time,
    duration_array, vx_array, vy_array, vw_array, velx, vely, velw, flag);
} // check_state_sikaku

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Draw TRIANGLE
void check_state_sankaku (int button, float *velx, float *vely, float *velw, int *flag)
{
  const int restt = 1000;
  int movet = int(1e3 * trapezoid_time(moving_length, moving_speed, moving_accel));
  int duration_array[] = {1000, 0, movet, restt, movet, restt, movet, -1, -1};
  const float len2 = 0.5, len3 = 1.7320508 / 2.0;
  float vx_array[] = {0, 0, -len2, 0, +1.0, 0, -len2,  0.0, 0.0, 0.0};
  float vy_array[] = {0, 0, +len3, 0,  0.0, 0, -len3,  0.0, 0.0, 0.0};
  float vw_array[] = {0, 0,   0.0, 0,  0.0, 0,   0.0,  0.0, 0.0, 0.0};
  static int last_button = 0;
  static int moving_state = STATE_STOP;
  static unsigned long start_moving_time;
  unsigned long current_time = millis();

  char label[] = "TRIANGLE";
  check_state (label, button, &last_button, &moving_state, current_time, &start_moving_time,
    duration_array, velx, vely, velw, flag);

  move_state (&moving_state, current_time, &start_moving_time,
    duration_array, vx_array, vy_array, vw_array, velx, vely, velw, flag);
} // check_state_sankaku
