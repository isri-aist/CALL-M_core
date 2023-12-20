#include <stdio.h>
#include <math.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>  
#define MAX_MOTOR (3)
#include "kinema.h"
#include "servo3moog.h"
#include "func.h"

//#define BAUDRATE B115200
#define BAUDRATE B9600

#define MAX_RPM (3800) // (3545)
#define MIN_RAD_PER_SEC (0.0 / 60.0 * 2.0 * M_PI)
#define MAX_RAD_PER_SEC (MAX_RPM / 60.0 * 2.0 * M_PI)

float inv_max_rad_per_sec = 1.0 / MAX_RAD_PER_SEC;

#define MAX_SERIALBUF (256)
char serialsendbuf[MAX_SERIALBUF];
char serialrecvbuf[MAX_SERIALBUF];
char serialcommbuf[MAX_SERIALBUF];
char reply[MAX_SERIALBUF];

/////////////////////// USB serial functions
int fd;
int open_serial(const char *port) {
  fd = open(port, O_RDWR |O_NONBLOCK);
  if(fd < 0){
    //printf("servo3motor: serial can't open '%s'\n", port); //PERSO MODIF
    printf("servo3motor: serial can't open port named: %s\nErrno: %d : '%s'\n", port, errno,strerror(errno)); //PERSO MODIF (Error codes: https://man7.org/linux/man-pages/man2/fcntl.2.html)
    return -1;
  }
  printf("servo3motor: serial opened:%s %d\n",port, fd);
  return fd;
}

void set_baudrate (unsigned int baud) {
  struct termios tio;
  memset(&tio,0,sizeof(tio));
  tio.c_cflag = CREAD | CLOCAL | CS8;
  // 受信有効 ローカルライン(モデム制御なし) データビット:8bit      
  tio.c_cflag += 0;              // ストップビット:1bit
  tio.c_cflag += 0;              // パリティ:None
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  tio.c_iflag = IGNPAR ;
  cfsetispeed(&tio, baud);  
  cfsetospeed(&tio, baud);  
  tcsetattr( fd, TCSANOW, &tio ); //set device
  if(ioctl(fd,TCSETS, &tio) < 0){
    printf("servo3motor: fd = %d, errno=%d: %s\n", fd, errno, strerror(errno));
  } else {
    printf("servo3motor: set baudrate %u\n", baud);
  }
}

///////////////////////////////////////////////////////////////////////
int serial_send_command (int id, const char *command, float wait_time) {
  if (id < 0) {
    return 0;
  }
  memset(&serialsendbuf, 0, sizeof(serialsendbuf));
  sprintf(serialsendbuf, "%c%s\r", (char) (0x80 + id), command);
  // send command
  write(fd, serialsendbuf, strlen(serialsendbuf));

  if (wait_time > 0) {
    float send_start_time = millis();
    while (1) {
      if (millis() - send_start_time > wait_time) {
        break;
      }
    }
  }
  return 1;
}
 
int serial_receive (char *reply) {
  int id = -1;
  int len=0;
  static int cbufi = 0;
  float send_start_time = millis(), max_wait = 6;
  while (1) {
    if (millis() - send_start_time > max_wait) {
      sprintf(serialrecvbuf, "RS232:No reply!\r\n");
      break;
    }
    char cc[2];
    bzero(cc,sizeof(cc));
    int rt = read (fd, cc, 1);
    if (rt > 0) {
      char c = cc[0];
      if(c > 0x80) {
        id = c - 0x80;
        cbufi = 0;
        memset(&serialrecvbuf, 0, sizeof(serialrecvbuf));
      }
      if (c == '\r') {
        reply[cbufi] = c;
        sprintf(serialrecvbuf, "%s",reply);
        break;
      }
      reply[cbufi] = c;
      cbufi++;
      if (cbufi >= MAX_SERIALBUF) {
        cbufi = MAX_SERIALBUF - 1;
      }
    }
  }
  return id;
}

#define WAKE_WAIT (10)
void wake_moog () {
printf("wake_moog...\n");
  for (int i = 0; i < 2; i++) {
    serial_send_command (0, "WAKE", 100);
    serial_send_command (0, "WAKE", WAKE_WAIT);
    serial_send_command (0, "ECHO", WAKE_WAIT);
    //serial_send_command (0, "RUN?", WAKE_WAIT); // delete because its bad for RS485
    serial_send_command (0, "END", WAKE_WAIT);
    serial_send_command (0, "X", WAKE_WAIT);
  }
#if 1
printf("set_baudrate...\n");
  for (int i = 0; i < 3; i++) {
    //serial_send_command (0, "BAUD115200", 0);
    serial_send_command (0, "OCHN(RS2,0,N,115200,1,8,C,1000)", 10);
    serial_send_command (0, "OCHN(RS4,1,N,115200,1,8,C,1000)", 10);
  }
  set_baudrate (B115200);
#endif
printf("set_id...\n");
  if (1 == 1) {
    serial_send_command (0, "SADDR0", WAKE_WAIT);
    serial_send_command (0, "ECHO_OFF", WAKE_WAIT);
    for (int motor = 0; motor < MAX_MOTORS; motor++) {
      sprintf(serialcommbuf, "SADDR%d", motor_id[motor]);
      serial_send_command (0, serialcommbuf, WAKE_WAIT);
printf("%s\n", serialcommbuf);
      serial_send_command (motor_id[motor], "ECHO", WAKE_WAIT);
      serial_send_command (motor_id[motor], "SLEEP", WAKE_WAIT);
    }
    serial_send_command (0, "WAKE", WAKE_WAIT);
    serial_send_command (0, "ECHO", WAKE_WAIT);
  } else {
    serial_send_command (0, "ADDR=CADDR", WAKE_WAIT);
  }
printf("Mode Vel...\n");
  serial_send_command (0, "ZS", WAKE_WAIT);
  serial_send_command (0, "EIGN(W,0)", WAKE_WAIT);
  //serial_send_command (0, "PRINT(ADDR)", 0);
  serial_send_command (0, "MV", 0);
  serial_send_command (0, "ADT=800", 0);
  serial_send_command (0, "RRES", 0);
  serial_send_command (0, "RBAUD(0)", 0);
  //serial_send_command (motor_id[2], "ECHO_OFF", 0);
printf("done\n");
}

#define MOOG_ENC_COUNT_PER_REV (4000.0)
#define MOOG_SAMPLE_RATE (8000.0)
#define MOOG_MAX_RANGE (2147483647L)

float moog_vt2rpm (long vt) {
 return (float) vt * MOOG_SAMPLE_RATE / (MOOG_ENC_COUNT_PER_REV * 65536.0);
}

long moog_rpm2vt (float velocity_in_rpm) {
  float velocity_in_rps = velocity_in_rpm / 60.0;
  long vt = (long) (velocity_in_rps
           * (MOOG_ENC_COUNT_PER_REV / MOOG_SAMPLE_RATE) * 65536.0);
  if (vt > MOOG_MAX_RANGE) {
    vt = MOOG_MAX_RANGE;
  }
  if (vt < -MOOG_MAX_RANGE) {
    vt = -MOOG_MAX_RANGE;
  }
  return vt;
}

///////////////////////////////////////////////////////////////////////
void wake_up_motors () {
  wake_moog ();
}

int servomotor_setup(const char *SEREAL_PORT) {
  printf("servomotor_setup...\n");
  fd = open_serial(SEREAL_PORT);
  if(fd>=0){
    set_baudrate (B9600);
    wake_up_motors ();
    inv_max_rad_per_sec = 1.0 / MAX_RAD_PER_SEC;
    printf("servomotor_setup...DONE\n");
  }
  return fd;
}

void servoPos_loop (float cur_angarm) {
  return;
}

float minimum_rad_per_sec (int motor) {
  return fabs(MIN_RAD_PER_SEC / REDUCTION_RATIO[motor]);
}

float restrict_rad_per_sec (float *rotor_rad_per_sec) {
  float max_ratio = 1.0;
  int low_rev_count = 0;
  for (int motor = 0; motor < 3; motor++) {
    float rev = fabs(rotor_rad_per_sec[motor] * REDUCTION_RATIO[motor]);
    float ratio = rev * inv_max_rad_per_sec;
    if (max_ratio < ratio) {
       max_ratio = ratio;
    }
    if ((rev > 0.0) && (rev < MIN_RAD_PER_SEC)) {
      low_rev_count++;
    }
  }
  if (low_rev_count > 1) {
    return 0.0;
  }

  //for (int i = 0; i < MAX_SBUF; i++) {
    //serialbuf[i] = 0;
  //}

  return (1.0 / max_ratio);
}

///////////////////////////////////////////////////////////////////////
float servo3motor_loop(float *rotor_rad_per_sec, float *target_motor_rpm, float *real_motor_rpm) {
  unsigned long currtime = millis();
  static unsigned long lastmovetime = currtime;
  static float last_target_motor_rpm[] = {0, 0, 0};
  //float target_speed[] = {0, 0, 0};
  float gain = restrict_rad_per_sec (rotor_rad_per_sec);
  int toobigacc = 0, moving_flag = 0;
  for (int motor = 0; motor < MAX_MOTORS; motor++) {
    rotor_rad_per_sec[motor] *= gain;
    float motor_rad_per_sec = rotor_rad_per_sec[motor] * REDUCTION_RATIO[motor];
    target_motor_rpm[motor] = motor_rad_per_sec * 60.0 / (2.0 * M_PI);
    if (fabs(target_motor_rpm[motor] - last_target_motor_rpm[motor]) > 1000.0) {
      toobigacc = 1;
    }
    last_target_motor_rpm[motor] = target_motor_rpm[motor];
  }
  for (int motor = 0; motor < MAX_MOTORS; motor++) {
    long vt = moog_rpm2vt (target_motor_rpm[motor]);
    if (abs(vt) > 0) {
      moving_flag = 1;
    }
    sprintf(serialcommbuf, "VT=%ld", vt);
    serial_send_command(motor_id[motor], serialcommbuf, 0);
  }
  // if (toobigacc == 0)
  if (moving_flag == 1) {
    serial_send_command(0, "G", 0);
    lastmovetime = currtime;
  } else {
    serial_send_command(0, "X", 0);
  }
  //  else {
  //  printf("too big acc!\n");
  //}
  if (currtime - lastmovetime > 1000) {
    lastmovetime = currtime;
    serial_send_command (0, "ZS", 0);
    serial_send_command (0, "EIGN(W,0)", 0);
  }

  if (0) {
    static int itvl = 0;
    if(++itvl >= 10) {
      itvl = 0;
      //printf("%f %f %f: ", cur_velx, cur_vely, cur_velw);
      printf("%d, %f %f %f\n", moving_flag, target_motor_rpm[0], target_motor_rpm[1], target_motor_rpm[2]);
    }
  }

  // int id = serial_receive (reply);
  // printf("%d:%s\n", id, reply);
  return gain;
}
