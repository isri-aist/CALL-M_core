// ensure this library description is only included once
#ifndef servo3motor_h
#define servo3motor_h

#define CONTROL_INTERVAL 2

#define MAX_MOTORS (3)

int open_serial(const char *port);
void set_baudrate (unsigned int baud);
void wake_up_motors (void);

int serial_send_command (int id, const char *command, float wait_time);
int serial_receive (char *reply);
void wake_moog ();
long moog_rpm2vt (float velocity_in_rpm);

void servomotor_setup();
void servoPos_loop (float cur_angarm);
float minimum_rad_per_sec (int motor);
float limit_rpm (float *motor_rpm);
float servo3motor_loop(float *rotor_rad_per_sec, float *target_motor_rpm, float *real_motor_rpm);

#endif
