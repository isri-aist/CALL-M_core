/*
 kinema.h
  created 22/01/02
*/

// ensure this library description is only included once
#ifndef kinema_h
#define kinema_h

extern float REDUCTION_RATIO[];

// 0.167m/s=0.6km/h, 0.3m/s=1.08km/h, 1.0m/s=3.60km/h, 1.6666m/s=6km/h
extern float MAX_VX; // left right
extern float MAX_VY; // rear front
extern float MAX_W; // turn CCW CW

extern int motor_id[];

void vel2rotor(float *rotor_rps, float vx_mps, float vy_mps, float w_rps);
void rotor2vel(float *rotor_rps, float *vx_mps, float *vy_mps, float *w_rps);

#endif
