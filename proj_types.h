#ifndef PROJ_TYPES_H
#define PROJ_TYPES_H

#include "Arduino.h"

typedef struct{
  int enc1, enc2;
  float v1e, v2e;
  float ve, we;
  float ds, dtheta;
  
  byte state;
  float v, w;
  byte solenoid_state;  

  float r1, r2, b;
} robot_t;



typedef struct{
    float dt, Ki, Kp, Kd, Kf;
    float w, w_ref;
    float e, last_e, Se;
    float m, scale;
    byte active;
} PID_t;

#endif // PROJ_TYPES_H
