#ifndef __PID_H__
#define __PID_H__

#include "main.h"

typedef struct pid_context_t{
    float dt;
    float kp,ki,kd;
    float prev_error;
    float integral;
} pid_context_t;

float runPID(pid_context_t * p,float setpoint,float input);

#endif
