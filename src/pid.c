
#include "pid.h"

float runPID(pid_context_t * p,float setpoint,float input){

    float error;
    float deriv;

    error = setpoint - input;
    deriv = error - p->prev_error;

    p->integral += error * p->dt;

    p->prev_error = error;
    return error*p->kp + p->integral*p->ki + deriv*p->kd;
}

