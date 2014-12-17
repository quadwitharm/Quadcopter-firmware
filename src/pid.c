
#include "pid.h"

float runPID(pid_context_t * p,float setpoint,float input){

    float error;
    float deriv;
    float output;

    error = setpoint - input;
    //use input instead of error to avoid derivative kick
    deriv = (input - p->prev_in) / p->dt;

    //intergal has already included ki part
    p->integral += p->ki * error * p->dt;

    if(p->integral > p->max){p->integral = p->max;}
    if(p->integral < p->min){p->integral = p->min;}

    p->prev_in = input;
    output =  error*p->kp + p->integral + deriv*p->kd;

    if(output > p->max){output = p->max;}     
    if(output < p->min){output = p->min;}
    return output;
}

