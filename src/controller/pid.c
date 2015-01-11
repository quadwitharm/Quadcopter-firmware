#include "controller/pid.h"

//control loop has fixed frequency ,use constant dt for pid
#define FREQUENCY 60.0f
#define DT (1.0/FREQUENCY)
//angle range for stabilization pid
#define MAX_ANGLE 180.0
#define MIN_ANGLE -180.0
//output range for angle rate pid to mortor
#define MAX_ROUT 0.3
#define MIN_ROUT 0.0

float _runPID(pid_context_t * p,float error,float diff){

    float deriv;
    float output;

    //use input instead of error to avoid derivative kick
    deriv = diff / DT;

    //intergal has already included ki part
    //to avoid sudden gain caused by changing ki 
    p->integral += p->ki * error * DT;

    if(p->integral > p->max){p->integral = p->max;}
    if(p->integral < p->min){p->integral = p->min;}

    output =  error*p->kp + p->integral + deriv*p->kd;

    if(output > p->max){output = p->max;}     
    if(output < p->min){output = p->min;}
    return output;
}

float runPID(pid_context_t * p,float setpoint,float input){

    float diff = input - p->prev_in;

    p->prev_in = input;
    return _runPID(p, setpoint - input, diff);
}

float runPID_warp(pid_context_t * p,float setpoint,float input,
    float warp_max,float warp_min){

    float error = setpoint - input;
    float diff = input - p->prev_in;
    float range = warp_max - warp_min;

    //warp the values
    if(error > warp_max){
        error -= range;
    }else if(error < warp_min){
        error += range;
    }

    if(diff > warp_max){
        diff -= range;
    }else if(diff < warp_min){
        diff += range;
    }

    p->prev_in = input;
    return _runPID(p,error,diff);
}

void stablize_pid_init(pid_context_t *roll,pid_context_t *pitch,
    pid_context_t *yaw){

    roll->kp = 0.0;
    roll->ki = 0.0;
    roll->kd = 0.0;
    roll->prev_in = 0.0;
    roll->integral = 0.0;
    roll->max = MAX_ANGLE;
    roll->min = MIN_ANGLE;

    pitch->kp = 0.0;
    pitch->ki = 0.0;
    pitch->kd = 0.0;
    pitch->prev_in = 0.0;
    pitch->integral = 0.0;
    pitch->max = MAX_ANGLE;
    pitch->min = MIN_ANGLE;

    yaw->kp = 0.0;
    yaw->ki = 0.0;
    yaw->kd = 0.0;
    yaw->prev_in = 0.0;
    yaw->integral = 0.0;
    yaw->max = MAX_ANGLE;
    yaw->min = MIN_ANGLE;
}

void rate_pid_init(pid_context_t *roll_r,pid_context_t *pitch_r,
    pid_context_t *yaw_r){

    roll_r->kp = 0.0;
    roll_r->ki = 0.0;
    roll_r->kd = 0.0;
    roll_r->prev_in = 0.0;
    roll_r->integral = 0.0;
    roll_r->max = MAX_ROUT;
    roll_r->min = MIN_ROUT;

    pitch_r->kp = 0.0;
    pitch_r->ki = 0.0;
    pitch_r->kd = 0.0;
    pitch_r->prev_in = 0.0;
    pitch_r->integral = 0.0;
    pitch_r->max = MAX_ROUT;
    pitch_r->min = MIN_ROUT;

    yaw_r->kp = 0.0;
    yaw_r->ki = 0.0;
    yaw_r->kd = 0.0;
    yaw_r->prev_in = 0.0;
    yaw_r->integral = 0.0;
    yaw_r->max = MAX_ROUT;
    yaw_r->min = MIN_ROUT;
}

