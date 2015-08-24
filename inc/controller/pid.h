#ifndef __PID_H__
#define __PID_H__

#include "clib.h"

#define FREQUENCY 60.0f

typedef struct pid_context_t{
    float kp,ki,kd;
    float prev_in;
    float integral;
    float max,min;
    float out;
} pid_context_t;

float runPID(pid_context_t * p,float setpoint,float input);

float runPID_warp(pid_context_t * p,float setpoint,float input,
    float warp_max,float warp_min);

float passPID(pid_context_t * p,float setpoint,float input);

void stablize_pid_init(pid_context_t *roll,
    pid_context_t *pitch,pid_context_t *yaw);

void rate_pid_init(pid_context_t *roll_r,
    pid_context_t *pitch_r,pid_context_t *yaw_r);

void vertical_pid_init(pid_context_t *height,pid_context_t *rate);

#endif
