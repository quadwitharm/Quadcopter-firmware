#include "controller/control_api.h"
#include "controller/pid.h"

extern bool controllerEnable;
extern float setPoint[NUM_AXIS];
extern pid_context_t pid_roll, pid_pitch, pid_yaw;

void setPidParameter( enum Axis axis, enum PID_K pid, float value){
    pid_context_t *context;
    switch( axis ){
        case ROLL : context = &pid_roll; break;
        case PITCH: context = &pid_pitch;break;
        case YAW  : context = &pid_yaw;  break;
        case ROLL_RATE : context = &pid_roll_r; break;
        case PITCH_RATE: context = &pid_pitch_r;break;
        case YAW_RATE  : context = &pid_yaw_r;  break;
        default:;
    }
    switch( pid ){
        case KP: context->kp = value;break;
        case KI: context->ki = value;break;
        case KD: context->kd = value;break;
        case MAX: context->max = value;break;
        case MIN: context->min = value;break;
        default:;
    }
}

void setSetPoint( enum Axis a, float value){
    setPoint[a] = value;
}

void setControllerEnable( bool set ){
    controllerEnable = set;
}
