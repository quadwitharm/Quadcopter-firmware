#include "controller/control_api.h"
#include "controller/pid.h"

extern bool controllerEnable;
extern float setPoint[4];
extern pid_context_t pids[];

void setPidParameter( enum Axis axis, enum PID_K param, float value){
    pid_context_t *context = &pids[axis];
    switch( param ){
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

