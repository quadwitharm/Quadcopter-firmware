#include "controller/control_api.h"
#include "controller/pid.h"

extern bool controllerUpdate;
extern float setPoint[NUM_AXIS];
extern pid_context_t pid_roll, pid_pitch, pid_yaw;

void setPidParameter( enum Axis axis, enum PID_K pid, float value){
    pid_context_t *context;
    switch( axis ){
        case ROLL : context = &pid_roll; break;
        case PITCH: context = &pid_pitch;break;
        case YAW  : context = &pid_yaw;  break;
        default:;
    }
    switch( pid ){
        case KP: context->kp = value;break;
        case KI: context->ki = value;break;
        case KD: context->kd = value;break;
        default:;
    }
}

void setSetPoint( enum Axis a, float value){
    setPoint[a] = value;
}

void setControllerUpdate( bool set ){
    controllerUpdate = set;
}
