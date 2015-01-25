#include "controller/control_api.h"
#include "controller/pid.h"
#include "uart.h"
#include "task.h"
#include "shell/send.h"

extern bool controllerEnable;
extern float setPoint[NUM_RC_IN];
extern pid_context_t pids[NUM_AXIS];
extern float sensorData[NUM_AXIS];
extern float mFR,mBL,mFL,mBR;

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

float getPidParameter(enum Axis axis, enum PID_K param){
    pid_context_t *context = &pids[axis];
    float value = 0.0f;
    switch( param ){
        case KP: context->kp = value;break;
        case KI: context->ki = value;break;
        case KD: context->kd = value;break;
        case MAX: context->max = value;break;
        case MIN: context->min = value;break;
        default:;
    }
    return value;
}

void setSetPoint( enum RC_IN a, float value){
    setPoint[a] = value;
}

/**
*   basically same as setSetPoint
*   sets 4 setpoint in once
*/
void control(float v[4]){
    setPoint[0] = v[0];
    setPoint[1] = v[1];
    setPoint[2] = v[2];
    setPoint[3] = v[3];
}

void setControllerEnable( bool set ){
    controllerEnable = set;
}

#define SEND_MOTOR 1
#define SEND_RATE_PID 1
#define SEND_STAB_PID 1
#define SEND_SETPOINT 1

void sendControlInfo(){
    const uint8_t head = 0x02;
    taskENTER_CRITICAL();
    float motor[] = { mFR, mFL, mBL, mBR };
    float ratepid[] = {pids[ROLL_RATE].out, pids[PITCH_RATE].out,pids[YAW_RATE].out};
    float stabpid[] = {pids[ROLL].out, pids[PITCH].out,pids[YAW].out};
    float setpoint[] = {setPoint[THR_C]};
    taskEXIT_CRITICAL();

    if(SEND_MOTOR)SendCommand_3(head,0x00, (uint8_t *)motor, 16);
    if(SEND_RATE_PID)SendCommand_3(head,0x01, (uint8_t *)ratepid, 12);
    if(SEND_STAB_PID)SendCommand_3(head,0x02, (uint8_t *)stabpid, 12);
    if(SEND_SETPOINT)SendCommand_3(head,0x03, (uint8_t *)setpoint, 4);
}
