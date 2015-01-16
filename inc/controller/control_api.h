#ifndef __CONTROL_API_H__
#define __CONTROL_API_H__

#include "clib.h"

enum Axis{ ROLL, PITCH, YAW, ROLL_RATE, PITCH_RATE, YAW_RATE,
        NUM_AXIS };
enum RC_IN{ ROLL_C, PITCH_C ,YAW_C ,THR_C ,NUM_RC_IN};
enum PID_K{ KP, KI, KD, MAX, MIN };

void setPidParameter( enum Axis, enum PID_K, float );
float getPidParameter( enum Axis, enum PID_K);
void setSetPoint( enum RC_IN, float );
void control(float v[4]);
void setControllerEnable( bool );
void sendControlInfo();

#endif
