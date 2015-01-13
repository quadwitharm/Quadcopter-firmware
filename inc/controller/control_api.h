#ifndef __CONTROL_API_H__
#define __CONTROL_API_H__

#include "clib.h"

enum Axis{ ROLL, PITCH, YAW, ROLL_RATE, PITCH_RATE, YAW_RATE,
         X, Y, Z, NUM_AXIS };
enum PID_K{ KP, KI, KD, MAX, MIN };

void setPidParameter( enum Axis, enum PID_K, float );
void setSetPoint( enum Axis, float );
void setControllerUpdate( bool );

#endif
