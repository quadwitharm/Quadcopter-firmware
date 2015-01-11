#ifndef __CONTROL_API_H__
#define __CONTROL_API_H__

#include "clib.h"

enum Axis{ ROLL, PITCH, YAW, X, Y, Z, NUM_AXIS };
enum PID_K{ KP, KI, KD };

void setPidParameter( enum Axis, enum PID_K, float );
void setSetPoint( enum Axis, float );
void setControllerUpdate( bool );

#endif
