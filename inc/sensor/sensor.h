#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "clib.h"

struct Angle3D{
    float roll;
    float pitch;
    float yaw;
};

extern struct Angle3D xAttitude;
extern struct Angle3D lastAngularSpeed;

bool InitSensorPeriph();
bool InitSensorTask();
void SensorEnable(bool enable);
void SensorTask(void *);
void sendSensorInfo();


#endif
