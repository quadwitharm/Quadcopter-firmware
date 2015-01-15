#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "clib.h"

struct Angle3D{
    float roll;
    float pitch;
    float yaw;
};

struct Vector3D{
    float x;
    float y;
    float z;
};

extern struct Angle3D xAttitude;
extern struct Angle3D lastAngularSpeed;

extern struct Vector3D position;
extern struct Vector3D velocity;
extern struct Vector3D acceleration;

bool InitSensorPeriph();
bool InitSensorTask();
void SensorEnable(bool enable);
void SensorTask(void *);
void sendSensorInfo();


#endif
