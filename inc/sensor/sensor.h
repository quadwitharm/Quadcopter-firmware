#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "main.h"
#include "l3g4200d.h"

struct Attitude{
    float row;
    float pitch;
    float yaw;
};

struct Vector3D{
    float x;
    float y;
    float z;
};

bool InitSensorPeriph();
void SensorTask(void *);

#endif
