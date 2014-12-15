#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "main.h"
#include "sensor/i2c.h"
#include "sensor/l3g4200d.h"

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

extern struct Attitude xAttitude;
extern struct Attitude vAttitude;
extern struct Attitude aAttitude;

extern struct Vector3D position;
extern struct Vector3D velocity;
extern struct Vector3D acceleration;

bool InitSensorPeriph();
bool InitSensorTask();
void SensorTask(void *);

#endif
