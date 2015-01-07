#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "main.h"
#include "event_groups.h"
#include "sensor/i2c.h"

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

extern EventGroupHandle_t xDataReady;
void setDataReady(EventBits_t source);

bool InitSensorPeriph();
bool InitSensorTask();
void SensorTask(void *);

#define ADXL345_DRDY_BIT  0b00000001
#define L3G4200D_DRDY_BIT 0b00000010
#define HMC58831_DRDY_BIT 0b00000100
#define ALL_DRDY_BIT \
    (ADXL345_DRDY_BIT | L3G4200D_DRDY_BIT /* | HMC58831_DRDY_BIT */ )

#endif
