#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"

#include "task.h"
#include "semphr.h"

#include "stm32f4xx_hal_tim.h"

bool Init_Motor();
void MotorTask(void *arg);

struct MotorSpeed{
    float motor1;
    float motor2;
    float motor3;
    float motor4;
};

extern struct MotorSpeed MotorSpeed;
extern xSemaphoreHandle MotorSem;

#endif
