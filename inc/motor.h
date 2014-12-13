#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"

#include "task.h"

#include "stm32f4xx_hal_tim.h"

bool Init_Motor();
void MotorTask(void *arg);

#endif
