#include "controller/control_task.h"
#include "controller/pid.h"
#include "sensor/sensor.h"
#include "motor.h"
#include "task.h"

static void Controller_Task(void *args);
static void ControllerUpdate(void);

static pid_context_t pid_roll,pid_pitch,pid_yaw;
static pid_context_t pid_roll_r,pid_pitch_r,pid_yaw_r;
static float mFR,mBL,mFL,mBR;

bool Init_Controller(){
    bool ret = true;
    ret = xTaskCreate(Controller_Task,
            (portCHAR *)"ControllerTask",
            512,
            NULL,
            tskIDLE_PRIORITY + 4,
            NULL);
    if(ret != pdPASS)return false;
    return ret;
}

static void Controller_Task(void *args){
    TickType_t lastWakeTime;
    lastWakeTime = xTaskGetTickCount();

    // Initialize PID structures
    stablize_pid_init(&pid_roll, &pid_pitch, &pid_yaw);
    rate_pid_init(&pid_roll_r, &pid_pitch_r, &pid_yaw_r);

    while(1){
        // TODO: use timer to generate more precisely frequency
        vTaskDelayUntil(&lastWakeTime, (const TickType_t)1000 / 60);

        ControllerUpdate();

        UpdateMotorSpeed( (float []){ mFR, mFL, mBL, mBR } );
    }
}

static void ControllerUpdate(){
    // Read data, TODO: protect the data
    float sensorRoll  = xAttitude.roll;
    float sensorPitch = xAttitude.pitch;
    float sensorYaw   = xAttitude.yaw;

    // Set points
    float setRoll  = 0;
    float setPitch = 0;
    float setYaw   = 0;

    // Calculate PIDs
    float roll_out = runPID(&pid_roll,
            runPID(&pid_roll_r, setRoll, sensorRoll), sensorPitch);
    float pitch_out = runPID(&pid_pitch,
            runPID(&pid_pitch_r,setPitch, sensorPitch), sensorPitch);

    // Need refactor
#if 0
    if(/*rate mode*/){
        yaw_out = runPID(&pid_yaw_r,/**/,/**/);
    }else if(/*stablized mode*/){
#endif
       float yaw_out = runPID(&pid_yaw,
                runPID(&pid_yaw_r, setYaw, sensorYaw), sensorYaw);
#if 0
    }
#endif

    // Motor output
    mFR = + roll_out - pitch_out + yaw_out;
    mFL = - roll_out - pitch_out - yaw_out;
    mBR = + roll_out + pitch_out - yaw_out;
    mBL = - roll_out + pitch_out + yaw_out;
}

