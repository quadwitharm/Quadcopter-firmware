#include "controller/control_task.h"
#include "controller/control_api.h"
#include "controller/pid.h"
#include "sensor/sensor.h"
#include "motor.h"
#include "task.h"

static void Controller_Task(void *args);
static void ControllerUpdate(void);

pid_context_t pids[NUM_AXIS];
float mFR,mBL,mFL,mBR;

float setPoint[NUM_RC_IN] = {};
bool controllerEnable = true;

static xTaskHandle controllerTaskHandle;

bool Init_Controller(){
    // Initialize PID structures
    stablize_pid_init(&pids[ROLL], &pids[PITCH], &pids[YAW]);
    rate_pid_init(&pids[ROLL_RATE], &pids[PITCH_RATE], &pids[YAW_RATE]);

    // FreeRTOS Task
    bool ret = xTaskCreate(Controller_Task,
            (portCHAR *)"ControllerTask",
            512,
            NULL,
            tskIDLE_PRIORITY + 2,
            &controllerTaskHandle);
    if(ret != pdPASS)return false;
    return ret;
}

extern TIM_HandleTypeDef TIM2_Handle;
void TIM2_IRQHandler(void){
    /* TIM Update event */
    if(__HAL_TIM_GET_FLAG(&TIM2_Handle, TIM_FLAG_UPDATE) != RESET) {
        if(__HAL_TIM_GET_ITSTATUS(&TIM2_Handle, TIM_IT_UPDATE) !=RESET) {
            __HAL_TIM_CLEAR_IT(&TIM2_Handle, TIM_IT_UPDATE);
            if(pdTRUE == xTaskResumeFromISR(controllerTaskHandle)){
                taskYIELD();
            }
        }
    }
}
static void Controller_Task(void *args){
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    while(1){
        if( !controllerEnable ) continue;

        ControllerUpdate();
        UpdateMotorSpeed( (float []){ mFR, mFL, mBL, mBR } );

        // Timer interrupt will wake up this task
        vTaskSuspend(controllerTaskHandle);
    }
}

#define SCALE(var,min,max) do{\
    if(var > max)var = max;\
    else if(var < min)var = min;\
}while(0)

static void ControllerUpdate(){
    // Read data, TODO: protect the data
    float sensorRoll  = xAttitude.roll;
    float sensorPitch = xAttitude.pitch;
    float sensorYaw   = xAttitude.yaw;
    float rollRate = lastAngularSpeed.roll;
    float pitchRate = lastAngularSpeed.pitch;
    float yawRate = lastAngularSpeed.yaw;

    // Calculate PIDs
    float roll_out = runPID(&pids[ROLL],
            runPID(&pids[ROLL_RATE], setPoint[ROLL_C], rollRate), sensorRoll);
    float pitch_out = runPID(&pids[PITCH],
            runPID(&pids[PITCH_RATE], setPoint[PITCH_C], pitchRate), sensorPitch);

    // Need refactor
#if 0
    if(/*rate mode*/){
        yaw_out = runPID(&pid_yaw_r,/**/,/**/);
    }else if(/*stablized mode*/){
#endif
       float yaw_out = runPID(&pids[YAW],
                runPID(&pids[YAW_RATE], setPoint[YAW_C], yawRate), sensorYaw);
#if 0
    }
#endif

    float throttle = setPoint[THR_C];/*should be radio input*/

    // Motor output
    mFR = throttle + roll_out - pitch_out + yaw_out;
    mFL = throttle - roll_out - pitch_out - yaw_out;
    mBR = throttle + roll_out + pitch_out - yaw_out;
    mBL = throttle - roll_out + pitch_out + yaw_out;

    // scale output
    SCALE(mFR,0,1);
    SCALE(mFL,0,1);
    SCALE(mBR,0,1);
    SCALE(mBL,0,1);
}
