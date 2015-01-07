#include "controller/control_task.h"
#include "motor.h"
#include "task.h"

static void Controller_Task(void *args);

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

    while(1){
        // TODO: use timer to generate more precisely frequency
        vTaskDelayUntil(&lastWakeTime, (const TickType_t)1000 / 60);

        UpdateMotorSpeed( (float []){ 0.5f,0.5f,0.5f,0.5f } );
    }
}
