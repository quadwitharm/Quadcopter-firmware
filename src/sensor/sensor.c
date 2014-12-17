#include "sensor/sensor.h"

#include "task.h"

xTaskHandle recvTaskHandle;
xTaskHandle processTaskHandle;

/* Definition of shared resource */
struct Attitude xAttitude;
struct Attitude vAttitude;
struct Attitude aAttitude;

struct Vector3D position;
struct Vector3D velocity;
struct Vector3D acceleration;

void ProcessTask(void *arg);
void SensorTask(void *arg);
void TestOutput(void *arg);

bool InitSensorPeriph(){
    kputs("I2C: Initialing ...\r\n");
    if(!I2C_Init()) return false;
    kputs("I2C: Initialized\r\n");

    /* Initialize sensors on GY-801 */
    L3G4200D_Init();

    return true;
}

bool InitSensorTask(){
    /* FreeRTOS Tasks */
    portBASE_TYPE ret;
    ret = xTaskCreate(SensorTask,
            (signed portCHAR *)"IMU data fetch",
            512,
            NULL,
            tskIDLE_PRIORITY + 2,
            &recvTaskHandle);
    if(ret != pdPASS)return false;
    ret = xTaskCreate(ProcessTask,
            (signed portCHAR *)"Attitude data proccess",
            512,
            NULL,
            tskIDLE_PRIORITY + 2,
            &processTaskHandle);
    ret = xTaskCreate(TestOutput,
            (signed portCHAR *)"Test",
            512,
            NULL,
            tskIDLE_PRIORITY + 2,
            NULL);
    if(ret != pdPASS)return false;

    return true;
}

void SensorTask(void *arg){
    while(1){
        L3G4200D_Recv(arg);
    }
}

void ProcessTask(void *arg){
    while(1){
        L3G4200D_Process(arg);
    }
}

void TestOutput(void *arg){
    while(1){
    }
}
