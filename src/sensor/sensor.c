#include "sensor/sensor.h"
#include "sensor/l3g4200d.h"
#include "sensor/adxl345.h"

#include "task.h"
#include "arm_math.h"

EventGroupHandle_t xDataReady;
xTaskHandle recvTaskHandle;
xTaskHandle processTaskHandle;

/* Definition of shared resource */
struct Attitude xAttitude;
struct Attitude lastAngularSpeed;

struct Vector3D position;
struct Vector3D velocity;
struct Vector3D acceleration;

float F;

void ProcessTask(void *arg);
void SensorTask(void *arg);
void TestOutput(void *arg);

bool InitSensorPeriph(){
    kputs("I2C: Initialing ...\r\n");
    if(!I2C_Init()) return false;
    kputs("I2C: Initialized\r\n");

    /* Initialize sensors on GY-801 */
    L3G4200D_Init();
    ADXL345_Init();

    return true;
}

bool InitSensorTask(){

    xDataReady = xEventGroupCreate();

    /* FreeRTOS Tasks */
    portBASE_TYPE ret;
    ret = xTaskCreate(SensorTask,
            (portCHAR *)"IMU data fetch",
            512,
            NULL,
            tskIDLE_PRIORITY + 2,
            &recvTaskHandle);
    if(ret != pdPASS)return false;
    ret = xTaskCreate(ProcessTask,
            (portCHAR *)"Attitude data proccess",
            512,
            NULL,
            tskIDLE_PRIORITY + 3,
            &processTaskHandle);
    if(ret != pdPASS)return false;
    ret = xTaskCreate(TestOutput,
            (portCHAR *)"Test",
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
        ADXL345_Recv(arg);
    }
}

float X,Y,Z;

void ProcessTask(void *arg){
    while(1){
        xEventGroupWaitBits(xDataReady, ALL_DRDY_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

        // Read from shared memory
        struct Attitude lastAngle = {
            lastAngularSpeed.roll,
            lastAngularSpeed.pitch,
            lastAngularSpeed.yaw,
        };
        struct Attitude lastAttitude = {
            xAttitude.roll,
            xAttitude.pitch,
            xAttitude.yaw,
        };
        struct Vector3D accel = {
            ADXL345.int16.X,
            ADXL345.int16.Y,
            ADXL345.int16.Z,
        };

        xEventGroupClearBits(xDataReady, ALL_DRDY_BIT);
    }
}

void setDataReady(EventBits_t source){
    xEventGroupSetBits(xDataReady, source);
}

void TestOutput(void *arg){
    while(1){
        printFloat(X);
        kputs(",");
        printFloat(Y);
        kputs(",");
        printFloat(Z);
        kputs(",");
        printFloat(xAttitude.roll);
        kputs(",");
        printFloat(xAttitude.pitch);
        kputs(",");
        printFloat(F);
        kputs("\r\n");
        vTaskDelay(10);
    }
}
