#include "sensor/sensor.h"
#include "sensor/l3g4200d.h"
#include "sensor/adxl345.h"

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
    ADXL345_Init();

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
	ADXL345_Recv(arg);
    }
}

void ProcessTask(void *arg){
    while(1){
        L3G4200D_Process(arg);
	ADXL345_Process(arg);
    }
}

void printFloat(float a){
    if(a < 0){ kputs("-"); a = -a;}else{kputs(" ");}
    kputs(itoa(a, 10));
    kputs(".");
    kputs( itoa( (100000 * a - 100000 * (int)a ) ,10) );
}

void TestOutput(void *arg){
    while(1){
#if 0
        kputs("vRow:   ");
        printFloat(vAttitude.row);
        kputs("\r\n");
        kputs("vPitch: ");
        printFloat(vAttitude.pitch);
        kputs("\r\n");
        kputs("vYaw:   ");
        printFloat(vAttitude.yaw);
        kputs("\r\n");

        kputs("Row:   ");
        printFloat(xAttitude.row);
        kputs("\r\n");
        kputs("Pitch: ");
        printFloat(xAttitude.pitch);
        kputs("\r\n");
        kputs("Yaw:   ");
        printFloat(xAttitude.yaw);
        kputs("\r\n");

        kputs("Row:   ");
        printFloat(aAttitude.row);
        kputs("\r\n");
        kputs("Pitch: ");
        printFloat(aAttitude.pitch);
        kputs("\r\n");
        kputs("Yaw:   ");
        printFloat(aAttitude.yaw);
        kputs("\r\n");
#endif
    }
}
