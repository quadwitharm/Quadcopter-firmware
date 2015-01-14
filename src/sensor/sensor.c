#include "sensor/sensor.h"
#include "sensor/l3g4200d.h"
#include "sensor/adxl345.h"
#include "sensor/hmc5883l.h"
#include "sensor/i2c.h"
#include "sensor/filter.h"

#include "task.h"
#include "arm_math.h"

/* Arise if all data ready */
xTaskHandle recvTaskHandle;
xTaskHandle processTaskHandle;

bool accel_avail, gyro_avail, compass_avail;

/* Parameter of Kalman Filter */
struct KalmanParameter Kroll;
struct KalmanParameter Kpitch;

/* Definition of shared resource */
struct Angle3D xAttitude;
struct Angle3D lastAngularSpeed;

struct Vector3D position;
struct Vector3D velocity;
struct Vector3D acceleration;

void SensorTask(void *arg);
void DRDY_INT_Init();

bool InitSensorPeriph(){
    kputs("I2C: Initialing ...\r\n");
    if(!I2C_Init()) return false;
    kputs("I2C: Initialized\r\n");

    /* Initialize sensors on GY-801 */
    L3G4200D_Init();
    ADXL345_Init();

    accel_avail = gyro_avail = compass_avail = false;

    return true;
}

bool InitSensorTask(){
    /* FreeRTOS Tasks */
    portBASE_TYPE ret;
    ret = xTaskCreate(SensorTask,
            (portCHAR *)"IMU data fetch",
            512,
            NULL,
            tskIDLE_PRIORITY + 3,
            &recvTaskHandle);
    if(ret != pdPASS)return false;
    return true;
}

float getRoll(float x, float y, float z,float estimateRoll){
    if(x == 0)x = 0.00001;
    if(z == 0)z = 0.00001;
    int atan = 180 * atanf( y / sqrtf(x * x + z * z)) / 3.1415926;

    if(z < 0){
        if(y > 0) return 180 - atan;
        else return - 180 - atan;
    }
    return atan;
}

float getPitch(float x, float y, float z,float estimatePitch){
    if(z == 0)z = 0.00001;
    if(y == 0)y = 0.00001;
    int atan = 180 * atanf( x / sqrtf(z * z + y * y)) / 3.1415926;
    if(z < 0){
        if(x > 0) return 180 - atan;
        else return - 180 - atan;
    }
    return atan;
}

void Process(){
    const float Time = 0.005f;
    const float gyroScale = 0.0074f;

        // Read from shared memory
        struct Vector3D accel = {
            ADXL345.int16.X,
            ADXL345.int16.Y,
            ADXL345.int16.Z,
        };
        struct Angle3D angularSpeed = {
            L3G4200D.int16.X,
            L3G4200D.int16.Y,
            L3G4200D.int16.Z,
        };

        acceleration = accel;
        lastAngularSpeed = angularSpeed;

        // TODO: Kalman Filter

        // Convert to 360 degree
        struct Angle3D gyroEstimateAngle = {
            xAttitude.roll  + angularSpeed.roll  * Time * gyroScale,
            xAttitude.pitch + angularSpeed.pitch * Time * gyroScale,
            xAttitude.yaw   + angularSpeed.yaw   * Time * gyroScale,
        };
        if(gyroEstimateAngle.roll  > 180) gyroEstimateAngle.roll  -= 360;
        if(gyroEstimateAngle.roll  <-180) gyroEstimateAngle.roll  += 360;
        if(gyroEstimateAngle.pitch > 180) gyroEstimateAngle.pitch -= 360;
        if(gyroEstimateAngle.pitch <-180) gyroEstimateAngle.pitch += 360;
        if(gyroEstimateAngle.yaw   > 180) gyroEstimateAngle.yaw   -= 360;
        if(gyroEstimateAngle.yaw   <-180) gyroEstimateAngle.yaw   += 360;

        // Make a unit vector of force
        float F = sqrtf(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
        float F_inverse = 1 / F;
        float x = accel.x * F_inverse;
        float y = accel.y * F_inverse;
        float z = accel.z * F_inverse;

        struct Angle3D AccelEstimateAngle = {
            getRoll(x,y,z,gyroEstimateAngle.roll),
            -getPitch(x,y,z,gyroEstimateAngle.pitch),
            gyroEstimateAngle.yaw, /* Accelerometer can not estimate yaw */
        };

        // Conplementary filter
        xAttitude = ComplementaryFilter(&gyroEstimateAngle,&AccelEstimateAngle);
}

#define GYRO_DRDY 0x01
#define ACCEL_DRDY 0x02
void SensorTask(void *arg){
    while(1){
        /* Read Sensor */
        L3G4200D_Recv();
        ADXL345_Recv();
        HMC5883L_Recv();

        /* Process */
        Process();
    }
}

