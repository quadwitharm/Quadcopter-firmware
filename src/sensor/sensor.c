#include "sensor/sensor.h"
#include "sensor/l3g4200d.h"
#include "sensor/adxl345.h"
#include "sensor/i2c.h"

#include "task.h"
#include "arm_math.h"

/* Arise if all data ready */
EventGroupHandle_t xDataReady;
xTaskHandle recvTaskHandle;
xTaskHandle processTaskHandle;

/* Kalman Filter */
struct KalmanParameter Kroll;
struct KalmanParameter Kpitch;

/* Definition of shared resource */
struct Angle3D xAttitude;
struct Angle3D lastAngularSpeed;

struct Vector3D position;
struct Vector3D velocity;
struct Vector3D acceleration;

void ProcessTask(void *arg);
void SensorTask(void *arg);

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
            (portCHAR *)"IMU data fetch",
            512,
            NULL,
            tskIDLE_PRIORITY + 3,
            &recvTaskHandle);
    if(ret != pdPASS)return false;
    ret = xTaskCreate(ProcessTask,
            (portCHAR *)"Attitude data proccess",
            512,
            NULL,
            tskIDLE_PRIORITY + 4,
            &processTaskHandle);
    if(ret != pdPASS)return false;
    return true;
}

IRQ(){
    if(Pending  ){
        *_avail = true;
        xTaskResumeFromISR(recvTaskHandle);
        Clear Pending();
    }
}

#define GYRO_DRDY 0x01
#define ACCEL_DRDY 0x02
void SensorTask(void *arg){
    uint32_t GyroAccelDRDY = 0;
    while(1){
        if(g_avail){
            L3G4200D_Recv(arg);
            GyroAccelDRDY |= GYRO_DRDY;
            g_avail = false;
        }
        if(a_avail){
            ADXL345_Recv(arg);
            GyroAccelDRDY |= ACCEL_DRDY;
            a_avail = false;
        }
        if(h_avail){
            HMC5883L_Recv();
            h_avail = false;
        }

        if(GyroAccelDRDY == GYRO_DRDY | ACCEL_DRDY){
            Kalman & Complementary();
            GyroAccelDRDY = 0;
        }
        vTaskResume(recvTaskHandle);
    }
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

struct Angle3D ComplementaryFilter(struct Angle3D *gyro,struct Angle3D *fix){
    return (struct Angle3D){
        gyro -> roll  * 0.99 + fix -> roll  * 0.01,
        gyro -> pitch * 0.99 + fix -> pitch * 0.01,
        gyro -> yaw   * 0.99 + fix -> yaw   * 0.01,
    };
}

struct Angle3D KalmanFilter(struct Angle3D *gyro, struct Angle3D *newDataRate, float dt){

    /* Step 1 */
    Kroll.rate = newDataRate -> roll - Kroll.bias;
    Kroll.angle += dt * Kroll.rate;

    Kpitch.rate = newDataRate -> pitch - Kpitch.bias;
    Kpitch.angle += dt * Kpitch.rate;

    /* Step 2: Update estimation error covariance */
    Kroll.P[0][0] += dt * (dt * Kroll.P[1][1] - Kroll.P[0][1] - 
		     Kroll.P[1][0] + Kroll.Q_angle);
    Kroll.P[0][1] -= dt * Kroll.P[1][1];
    Kroll.P[1][0] -= dt * Kroll.P[1][1];
    Kroll.P[1][1] += Kroll.Q_bias * dt;

    Kpitch.P[0][0] += dt * (dt * Kpitch.P[1][1] - Kpitch.P[0][1] - 
		      Kpitch.P[1][0] + Kpitch.Q_angle);
    Kpitch.P[0][1] -= dt * Kpitch.P[1][1];
    Kpitch.P[1][0] -= dt * Kpitch.P[1][1];
    Kpitch.P[1][1] += Kpitch.Q_bias * dt;

    /* Step 3: Calculate angle and bias */
    float yroll = gyro -> roll - Kroll.angle;
    float ypitch = gyro -> pitch - Kroll.angle;

    /* Step 4: Calculate Kalman gain */
    float Sroll = Kroll.P[0][0] + Kpitch.R_measure;
    float Spitch = Kpitch.P[0][0] + Kpitch.R_measure;

    /* Step 5 */
    float kroll[2];
    kroll[0] = Kroll.P[0][0] / Sroll;
    kroll[1] = Kroll.P[1][1] / Sroll;

    float kpitch[2];
    kpitch[0] = Kpitch.P[0][0] / Spitch;
    kpitch[1] = Kpitch.P[1][1] / Spitch;

    /* Step 6:  */
    Kroll.angle += kroll[0] * yroll;
    Kroll.bias += kroll[1] * yroll;

    Kpitch.angle += kpitch[0] * ypitch;
    Kpitch.bias += kpitch[1] * ypitch;

    /* Step 7: Calculate estimation error covariance */
    float P00_roll = Kroll.P[0][0];
    float P01_roll = Kroll.P[0][1];

    Kroll.P[0][0] -= kroll[0] * P00_roll;
    Kroll.P[0][1] -= kroll[0] * P01_roll;
    Kroll.P[1][0] -= kroll[0] * P00_roll;
    Kroll.P[1][1] -= kroll[0] * P01_roll;
    
    float P00_pitch = Kpitch.P[0][0];
    float P01_pitch = Kpitch.P[0][1];

    Kpitch.P[0][0] -= kpitch[0] * P00_pitch;
    Kpitch.P[0][1] -= kpitch[0] * P01_pitch;
    Kpitch.P[1][0] -= kpitch[0] * P00_pitch;
    Kpitch.P[1][1] -= kpitch[0] * P01_pitch;

    return (struct Angle3D){
        Kroll.angle,
        Kpitch.angle,
        gyro -> yaw,
    };
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

void setDataReady(EventBits_t source){
    xEventGroupSetBits(xDataReady, source);
}
