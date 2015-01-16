#include "sensor/sensor.h"
#include "sensor/l3g4200d.h"
#include "sensor/adxl345.h"
#include "sensor/hmc5883l.h"
#include "sensor/i2c.h"
#include "sensor/filter.h"

#include "uart.h"

#include "task.h"
#include "arm_math.h"

/* Arise if all data ready */
xTaskHandle recvTaskHandle;

/* Parameter of Kalman Filter */
struct KalmanParameter Kroll;
struct KalmanParameter Kpitch;

/* Definition of shared resource */
struct Angle3D xAttitude;
struct Angle3D lastAngularSpeed;
static struct Angle3D gyroAngle;
static struct Angle3D accelAngle;

struct Angle3D acceleration;

void SensorTask(void *arg);
void Init_SensorDetective();
void Init_Attitude();
struct Angle3D getAngle();
void AHRSupdate(struct Angle3D, struct Angle3D, struct Angle3D);
void IMUupdate(struct Angle3D, struct Angle3D);
void Init_quaternion();
float invSqrt(float );

bool InitSensorPeriph(){
    kputs("I2C: Initialing ...\r\n");
    if(!I2C_Init()) return false;
    kputs("I2C: Initialized\r\n");

    I2C_PowerInit();

    /* Initialize sensors on GY-801 */
    L3G4200D_Init();
    ADXL345_Init();
    HMC5883L_Init();

    Init_SensorDetective();

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

    // Read from shared memory
    struct Angle3D accel = {
        ADXL345.int16.X,
        ADXL345.int16.Y,
        ADXL345.int16.Z,
    };
    struct Angle3D angularSpeed = {
        L3G4200D.int16.X* 0.000266 * 0.3 + lastAngularSpeed.roll * 0.7,
        L3G4200D.int16.Y* 0.000266 * 0.3 + lastAngularSpeed.pitch * 0.7,
        L3G4200D.int16.Z* 0.000266 * 0.3 + lastAngularSpeed.yaw * 0.7,
    };

    struct Angle3D compass = {
        HMC5883L.int16.X,
        HMC5883L.int16.Y,
        HMC5883L.int16.Z,
    };


    acceleration = accel;
    lastAngularSpeed = angularSpeed;
    //Init_quaternion(acceleration);
    //accel = getAngle();
    //acceleration.yaw = accel.pitch*180/3.141596;
    //acceleration.pitch *= 180.0;
    //lastAngularSpeed = accel;
    //AHRSupdate(angularSpeed, accel, compass);
    
    //IMUupdate(angularSpeed, accel);
#if 1
    float roll = -atan2(accel.pitch, accel.yaw);
    //float pitch = atan2(accel.roll, sqrt(accel.pitch*accel.pitch+accel.yaw*accel.yaw));
    float pitch = asin(accel.roll/256.0);
    float yaw = atan2(compass.roll, compass.pitch);

    struct Angle3D EularAngle = {
        roll * 57.296,
        pitch * 57.296,
        yaw * 57.296,
    };
#endif
    //xAttitude = getAngle();
    xAttitude = EularAngle;

    sendSensorInfo();
}

#define GYRO_DRDY 0x01
#define ACCEL_DRDY 0x02
void SensorTask(void *arg){
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
    Init_Attitude();
    while(1){
        /* Read Sensor */
        L3G4200D_Recv();
        ADXL345_Recv();
        HMC5883L_Recv();

        /* Process */
        Process();

        // Timer interrupt will wake up this task
        vTaskSuspend(recvTaskHandle);
    }
}


TIM_HandleTypeDef TIM5_Handle;

void Init_SensorDetective()
{
    HAL_StatusTypeDef status = HAL_OK;

    /* TIMx Peripheral clock enable */
    __TIM5_CLK_ENABLE();

    TIM5_Handle.Instance = TIM5;
    TIM5_Handle.Init.Prescaler = 0;
    TIM5_Handle.Init.Period = 1500000;
    TIM5_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TIM5_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;

    status = HAL_TIM_Base_Init(&TIM5_Handle);
    if( HAL_OK != status ){ return; }

    HAL_NVIC_SetPriority(TIM5_IRQn, 13, 0);

    HAL_TIM_Base_Start_IT(&TIM5_Handle);
    kputs("Initialized TIM4\r\n");
}

void TIM5_IRQHandler(void){
    /* TIM Update event */
    if(__HAL_TIM_GET_FLAG(&TIM5_Handle, TIM_FLAG_UPDATE) != RESET) {
        if(__HAL_TIM_GET_ITSTATUS(&TIM5_Handle, TIM_IT_UPDATE) !=RESET) {
            __HAL_TIM_CLEAR_IT(&TIM5_Handle, TIM_IT_UPDATE);
            if(pdTRUE == xTaskResumeFromISR(recvTaskHandle)){
                taskYIELD();
            }
        }
    }
}

#define sampleFreq 200.0f
#define twoKpDef (2.0f * 0.5f)
#define twoKiDef (2.0f * 0.0f)

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
float exInt = 0, eyInt = 0, ezInt = 0;
void Init_quaternion(struct Angle3D EulerAngle){

    float halfP = EulerAngle.pitch/2.0f;
    float halfR = EulerAngle.roll/2.0f;
    float halfY = EulerAngle.yaw/2.0f;

    float sinP = sinf(halfP);
    float cosP = cosf(halfP);
    float sinR = sinf(halfR);
    float cosR = cosf(halfR);
    float sinY = sinf(halfY);
    float cosY = cosf(halfY);

    q0 = cosY*cosR*cosP + sinY*sinR*sinP;
    q1 = cosY*cosR*sinP + sinY*sinR*cosP;
    q2 = cosY*sinR*cosP + sinY*cosR*sinP;
    q3 = sinY*cosR*cosP + cosY*sinR*sinP;
}


void Init_Attitude(){
    ADXL345_Recv();
    HMC5883L_Recv();

    // Read from shared memory
    struct Angle3D accel = {
        ADXL345.int16.X,
        ADXL345.int16.Y,
        ADXL345.int16.Z,
    };

    struct Angle3D compass = {
        HMC5883L.int16.X,
        HMC5883L.int16.Y,
        HMC5883L.int16.Z,
    };

    struct Angle3D EularAngle = {
        atan2(-accel.pitch, accel.yaw),
        atan2(accel.roll, sqrt(accel.pitch*accel.pitch+accel.yaw*accel.yaw)),
        atan2(compass.roll, compass.pitch),
    };

    Init_quaternion(EularAngle);
}

#define Kp 2.0f
#define Ki 0.005f
#define halfT 0.5f
void IMUupdate(struct Angle3D groy, struct Angle3D accel)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    norm = sqrt(accel.roll * accel.roll + accel.pitch * accel.pitch + 
            accel.yaw * accel.yaw);
    accel.roll /= norm;
    accel.pitch /= norm;
    accel.yaw /= norm;

    vx = 2*(q1*q3 - q0*q2);
    vy = 2*(q0*q1 + q2*q3);
    vz = q0*q0 - q1*q1 -q2*q2 + q3*q3;

    ex = (accel.pitch*vz - accel.yaw*vy);
    ey = (accel.yaw*vx - accel.roll*vz);
    ez = (accel.roll*vy - accel.pitch*vx);

    exInt += ex*Ki;
    eyInt += ey*Ki;
    ezInt += ez*Ki;

    groy.roll = groy.roll + Kp*ex + exInt;
    groy.pitch = groy.pitch + Kp*ey + eyInt;
    groy.yaw = groy.yaw + Kp*ez + ezInt;

    q0 += (-q1*groy.roll - q2*groy.pitch - q3*groy.yaw)*halfT;
    q1 += (q0*groy.roll + q2*groy.yaw - q3*groy.pitch)*halfT;
    q2 += (q0*groy.pitch - q1*groy.yaw + q3*groy.roll)*halfT;
    q3 += (q0*groy.yaw + q1*groy.pitch - q2*groy.roll)*halfT;

    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;
}

float twoKp = twoKpDef;
float twoKi = twoKiDef;
void AHRSupdate(struct Angle3D groy, struct Angle3D accel, struct Angle3D compass){
    float norm;
    float hx, hy, hz, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez, qa, qb, qc;

    // Auxiliary variables to avoid repeated arithmetic
    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;

    // Normalise magnetometer measurement
    norm = invSqrt(accel.roll * accel.roll + accel.pitch * accel.pitch + 
            accel.yaw * accel.yaw);
    accel.roll *= norm;
    accel.pitch *= norm;
    accel.yaw *= norm;

    //Normalise accelerometer measurement
    norm = sqrt(compass.roll * compass.roll + compass.pitch * compass.pitch + 
            compass.yaw * compass.yaw);
    compass.roll *= norm;
    compass.pitch *= norm;
    compass.yaw *= norm;

    hx = 2.0f * (compass.roll*(0.5 - q2q2 - q3q3) + compass.pitch*(q1q2 - q0q3) +
         compass.yaw*(q1q3 + q0q2));
    hy = 2.0f * (compass.roll*(q1q2 + q0q3) + compass.pitch*(0.5 - q1q1 - q3q3) +
         compass.yaw*(q2q3 - q0q1));
    hz = 2.0f * (compass.roll*(q1q3 - q0q2) + compass.pitch*(q2q3 + q0q1) +
         compass.yaw*(0.5 - q1q1 - q2q2));

    bx = sqrt((hx*hx) + (hy*hy));
    bz = hz;

    halfvx = (q1q3 - q0q2);
    halfvy = (q0q1 + q2q3);
    halfvz = q0q0 - 0.5f + q3q3;

    halfwx = bx*(0.5 - q2q2 - q3q3) + bz*(q1q3 - q0q2);
    halfwy = bx*(q1q2 - q0q3) + bz*(q0q1 + q2q3);
    halfwz = bx*(q0q2 + q1q3) + bz*(0.5 - q1q1 - q2q2);

    halfex = (accel.pitch*halfvz - accel.yaw*halfvy) + 
        (compass.pitch*halfwz - compass.yaw*halfwy);
    halfey = (accel.yaw*halfvx - accel.roll*halfvz) + 
        (compass.yaw*halfwx - compass.roll*halfwz);
    halfez = (accel.roll*halfvy - accel.pitch*halfvx) + 
        (compass.roll*halfwy - compass.pitch*halfwx);

    if(twoKi > 0.0f) {
        exInt += halfex*twoKi * (1.0f/sampleFreq);
        eyInt += halfey*twoKi * (1.0f/sampleFreq);
        ezInt += halfez*twoKi * (1.0f/sampleFreq);

        groy.roll += exInt;
        groy.pitch += eyInt;
        groy.yaw += ezInt;
    }
    else{
        exInt = 0;
        eyInt = 0;
        ezInt = 0;
    }

    groy.roll += twoKp * halfex;
    groy.pitch += twoKp * halfey;
    groy.yaw += twoKp * halfez;

    groy.roll *= (0.5f * (1.0f / sampleFreq));
    groy.pitch *= (0.5f * (1.0f / sampleFreq));
    groy.yaw *= (0.5f * (1.0f / sampleFreq));

    qa = q0;
    qb = q1;
    qc = q2;

    q0 += (-qb*groy.roll - qc*groy.pitch - q3*groy.yaw);
    q1 += (qa*groy.roll + qc*groy.yaw - q3*groy.pitch);
    q2 += (qa*groy.pitch - qb*groy.yaw + q3*groy.roll);
    q3 += (qa*groy.yaw + qb*groy.pitch - qc*groy.roll);

    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;
}

float invSqrt(float x){

    float halfx = 0.5f * x;
    float y = x;

    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));

    return y;
}

struct Angle3D getAngle(){
    return (struct Angle3D){
        atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1-2*q2*q2+1)*57.296,
        asin(-2*q1*q3 + 2*q0*q2)*57.296,
        atan2(2*q1*q2 + 2*q0*q3, -2*q2*q2-2*q3*q3+1)*57.296,
    };
}

void sendSensorInfo(){
    uint8_t head = 0x01;
    UART_send((uint8_t []){head,0x00},2);
    UART_send((uint8_t *)(float []){lastAngularSpeed.roll,
        lastAngularSpeed.pitch,lastAngularSpeed.yaw},12);
    UART_send((uint8_t []){head,0x01},2);
    UART_send((uint8_t *)(float []){acceleration.roll,
        acceleration.pitch,acceleration.yaw},12);
    UART_send((uint8_t []){head,0x02},2);
    UART_send((uint8_t *)(float []){gyroAngle.roll,
        gyroAngle.pitch,gyroAngle.yaw},12);
    UART_send((uint8_t []){head,0x03},2);
    UART_send((uint8_t *)(float []){accelAngle.roll,
        accelAngle.pitch,accelAngle.yaw},12);
    UART_send((uint8_t []){head,0x04},2);
    UART_send((uint8_t *)(float []){xAttitude.roll,
        xAttitude.pitch,xAttitude.yaw},12);
}
