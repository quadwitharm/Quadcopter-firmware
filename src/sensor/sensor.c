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

bool InitSensorPeriph(){
    kputs("I2C: Initialing ...\r\n");
    if(!I2C_Init()) return false;
    kputs("I2C: Initialized\r\n");

    /* Initialize sensors on GY-801 */
    L3G4200D_Init();
    ADXL345_Init();
    HMC5883L_Init();

    Init_SensorDetective();
    Init_Attitude();

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
        L3G4200D.int16.X,
        L3G4200D.int16.Y,
        L3G4200D.int16.Z,
    };
    struct Angle3D compass = {
        HMC5883L.int16.X,
        HMC5883L.int16.Y,
        HMC5883L.int16.Z,
    };

    acceleration = accel;
    lastAngularSpeed = angularSpeed;

    // TODO: Kalman Filter

    AHRSupdate(angularSpeed, accel, compass);

    // Conplementary filter
    xAttitude = getAngle(angularSpeed, accel, compass);
    // Make a unit vector of force

    sendSensorInfo();
}

#define GYRO_DRDY 0x01
#define ACCEL_DRDY 0x02
void SensorTask(void *arg){
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
    while(1){
        /* Read Sensor */
        L3G4200D_Recv();
        ADXL345_Recv();
        HMC5883L_Recv();

        /* Process */
        Process();

        kputs("SensorTask\r\n");
        // Timer interrupt will wake up this task
        vTaskSuspend(recvTaskHandle);
    }
}


TIM_HandleTypeDef TIM4_Handle;

void Init_SensorDetective()
{
    HAL_StatusTypeDef status = HAL_OK;

    /* TIMx Peripheral clock enable */
    __TIM4_CLK_ENABLE();

    TIM4_Handle.Instance = TIM4;
    TIM4_Handle.Init.Prescaler = 0;
    TIM4_Handle.Init.Period = 1500000;
    TIM4_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TIM4_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;

    status = HAL_TIM_Base_Init(&TIM4_Handle);
    if( HAL_OK != status ){ return; }

    HAL_NVIC_SetPriority(TIM4_IRQn, 13, 0);

    HAL_TIM_Base_Start_IT(&TIM4_Handle);
    kputs("Initialized TIM4\r\n");
}

void TIM4_IRQHandler(void){
    kputs("TIM4\r\n");
    /* TIM Update event */
    if(__HAL_TIM_GET_FLAG(&TIM4_Handle, TIM_FLAG_UPDATE) != RESET) {
        if(__HAL_TIM_GET_ITSTATUS(&TIM4_Handle, TIM_IT_UPDATE) !=RESET) {
            __HAL_TIM_CLEAR_IT(&TIM4_Handle, TIM_IT_UPDATE);
            if(pdTRUE == xTaskResumeFromISR(recvTaskHandle)){
                taskYIELD();
            }
        }
    }
}

#define Kp 2.0f
#define Ki 0.005f
#define halfT 0.5f                // half the sample period

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
    const float Time = 0.005f;
    const float gyroScale = 0.0074f;
    
    L3G4200D_Recv();
    ADXL345_Recv();
    HMC5883L_Recv();

    // Read from shared memory
    struct Angle3D accel = {
        ADXL345.int16.X,
        ADXL345.int16.Y,
        ADXL345.int16.Z,
    };
    struct Angle3D angularSpeed = {
        L3G4200D.int16.X,
        L3G4200D.int16.Y,
        L3G4200D.int16.Z,
    };

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
    gyroAngle = gyroEstimateAngle;

    // Make a unit vector of force
    float F = sqrtf(accel.roll * accel.roll + accel.pitch * accel.pitch + 
            accel.yaw * accel.yaw);
    float F_inverse = 1 / F;
    float x = accel.roll * F_inverse;
    float y = accel.pitch * F_inverse;
    float z = accel.yaw * F_inverse;

    struct Angle3D AccelEstimateAngle = {
        getRoll(x,y,z,gyroEstimateAngle.roll),
        -getPitch(x,y,z,gyroEstimateAngle.pitch),
        gyroEstimateAngle.yaw, /* Accelerometer can not estimate yaw */
    };
    accelAngle = AccelEstimateAngle;

    struct Angle3D EularAngle = 
        ComplementaryFilter(&gyroEstimateAngle,&AccelEstimateAngle);
    Init_quaternion(EularAngle);
}

void AHRSupdate(struct Angle3D groy, struct Angle3D accel, struct Angle3D compass){
    float norm;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;

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

    norm = sqrt(groy.roll * groy.roll + groy.pitch * groy.pitch + 
            groy.yaw * groy.yaw);
    groy.roll /= norm;
    groy.pitch /= norm;
    groy.yaw /= norm;

    norm = sqrt(compass.roll * compass.roll + compass.pitch * compass.pitch + 
            compass.yaw * compass.yaw);
    compass.roll /= norm;
    compass.pitch /= norm;
    compass.yaw /= norm;

    hx = 2*compass.roll*(0.5 - q2q2 - q3q3) + 2*compass.pitch*(q1q2 - q0q3) +
         2*compass.yaw*(q1q3 + q0q2);
    hy = 2*compass.roll*(q1q2 - q0q3) + 2*compass.pitch*(0.5 - q1q1 - q3q3) +
         2*compass.yaw*(q3q3 + q0q1);
    hz = 2*compass.roll*(q1q3 - q0q2) + 2*compass.pitch*(q2q3 - q0q1) +
         2*compass.yaw*(0.5 - q1q1 + q2q2);

    bx = sqrt((hx*hx) + (hy*hy));
    bz = hz;

    vx = 2*(q1q3 - q0q2);
    vy = 2*(q0q1 + q2q3);
    vz = q0q0 - q1q1 -q2q2 + q3q3;

    wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
    wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
    wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);

    ex = (accel.pitch*vz - accel.yaw*vy) + (compass.pitch*wz - compass.yaw*wy);
    ey = (accel.yaw*vx - accel.roll*vz) + (compass.yaw*wx - compass.roll*wz);
    ez = (accel.roll*vy - accel.pitch*vx) + (compass.roll*wy - compass.pitch*wx);

    exInt = exInt + ex*Ki;
    eyInt = eyInt + ey*Ki;
    ezInt = ezInt + ez*Ki;

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

struct Angle3D getAngle(){
    return (struct Angle3D){
        atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1-2*q2*q2+1),
        asin(-2*q1*q3 + 2*q0*q2),
        atan2(2*q1*q2 + 2*q0*q3, -2*q2*q2-2*q3*q3+1),
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
