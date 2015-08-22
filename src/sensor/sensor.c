#include "sensor/sensor.h"
#include "sensor/l3g4200d.h"
#include "sensor/adxl345.h"
#include "sensor/hmc5883l.h"
#include "sensor/bmp180.h"
#include "sensor/i2c.h"
#include "sensor/filter.h"
#include "sensor/MadgwickAHRS.h"

#include "uart.h"
#include "shell/send.h"

#include "task.h"
#include "arm_math.h"

/* Arise if all data ready */
xTaskHandle recvTaskHandle;

/* Parameter of Kalman Filter */
struct KalmanParameter Kroll;
struct KalmanParameter Kpitch;

/* Parameter of height calculation */
float seaLevel_p = 993.8F;

/* Definition of shared resource */
struct Angle3D xAttitude;
struct Angle3D lastAngularSpeed;
float xHeight;

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
    BMP180_Init();

    return true;
}

bool InitSensorTask(){
    Init_SensorDetective();

    I2C_PowerOn();

    /* FreeRTOS Tasks */
    portBASE_TYPE ret;
    ret = xTaskCreate(SensorTask,
            (portCHAR *)"IMU data fetch",
            512,
            NULL,
            tskIDLE_PRIORITY + 4,
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

#define alpha 0.5
void Process(){

    // Read from shared memory
    struct Angle3D accel = {
        ADXL345.int16.X * alpha + (acceleration.roll * (1.0 - alpha)),
        ADXL345.int16.Y * alpha + (acceleration.pitch * (1.0 - alpha)),
        ADXL345.int16.Z * alpha + (acceleration.yaw * (1.0 - alpha)),
    };
    struct Angle3D angularSpeed = {
        L3G4200D.int16.X *0.0175,
        L3G4200D.int16.Y *0.0175,
        L3G4200D.int16.Z *0.0175,
    };

    struct Angle3D compass = {
        HMC5883L.int16.X,
        HMC5883L.int16.Y,
        HMC5883L.int16.Z,
    };

	float atmospheric_p = (float)BMP180.Pressure / 100.0F;
	float temp = (float)BMP180.Temperature;

    acceleration = accel;

    //Init_quaternion(acceleration);
    //accel = getAngle();
    //acceleration.yaw = accel.pitch*180/3.141596;
    //acceleration.pitch *= 180.0;
    //lastAngularSpeed = accel;
    //AHRSupdate(angularSpeed, accel, compass);

    //IMUupdate(angularSpeed, accel);
#if 0
    MadgwickAHRSupdate(
            angularSpeed.roll*0.01745, angularSpeed.pitch*0.01745, angularSpeed.yaw*0.01745,
            accel.roll, accel.pitch, accel.yaw,
            compass.roll, compass.pitch, compass.yaw);
    xAttitude = getEulerAngle();
#endif
#if 0
    float roll = -atan2(accel.pitch, accel.yaw);
    float pitch =  atan2(accel.roll, sqrt(accel.pitch*accel.pitch+accel.yaw*accel.yaw));
    float yaw = atan2(compass.roll, compass.pitch);

    struct Angle3D EularAngle = {
        roll * 57.296,
        pitch * 57.296,
        yaw * 57.296,
    };
    xAttitude = EularAngle;
#endif
#if 1
    // Convert to 360 degree
    struct Angle3D gyroEstimateAngle = {
        xAttitude.roll + angularSpeed.roll * 0.01 ,
        xAttitude.pitch + angularSpeed.pitch * 0.01,
        xAttitude.yaw + angularSpeed.yaw * 0.01,
    };
    if(gyroEstimateAngle.roll > 180) gyroEstimateAngle.roll -= 360;
    if(gyroEstimateAngle.roll <-180) gyroEstimateAngle.roll += 360;
    if(gyroEstimateAngle.pitch > 180) gyroEstimateAngle.pitch -= 360;
    if(gyroEstimateAngle.pitch <-180) gyroEstimateAngle.pitch += 360;
    if(gyroEstimateAngle.yaw > 180) gyroEstimateAngle.yaw -= 360;
    if(gyroEstimateAngle.yaw <-180) gyroEstimateAngle.yaw += 360;
    // Make a unit vector of force
    float F = sqrtf(accel.roll * accel.roll + accel.pitch * accel.pitch + accel.yaw * accel.yaw);
    float F_inverse = 1 / F;
    float x = accel.roll * F_inverse;
    float y = accel.pitch * F_inverse;
    float z = accel.yaw * F_inverse;
    struct Angle3D AccelEstimateAngle = {
        getRoll(x,y,z,gyroEstimateAngle.roll),
        -getPitch(x,y,z,gyroEstimateAngle.pitch),
        atan2(compass.roll, compass.pitch)*57.296,
    };

    // Conplementary filter
    xAttitude = ComplementaryFilter(&gyroEstimateAngle,
			&AccelEstimateAngle);

	lastAngularSpeed = angularSpeed;
	//height from sea level
	xHeight = ((powf((seaLevel_p/atmospheric_p), 0.190223F) - 1.0F)
			* (temp + 273.15F)) / 0.0065F;
#endif
}

#define GYRO_DRDY 0x01
#define ACCEL_DRDY 0x02

void SensorEnable(bool enable){

    if(enable){
        HAL_NVIC_EnableIRQ(TIM4_IRQn);
        InitSensorPeriph();
        //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
        I2C_PowerOn();
    }else{
        HAL_NVIC_DisableIRQ(TIM4_IRQn);
        vTaskSuspend(recvTaskHandle);
        //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
        I2C_PowerOff();
    }
}

void SensorTask(void *arg){
    HAL_NVIC_EnableIRQ(TIM5_IRQn);

    /* wait initial */
    vTaskDelay(100);
    Init_Attitude();

    while(1){
        /* Read Sensor */
        L3G4200D_Recv();
        ADXL345_Recv();
        HMC5883L_Recv();
        BMP180_Recv();

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
    TIM5_Handle.Init.Period = 900000;
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

#define SEND_GYRO 1
#define SEND_ACCEL 1
#define SEND_ATTITUDE 1

void sendSensorInfo(){
    const uint8_t head = 0x01;

    taskENTER_CRITICAL();
    float gyro[3] = {lastAngularSpeed.roll, lastAngularSpeed.pitch, lastAngularSpeed.yaw};
    float accel[3] = {acceleration.roll, acceleration.pitch, acceleration.yaw};
    float att[3] = {xAttitude.roll, xAttitude.pitch, xAttitude.yaw};
    taskEXIT_CRITICAL();

    if(SEND_GYRO)SendCommand_3( head, 0x00, (uint8_t *)gyro, 12);
    if(SEND_ACCEL)SendCommand_3( head, 0x01, (uint8_t *)accel, 12);
    if(SEND_ATTITUDE)SendCommand_3( head, 0x04, (uint8_t *)att, 12);

    kprintf("Pressure: %d, Temperature: %d",BMP180.Pressure, BMP180.Temperature);
}
