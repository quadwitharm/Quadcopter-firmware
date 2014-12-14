#include "motor.h"

static TIM_HandleTypeDef TIM2_Handle;
static TIM_OC_InitTypeDef OCConfig;
static TIM_ClockConfigTypeDef ClockSourceConfig;

static xTaskHandle motorTaskHandle;

xSemaphoreHandle MotorSem;
struct MotorSpeed MotorSpeed = {
    .motor1 = 0.5f,
    .motor2 = 0.5f,
    .motor3 = 0.5f,
    .motor4 = 0.5f,
};

bool Init_Motor(){
    HAL_StatusTypeDef status = HAL_OK;

    TIM2_Handle.Instance = TIM2;
    TIM2_Handle.Init.Prescaler = 0;
    TIM2_Handle.Init.Period = 1500000;
    TIM2_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TIM2_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;

    status = HAL_TIM_Base_Init(&TIM2_Handle);
    if( HAL_OK != status ){ return false; }

    ClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    status = HAL_TIM_ConfigClockSource( &TIM2_Handle, &ClockSourceConfig);
    if( HAL_OK != status ){ return false; }

    status = HAL_TIM_PWM_Init(&TIM2_Handle);
    if( HAL_OK != status ){ return false; }

    OCConfig.OCMode = TIM_OCMODE_PWM1;
    OCConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
    OCConfig.OCFastMode = TIM_OCFAST_DISABLE;

    /* Initial Value to start ESC */
    OCConfig.Pulse = 90000;
    status = HAL_TIM_PWM_ConfigChannel(&TIM2_Handle, &OCConfig, TIM_CHANNEL_1);
    status = HAL_TIM_PWM_ConfigChannel(&TIM2_Handle, &OCConfig, TIM_CHANNEL_2);
    status = HAL_TIM_PWM_ConfigChannel(&TIM2_Handle, &OCConfig, TIM_CHANNEL_3);
    status = HAL_TIM_PWM_ConfigChannel(&TIM2_Handle, &OCConfig, TIM_CHANNEL_4);
    if( HAL_OK != status ){ return false; }

    status = HAL_TIM_PWM_Start(&TIM2_Handle, TIM_CHANNEL_1);
    status = HAL_TIM_PWM_Start(&TIM2_Handle, TIM_CHANNEL_2);
    status = HAL_TIM_PWM_Start(&TIM2_Handle, TIM_CHANNEL_3);
    status = HAL_TIM_PWM_Start(&TIM2_Handle, TIM_CHANNEL_4);
    if( HAL_OK != status ){ return false; }

    /* Task to update PWM duty cycle */
    xTaskCreate(MotorTask,
            (signed portCHAR *)"Motor control proccess",
            512,
            NULL,
            tskIDLE_PRIORITY + 2,
            &motorTaskHandle);
    MotorSem = xSemaphoreCreateMutex();

    kputs("Initialized TIM\r\n");
    return true;
}

#define GPIO_PIN_CHANNEL1 GPIO_PIN_0  // PA0
#define GPIO_PIN_CHANNEL2 GPIO_PIN_3  // PB3
#define GPIO_PIN_CHANNEL3 GPIO_PIN_10 // PB10
#define GPIO_PIN_CHANNEL4 GPIO_PIN_3  // PA3
;
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim){
    GPIO_InitTypeDef   GPIO_InitStruct;

    /* TIMx Peripheral clock enable */
    __TIM2_CLK_ENABLE();
    /* Enable GPIO channels Clock */
    __GPIOA_CLK_ENABLE();
    __GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;

    GPIO_InitStruct.Pin = GPIO_PIN_CHANNEL1 | GPIO_PIN_CHANNEL4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* MEMS Sensor conflict PA1, PA2 on STM32F429I-Discovery */
    GPIO_InitStruct.Pin = GPIO_PIN_CHANNEL2 | GPIO_PIN_CHANNEL3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void MotorTask(void *arg){
    /* Wait motor to be ready */
    for(int i = 0 ; i < 100000000;++i);
    while(1){
        /* For ESC: 90000 ~ 180000 */

        xSemaphoreTake(MotorSem, portMAX_DELAY);

        OCConfig.Pulse = MotorSpeed.motor1 * 90000 + 90000;
        HAL_TIM_PWM_ConfigChannel(&TIM2_Handle, &OCConfig, TIM_CHANNEL_1);
        OCConfig.Pulse = MotorSpeed.motor1 * 90000 + 90000;
        HAL_TIM_PWM_ConfigChannel(&TIM2_Handle, &OCConfig, TIM_CHANNEL_2);
        OCConfig.Pulse = MotorSpeed.motor1 * 90000 + 90000;
        HAL_TIM_PWM_ConfigChannel(&TIM2_Handle, &OCConfig, TIM_CHANNEL_3);
        OCConfig.Pulse = MotorSpeed.motor1 * 90000 + 90000;
        HAL_TIM_PWM_ConfigChannel(&TIM2_Handle, &OCConfig, TIM_CHANNEL_4);

        HAL_TIM_PWM_Start(&TIM2_Handle, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&TIM2_Handle, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&TIM2_Handle, TIM_CHANNEL_3);
        HAL_TIM_PWM_Start(&TIM2_Handle, TIM_CHANNEL_4);

        xSemaphoreGive(MotorSem);

        for(int i = 0 ; i < 1000000;++i);
    }
}
