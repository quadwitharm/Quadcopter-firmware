#include "motor.h"

#include "task.h"
#include "semphr.h"

#include "stm32f4xx_hal_tim.h"

#define GPIO_PIN_CHANNEL1 GPIO_PIN_0  // PA0
#define GPIO_PIN_CHANNEL2 GPIO_PIN_3  // PB3
#define GPIO_PIN_CHANNEL3 GPIO_PIN_10 // PB10
#define GPIO_PIN_CHANNEL4 GPIO_PIN_3  // PA3
#define PERIOD   1500000
#define MAX_PULSE 180000
#define MIN_PULSE  90000
#define PULSE_RANGE (MAX_PULSE - MIN_PULSE)

#ifdef USE_OPENDRAIN
    #define TIM_GPIO_OTYPE GPIO_MODE_AF_OD;
    #define PULSE(Speed) ((Speed) * PULSE_RANGE + MIN_PULSE)
#else
    #define TIM_GPIO_OTYPE GPIO_MODE_AF_PP;
    #define PULSE(Speed) ((Speed) * PULSE_RANGE + MIN_PULSE)
#endif


TIM_HandleTypeDef TIM2_Handle;
static TIM_OC_InitTypeDef OCConfig;
static TIM_ClockConfigTypeDef ClockSourceConfig;

bool Init_Motor(){
    HAL_StatusTypeDef status = HAL_OK;

    TIM2_Handle.Instance = TIM2;
    TIM2_Handle.Init.Prescaler = 0;
    TIM2_Handle.Init.Period = PERIOD;
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
    OCConfig.Pulse = PULSE(0);
    status = HAL_TIM_PWM_ConfigChannel(&TIM2_Handle, &OCConfig, TIM_CHANNEL_1);
    status = HAL_TIM_PWM_ConfigChannel(&TIM2_Handle, &OCConfig, TIM_CHANNEL_2);
    status = HAL_TIM_PWM_ConfigChannel(&TIM2_Handle, &OCConfig, TIM_CHANNEL_3);
    status = HAL_TIM_PWM_ConfigChannel(&TIM2_Handle, &OCConfig, TIM_CHANNEL_4);

    status = HAL_TIM_PWM_Start(&TIM2_Handle, TIM_CHANNEL_1);
    status = HAL_TIM_PWM_Start(&TIM2_Handle, TIM_CHANNEL_2);
    status = HAL_TIM_PWM_Start(&TIM2_Handle, TIM_CHANNEL_3);
    status = HAL_TIM_PWM_Start(&TIM2_Handle, TIM_CHANNEL_4);


    kputs("Wait ESC to be initialized\r\n");
//    for(int i = 0 ; i < 100000000;++i);

    HAL_NVIC_SetPriority(TIM2_IRQn, 10, 0);
    HAL_TIM_Base_Start_IT(&TIM2_Handle);

    kputs("Initialized TIM\r\n");
    return true;
}

void UpdateMotorSpeed(float speed[4]){
    OCConfig.Pulse = PULSE(speed[0]);
    HAL_TIM_PWM_ConfigChannel(&TIM2_Handle, &OCConfig, TIM_CHANNEL_1);
    OCConfig.Pulse = PULSE(speed[1]);
    HAL_TIM_PWM_ConfigChannel(&TIM2_Handle, &OCConfig, TIM_CHANNEL_2);
    OCConfig.Pulse = PULSE(speed[2]);
    HAL_TIM_PWM_ConfigChannel(&TIM2_Handle, &OCConfig, TIM_CHANNEL_3);
    OCConfig.Pulse = PULSE(speed[3]);
    HAL_TIM_PWM_ConfigChannel(&TIM2_Handle, &OCConfig, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&TIM2_Handle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&TIM2_Handle, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&TIM2_Handle, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&TIM2_Handle, TIM_CHANNEL_4);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim){
    GPIO_InitTypeDef   GPIO_InitStruct;

    /* TIMx Peripheral clock enable */
    __TIM2_CLK_ENABLE();
    /* Enable GPIO channels Clock */
    __GPIOA_CLK_ENABLE();
    __GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Mode = TIM_GPIO_OTYPE;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;

    GPIO_InitStruct.Pin = GPIO_PIN_CHANNEL1 | GPIO_PIN_CHANNEL4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* MEMS Sensor conflict PA1, PA2 on STM32F429I-Discovery */
    GPIO_InitStruct.Pin = GPIO_PIN_CHANNEL2 | GPIO_PIN_CHANNEL3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
