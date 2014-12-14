#include "motor.h"

static TIM_HandleTypeDef TIM2_Handle;
static TIM_OC_InitTypeDef TIM2_OCConfig;
static TIM_ClockConfigTypeDef TIM2_ClockSourceConfig;

static xTaskHandle motorTaskHandle;

bool Init_Motor(){

/* Definition for TIMx's NVIC */

    TIM2_Handle.Instance = TIM2;

    TIM2_Handle.Init.Prescaler = 0;
    TIM2_Handle.Init.Period = 1500000;
    TIM2_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TIM2_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    if(HAL_TIM_Base_Init(&TIM2_Handle) != HAL_OK){
        return false;
    }

    TIM2_ClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if(HAL_TIM_ConfigClockSource( &TIM2_Handle, &TIM2_ClockSourceConfig)
            != HAL_OK){
        return false;
    }

    if(HAL_TIM_PWM_Init(&TIM2_Handle) != HAL_OK){
        return false;
    }

    /* Produce High Voltage when CCR > CNT*/
    TIM2_OCConfig.OCMode = TIM_OCMODE_PWM1;
    TIM2_OCConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
    TIM2_OCConfig.OCFastMode = TIM_OCFAST_DISABLE;

    TIM2_OCConfig.Pulse = 90000;

    if(HAL_TIM_PWM_ConfigChannel(&TIM2_Handle, &TIM2_OCConfig, TIM_CHANNEL_1)
            != HAL_OK){
        return false;
    }

    if(HAL_TIM_PWM_Start(&TIM2_Handle, TIM_CHANNEL_1)
            != HAL_OK){
        return false;
    }

    kputs("Initialized TIM\r\n");

    TIM2_Handle.Instance->CR1 |= 0x1;
    xTaskCreate(MotorTask,
            (signed portCHAR *)"Motor control proccess",
            512,
            NULL,
            tskIDLE_PRIORITY + 2,
            &motorTaskHandle);
    return true;
}

#define GPIO_PIN_CHANNEL1 GPIO_PIN_0 // PA0
#define GPIO_PIN_CHANNEL2 GPIO_PIN_1 // PA1
#define GPIO_PIN_CHANNEL3 GPIO_PIN_2 // PA2
#define GPIO_PIN_CHANNEL4 GPIO_PIN_3 // PA3
;
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim){
    GPIO_InitTypeDef   GPIO_InitStruct;

    /* TIMx Peripheral clock enable */
    __TIM2_CLK_ENABLE();
    /* Enable GPIO channels Clock */
    __GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;

    GPIO_InitStruct.Pin = GPIO_PIN_CHANNEL1 | GPIO_PIN_CHANNEL2
                        | GPIO_PIN_CHANNEL3 | GPIO_PIN_CHANNEL4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void MotorTask(void *arg){

    for(int i = 0 ; i < 100000000;++i);
    while(1){
        TIM2_OCConfig.Pulse = 135000;
        printBinary_uint32(TIM2_OCConfig.Pulse);
        kputs("\r\n");
//        if(TIM2_OCConfig.Pulse > 180000){
//            TIM2_OCConfig.Pulse = 90000;
//        }
        if(HAL_TIM_PWM_ConfigChannel(&TIM2_Handle, &TIM2_OCConfig, TIM_CHANNEL_1)
                != HAL_OK){
        }
        for(int i = 0 ; i < 1000000;++i);
    }
}
