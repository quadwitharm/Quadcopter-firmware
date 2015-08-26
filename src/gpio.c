#include "gpio.h"
#include "stm32f4xx_hal_gpio.h"

#include "shell/nrf24l01.h"

bool GPIO_Init(){
    /*
     * NRF-1 pin list
     * PB0 : IRQ
     * PC4 : CSN
     * PC5 : CE
     *
     * NRF-1 pin list
     * PD9 : IRQ
     * PD8 : CSN
     * PD10: CE
     */

    GPIO_InitTypeDef GPIO_InitStruct;

    __GPIOB_CLK_ENABLE();
    __GPIOC_CLK_ENABLE();
    __GPIOD_CLK_ENABLE();

    /* Simple Output pin */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_10;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* IRQ pin */
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* NVIC settings */
    HAL_NVIC_SetPriority(EXTI0_IRQn, 12, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 12, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    return true;
}

void EXTI0_IRQHandler(void) {
    NRF24L01_IRQ(0);
}

void EXTI9_5_IRQHandler(void) {
    NRF24L01_IRQ(1);
}

