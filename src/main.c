#include "main.h"

#include "stm32f4xx_hal.h"

#include "rcc.h"
#include "uart.h"
#include "gy801.h"

int main(void){
    HAL_Init();
    SystemClock_Config();

    if(UART_init(USART1,115200) != HAL_OK){
        // Something wrong
    }
    kputs("USART test\r\n");

    __GPIOG_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_G;
    GPIO_G.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_G.Pull = GPIO_PULLUP;
    GPIO_G.Pin = GPIO_PIN_13;
    GPIO_G.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOG, &GPIO_G);
    HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);

    if(Init_GY801())
        GY801_Task();

    while(1);
}

