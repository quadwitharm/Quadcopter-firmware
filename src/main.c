#include "main.h"

#include "stm32f4xx_hal.h"

#include "rcc.h"
#include "uart.h"
void printBinary(uint8_t c){
    for(uint8_t i = 1u << 7; i != 0;i >>= 1){
        kputc( (c & i) ? '1' : '0');
    }
    kputc(' ');
}

int main(void){
    HAL_Init();
    SystemClock_Config();

    if(UART_init(USART1,115200) != HAL_OK){
        // Something wrong
    }

    __GPIOG_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_G;
    GPIO_G.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_G.Pull = GPIO_PULLUP;
    GPIO_G.Pin = GPIO_PIN_13 | GPIO_PIN_14;
    GPIO_G.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOG, &GPIO_G);

    HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);

    kputs("Hello World!\r\n");

    while(1){
        if( 'a' != kgetc() ) continue;
        kputs("Blink!\r\n");
        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);
        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_14);
        for(int i = 0;i < 1000000;++i);
    }
}
