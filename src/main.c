#include "main.h"

#include "stm32f4xx_hal.h"

#include "rcc.h"
#include "uart.h"
#include "gy801.h"

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
    kputs("USART test\r\n");

    __GPIOG_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_G;
    GPIO_G.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_G.Pull = GPIO_PULLUP;
    GPIO_G.Pin = GPIO_PIN_13;
    GPIO_G.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOG, &GPIO_G);
    HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);
    kputs("Toggle LED\r\n");

    uint8_t buf = 0x00;
    Init_GY801();
    kputs("Initialized GY-801\r\n");
    buf = 0x4F;
    I2C_Master_Trasmit(L3G42000_START, CTRL_REG1, &buf, 1);
    buf = 0x80;
    I2C_Master_Trasmit(L3G42000_START, CTRL_REG4, &buf, 1);
    kputs("Set control reg\r\n");

    while(1){
        if( 'a' != kgetc() ) continue;
        kputs("Start data fetch!\r\n");

        // Magic number:0b11010011
        I2C_Master_Receive(L3G42000_START, WHO_AM_I, &buf, 1);
        printBinary(buf);

        I2C_Master_Receive(L3G42000_START, OUT_X_L, &buf, 1);
        printBinary(buf);
        I2C_Master_Receive(L3G42000_START, OUT_X_H, &buf, 1);
        printBinary(buf);

        I2C_Master_Receive(L3G42000_START, OUT_Y_L, &buf, 1);
        I2C_Master_Receive(L3G42000_START, OUT_Y_H, &buf, 1);
        I2C_Master_Receive(L3G42000_START, OUT_Z_L, &buf, 1);
        I2C_Master_Receive(L3G42000_START, OUT_Z_H, &buf, 1);

        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);
    }
}
