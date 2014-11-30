#include "main.h"

#include "stm32f4xx_hal.h"

#include "rcc.h"
#include "uart.h"
#include "gy801.h"

void READ_I2C(uint16_t start,uint8_t addr,uint8_t buf[]){
        buf[0] = addr;
        I2C_Master_Trasmit(start,buf,1);
        I2C_Master_Receive(start,buf,1);
};

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

    uint8_t buf[2];
    Init_GY801();
    kputs("Initialized I2C\r\n");
    buf[0] = CTRL_REG1;
    buf[1] = 0x4F;
    I2C_Master_Trasmit(L3G42000_START, buf, 2);
    buf[0] = CTRL_REG4;
    buf[0] = 0x80;
    I2C_Master_Trasmit(L3G42000_START, buf, 2);
    kputs("Control Register for L3G400D had been set\r\n");
    while(1){
        kputs("Start data fetch!\r\n");

        kputs("Who_AM_I: ");
        READ_I2C(L3G42000_START, WHO_AM_I, buf);
        printBinary(buf[0]);
        kputs("\r\n");

        kputs("X: \r\n");
        READ_I2C(L3G42000_START, OUT_X_H, buf);
        printBinary(buf[0]);
        READ_I2C(L3G42000_START, OUT_X_L, buf + 1);
        printBinary(buf[1]);
        kputs("\r\n");

        kputs("Y: \r\n");
        READ_I2C(L3G42000_START, OUT_Y_H, buf);
        printBinary(buf[0]);
        READ_I2C(L3G42000_START, OUT_Y_L, buf + 1);
        printBinary(buf[1]);
        kputs("\r\n");

        kputs("Z: \r\n");
        READ_I2C(L3G42000_START, OUT_Z_H, buf);
        printBinary(buf[0]);
        READ_I2C(L3G42000_START, OUT_Z_L, buf + 1);
        printBinary(buf[1]);
        kputs("\r\n");

        for(int i = 0 ; i < 1000000; ++i);
        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);
    }
}

