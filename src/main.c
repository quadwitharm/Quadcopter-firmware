#include "clib.h"

#include "stm32f4xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#include "rcc.h"
#include "uart.h"
#include "sensor/sensor.h"
#include "motor.h"
#include "controller/control_task.h"

int main(void){
    HAL_Init();
    SystemClock_Config();

    if(UART_init(USART1,115200) != HAL_OK){
        // Something wrong
    }
    kputs("USART test\r\n");

    __GPIOG_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_G;
    GPIO_G.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_G.Pull = GPIO_NOPULL;
    GPIO_G.Pin = GPIO_PIN_9;
    GPIO_G.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOG, &GPIO_G);
    HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_9);

    if(!InitSensorPeriph() || !InitSensorTask()){
        kputs("Initialze sensor task failed!");
    }
    if(!Init_Motor()){
        kputs("Initialze motor task failed!");
    }
    Init_Controller();

    vTaskStartScheduler();

    while(1);
}
