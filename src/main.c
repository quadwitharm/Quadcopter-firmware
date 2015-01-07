#include "clib.h"

#include "stm32f4xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#include "rcc.h"
#include "uart.h"
#include "motor.h"

#include "sensor/sensor.h"
#include "controller/control_task.h"
#include "shell/shell.h"

int main(void){
    HAL_Init();
    SystemClock_Config();

    if(UART_init(USART1,115200) != HAL_OK){
        /* Something wrong, Freeze */
        while(1);
    }
    kputs("USART test\r\n");

    if(!InitSensorPeriph() || !InitSensorTask()){
        kputs("Initialize sensor task failed!\r\n");
    }
    if(!Init_Motor()){
        kputs("Initialize motor failed!\r\n");
    }
    if(!Init_Controller()){
        kputs("Initialize controller task failed!\r\n");
    }
    if(!InitShell()){
        kputs("Initialize shell task failed!\r\n");
    }

    kputs("Initialization complete, Start schedular!\r\n");
    vTaskStartScheduler();

    /* Should not reach here */
    while(1);
}
