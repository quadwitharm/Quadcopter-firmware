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
#include "shell/send.h"
#include "shell/nrf24l01.h"

void NRFTestTask(void *args){
    uint8_t buf[32];
    NRF24L01_Transmit(1, (uint8_t *)"It works!\r\n", 11);
    UART_send((uint8_t *)"Sent. try to receive\r\n", 22);
    NRF24L01_Receive(0, buf, 11);
    UART_send(buf, 11);
    while(1);
}

int main(void){
    HAL_Init();
    SystemClock_Config();

    if(UART_init(USART1,9600) != HAL_OK){
        /* Something wrong, Freeze */
        while(1);
    }
    Init_IO();
//    UART_send((uint8_t []){(uint8_t)0xFF}, 1);
//    kputs("USART test\r\n");
//    if(!InitSensorPeriph() || !InitSensorTask()){
//        kputs("Initialize sensor task failed!\r\n");
//    }
//    if(!Init_Motor()){
//        kputs("Initialize motor failed!\r\n");
//    }
//    if(!Init_Controller()){
//        kputs("Initialize controller task failed!\r\n");
//    }
//    if(!InitShell()){
//        kputs("Initialize shell task failed!\r\n");
//    }
//    if(!Init_SendInfoTask()){
//        kputs("Initialize information task failed!\r\n");
//    }

//    kputs("Initialization complete, Start schedular!\r\n");

    UART_send((uint8_t *)"Test \r\n", 7);
    if(!NRF24L01_Init()){
        UART_send((uint8_t *)"Init failed \r\n", 14);
    }
    xTaskCreate(NRFTestTask,
                "NRF IRQ Handling Task",
                256,
                NULL,
                tskIDLE_PRIORITY + 6,
                NULL
                );

//    schestart = 1;
    vTaskStartScheduler();

    /* Should not reach here */
    while(1);
}
