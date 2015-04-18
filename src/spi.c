#include "spi.h"
#include "clib.h"

#include "semphr.h"


#define SPIx                             SPI4
#define SPIx_CLK_ENABLE()                __SPI4_CLK_ENABLE()
#define SPIx_SCK_GPIO_CLK_ENABLE()       __GPIOE_CLK_ENABLE()
#define SPIx_MISO_GPIO_CLK_ENABLE()      __GPIOE_CLK_ENABLE() 
#define SPIx_MOSI_GPIO_CLK_ENABLE()      __GPIOE_CLK_ENABLE() 

#define SPIx_FORCE_RESET()               __SPI4_FORCE_RESET()
#define SPIx_RELEASE_RESET()             __SPI4_RELEASE_RESET()

/* Definition for SPIx Pins */
#define SPIx_SCK_PIN                     GPIO_PIN_2
#define SPIx_SCK_GPIO_PORT               GPIOE
#define SPIx_SCK_AF                      GPIO_AF5_SPI4
#define SPIx_MISO_PIN                    GPIO_PIN_5
#define SPIx_MISO_GPIO_PORT              GPIOE
#define SPIx_MISO_AF                     GPIO_AF5_SPI4
#define SPIx_MOSI_PIN                    GPIO_PIN_6
#define SPIx_MOSI_GPIO_PORT              GPIOE
#define SPIx_MOSI_AF                     GPIO_AF5_SPI4

/* Definition for SPIx's NVIC */
#define SPIx_IRQn                        SPI4_IRQn
#define SPIx_IRQHandler                  SPI4_IRQHandler



bool SPI_init(void){
/*    
     *  + SCK:  PB3
     *  + MISO: PB4
     *  + MOSI: PB5
     *
     * SPI2:
     *  + SCK:  PB13
     *  + MISO: PB14
     *  + MOSI: PB15
     *
     *  Configure the interrupts but do not enable.
     * */
}

void SPI_send(uint8_t* data, uint16_t length){
    /*
     * Use polling mode if FreeRTOS hasn't startup yet,
     * otherwise else interrupt mode
     */
}

void SPI_recv(uint8_t* data, uint16_t length){
    /*
     * Use polling mode if FreeRTOS hasn't startup yet,
     * otherwise else interrupt mode
     */
}

void SPI_send_IT(uint8_t* data, uint16_t length){

}

void SPI_recv_IT(uint8_t* buffer, uint16_t length){

}

void SPI_send_POLL(uint8_t* data, uint16_t length){

}

void SPI_recv_POLL(uint8_t* buffer, uint16_t length){

}

void StartSPIRXInterrupt(){

}


void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi){	
    GPIO_InitTypeDef  GPIO_InitStruct{ .Mode=GPIO_MODE_AF_PP, .Pull=GPIO_PULLUP,Speed=GPIO_SPEED_FAST};

    /* SPI1:
     *  + SCK:  PB3
     *  + MISO: PB4
     *  + MOSI: PB5
     *
     * SPI2:
     *  + SCK:  PB13
     *  + MISO: PB14
     *  + MOSI: PB15
    */ 
  
    if(hspi->Instance ==SPI1){
        /*##-1- Enable peripherals and GPIO Clocks #################################*/
        /* Enable SPI clock */
        __SPI1_CLK_ENABLE();
        /* Enable GPIO TX/RX clock */
        __GPIOB_CLK_ENABLE();

        /*##-2- Configure peripheral GPIO ##########################################*/  
        /* SPI SCK GPIO pin configuration  */
        GPIO_InitStruct.Pin = GPIO_PIN_3;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIO_PORT_B, &GPIO_InitStruct);

        /* SPI MISO GPIO pin configuration  */
        GPIO_InitStruct.Pin = GPIO_PIN_4;
        HAL_GPIO_Init(GPIO_PORT_B, &GPIO_InitStruct);

        /* SPI MOSI GPIO pin configuration  */
        GPIO_InitStruct.Pin = GPIO_PIN_5;
        HAL_GPIO_Init(GPIO_PORT_B, &GPIO_InitStruct);

        /*##-3- Configure the NVIC for SPI #########################################*/
        /* NVIC for SPI */
        HAL_NVIC_SetPriority(SPIx_IRQn, 0, 1);
        HAL_NVIC_EnableIRQ(SPIx_IRQn);
    }else if(hspi->Instance ==SPI2){
        __SPI2_CLK_ENABLE();
        __GPIOB_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_13;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init(GPIO_PORT_B, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_14;
        HAL_GPIO_Init(GPIO_PORT_B, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_15;
        HAL_GPIO_Init(GPIO_PORT_B, &GPIO_InitStruct);
    }
}


