#include "spi.h"

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

SPI_HandleTypeDef Spi1Handle , Spi2Handle;


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

	Spi1Handle = (SPI_HandleTypeDef) {
		.Instance = SPI1,
		.Init = {
			.Mode = SPI_MODE_MASTER,
			.Direction = SPI_DIRECTION_2LINES,
			.DataSize = SPI_DATASIZE_8BIT,
			.CLKPolarity = SPI_POLARITY_LOW,
			.CLKPhase = SPI_PHASE_1EDGE,
			.NSS = SPI_NSS_SOFT,
			//APB2 84mhz /2 ,scale 16 =2.625mhz
			.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16,
			.FirstBit = SPI_FIRSTBIT_MSB,
			.TIMode = SPI_TIMODE_DISABLED,
			.CRCCalculation = SPI_CRCCALCULATION_DISABLED
		}
	};

	Spi2Handle = (SPI_HandleTypeDef) {
		.Instance = SPI2,
		.Init = {
			.Mode = SPI_MODE_MASTER,
			.Direction = SPI_DIRECTION_2LINES,
			.DataSize = SPI_DATASIZE_8BIT,
			.CLKPolarity = SPI_POLARITY_LOW,
			.CLKPhase = SPI_PHASE_1EDGE,
			.NSS = SPI_NSS_SOFT,
			//APB1 42mhz /2 ,scale 8 =2.625mhz
			.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8,
			.FirstBit = SPI_FIRSTBIT_MSB,
			.TIMode = SPI_TIMODE_DISABLED,
			.CRCCalculation = SPI_CRCCALCULATION_DISABLED
		}
	};


	return (HAL_SPI_Init(&Spi1Handle) == HAL_OK) &&
		(HAL_SPI_Init(&Spi2Handle) == HAL_OK);
}

void SPI_send(int nspi,uint8_t* data, uint16_t length){
    /*
     * Use polling mode if FreeRTOS hasn't startup yet,
     * otherwise else interrupt mode
     */
}

void SPI_recv(int nspi,uint8_t* data, uint16_t length){
    /*
     * Use polling mode if FreeRTOS hasn't startup yet,
     * otherwise else interrupt mode
     */
}

void SPI_send_IT(int nspi,uint8_t* data, uint16_t length){

}

void SPI_recv_IT(int nspi,uint8_t* buffer, uint16_t length){

}

void SPI_send_POLL(int nspi,uint8_t* data, uint16_t length){

}

void SPI_recv_POLL(int nspi,uint8_t* buffer, uint16_t length){

}

void StartSPIRXInterrupt(){

}


void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi){	
	GPIO_InitTypeDef GPIO_InitStruct = { 
		.Mode=GPIO_MODE_AF_PP, 
		.Pull=GPIO_PULLUP,
		.Speed=GPIO_SPEED_FAST
	};

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
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* SPI MISO GPIO pin configuration  */
        GPIO_InitStruct.Pin = GPIO_PIN_4;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* SPI MOSI GPIO pin configuration  */
        GPIO_InitStruct.Pin = GPIO_PIN_5;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /*##-3- Configure the NVIC for SPI #########################################*/
        /* NVIC for SPI */
        HAL_NVIC_SetPriority(SPI1_IRQn, 12, 0);
        HAL_NVIC_EnableIRQ(SPI1_IRQn);
    }else if(hspi->Instance ==SPI2){
        __SPI2_CLK_ENABLE();
        __GPIOB_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_13;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_14;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_15;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		HAL_NVIC_SetPriority(SPI2_IRQn, 12, 0);
		HAL_NVIC_EnableIRQ(SPI2_IRQn);

    }
}


