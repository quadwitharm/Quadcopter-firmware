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

//[0] -> spi1 ,[1] -> spi2
SPI_HandleTypeDef SpiHandle[2];
volatile xSemaphoreHandle _spi_sem[2];
xQueueHandle rxQueue[2];


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

	SpiHandle[0] = (SPI_HandleTypeDef) {
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

	SpiHandle[1] = (SPI_HandleTypeDef) {
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

	_spi_sem[0] = xSemaphoreCreateBinary();
	_spi_sem[1] = xSemaphoreCreateBinary();

	return (HAL_SPI_Init(&SpiHandle[0]) == HAL_OK) &&
		(HAL_SPI_Init(&SpiHandle[1]) == HAL_OK);
}

void SPI_sendRecv(int nspi,uint8_t *txData,uint8_t *rxData, uint16_t length){


}

void SPI_sendRecv_IT(int nspi,uint8_t *txData,uint8_t *rxData, uint16_t length){


}

void SPI_sendRecv_POLL(int nspi,uint8_t *txData,uint8_t *rxData, uint16_t length){


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
		/*##-1- Enable peripherals and GPIO Clocks ###########################*/
		/* Enable SPI clock */
		__SPI1_CLK_ENABLE();
		/* Enable GPIO TX/RX clock */
		__GPIOB_CLK_ENABLE();

		/*##-2- Configure peripheral GPIO ####################################*/
		/* SPI SCK,MISO,MOSI GPIO pin configuration  */
		GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/*##-3- Configure the NVIC for SPI ###################################*/
		/* NVIC for SPI */
		HAL_NVIC_SetPriority(SPI1_IRQn, 12, 0);
		HAL_NVIC_EnableIRQ(SPI1_IRQn);

	}else if(hspi->Instance ==SPI2){
		__SPI2_CLK_ENABLE();
		__GPIOB_CLK_ENABLE();

		GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		HAL_NVIC_SetPriority(SPI2_IRQn, 12, 0);
		HAL_NVIC_EnableIRQ(SPI2_IRQn);

	}
}


