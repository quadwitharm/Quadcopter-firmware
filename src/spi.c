#include "spi.h"
#include "clib.h"
#include "semphr.h"
#include "task.h"

#define TIMEOUT 10000

// channels : [0] -> tx(spi1) ,[1] -> rx(spi2)
SPI_HandleTypeDef SpiHandle[2];
volatile xSemaphoreHandle _spi_sem[2];
volatile xSemaphoreHandle _spi_mux[2];

bool SPI_init(void)
{
    // Init according to NRF24L01 specs

    SpiHandle[SPI_TX] = (SPI_HandleTypeDef){
        .Instance = SPI1,
        .Init = {.Mode = SPI_MODE_MASTER,
                 .Direction = SPI_DIRECTION_2LINES,
                 .DataSize = SPI_DATASIZE_8BIT,
                 .CLKPolarity = SPI_POLARITY_LOW,
                 .CLKPhase = SPI_PHASE_1EDGE,
                 .NSS = SPI_NSS_SOFT,
                 // APB2 84mhz /2 ,scale 16 =2.625mhz
                 .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128,
                 .FirstBit = SPI_FIRSTBIT_MSB,
                 .TIMode = SPI_TIMODE_DISABLED,
                 .CRCCalculation = SPI_CRCCALCULATION_DISABLED}};

    SpiHandle[SPI_RX] = (SPI_HandleTypeDef){
        .Instance = SPI3,
        .Init = {.Mode = SPI_MODE_MASTER,
                 .Direction = SPI_DIRECTION_2LINES,
                 .DataSize = SPI_DATASIZE_8BIT,
                 .CLKPolarity = SPI_POLARITY_LOW,
                 .CLKPhase = SPI_PHASE_1EDGE,
                 .NSS = SPI_NSS_SOFT,
                 // APB1 42mhz /2 ,scale 8 =2.625mhz
                 .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64,
                 .FirstBit = SPI_FIRSTBIT_MSB,
                 .TIMode = SPI_TIMODE_DISABLED,
                 .CRCCalculation = SPI_CRCCALCULATION_DISABLED}};

    _spi_sem[SPI_TX] = xSemaphoreCreateBinary();
    _spi_sem[SPI_RX] = xSemaphoreCreateBinary();
    _spi_mux[SPI_TX] = xSemaphoreCreateMutex();
    _spi_mux[SPI_RX] = xSemaphoreCreateMutex();

    // initial take state for blocking
    xSemaphoreTake(_spi_sem[SPI_TX], (TickType_t)0);
    xSemaphoreTake(_spi_sem[SPI_RX], (TickType_t)0);

    return (HAL_SPI_Init(&SpiHandle[SPI_TX]) == HAL_OK) &&
           (HAL_SPI_Init(&SpiHandle[SPI_RX]) == HAL_OK);
}

void SPI_sendRecv(int nspi, uint8_t *txData, uint8_t *rxData, uint16_t length)
{
    if (!schestart) {
        if (HAL_SPI_TransmitReceive(&SpiHandle[nspi], txData, rxData, length,
                                    TIMEOUT) != HAL_OK) {
            while (1)
                ;  // Error occured
        }
    } else {
        SPI_sendRecv_IT(nspi, txData, rxData, length);
    }
}

void SPI_sendRecv_IT(int nspi,
                     uint8_t *txData,
                     uint8_t *rxData,
                     uint16_t length)
{
    while (!xSemaphoreTake(_spi_mux[nspi], portMAX_DELAY))
        ;

    HAL_SPI_TransmitReceive_IT(&SpiHandle[nspi], txData, rxData, length);
    while (!xSemaphoreTake(_spi_sem[nspi], portMAX_DELAY))
        ;

    xSemaphoreGive(_spi_mux[nspi]);
}

// handlers to call HAL
void SPI1_IRQHandler() { HAL_SPI_IRQHandler(&SpiHandle[SPI_TX]); }

void SPI3_IRQHandler() { HAL_SPI_IRQHandler(&SpiHandle[SPI_RX]); }

// called after HAL_SPI_TransmitReceive_IT complete
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    static signed portBASE_TYPE xHigherPriorityTaskWoken;

    if (hspi->Instance == SPI1) {
        xSemaphoreGiveFromISR(_spi_sem[SPI_TX], &xHigherPriorityTaskWoken);
    } else if (hspi->Instance == SPI3) {
        xSemaphoreGiveFromISR(_spi_sem[SPI_RX], &xHigherPriorityTaskWoken);
    }
    if (xHigherPriorityTaskWoken) {
        taskYIELD();
    }
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef GPIO_InitStruct = {
        .Mode = GPIO_MODE_AF_PP, .Pull = GPIO_PULLDOWN, .Speed = GPIO_SPEED_FAST};

    /* SPI1:
     *  + SCK:  PA5
     *  + MISO: PA6
     *  + MOSI: PA7
     *
     * SPI3:
     *  + SCK:  PC1
     *  + MISO: PC1
     *  + MOSI: PC1
     */

    if (hspi->Instance == SPI1) {
        /*##-1- Enable peripherals and GPIO Clocks ###########################*/
        /* Enable SPI clock */
        __SPI1_CLK_ENABLE();
        /* Enable GPIO TX/RX clock */
        __GPIOA_CLK_ENABLE();

        /*##-2- Configure peripheral GPIO ####################################*/
        /* SPI SCK,MISO,MOSI GPIO pin configuration  */
        GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /*##-3- Configure the NVIC for SPI ###################################*/
        /* NVIC for SPI */
        HAL_NVIC_SetPriority(SPI1_IRQn, 12, 0);
        HAL_NVIC_EnableIRQ(SPI1_IRQn);

    } else if (hspi->Instance == SPI3) {
        __SPI3_CLK_ENABLE();
        __GPIOC_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
        GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(SPI3_IRQn, 12, 0);
        HAL_NVIC_EnableIRQ(SPI3_IRQn);
    }
}
