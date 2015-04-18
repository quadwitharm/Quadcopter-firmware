#include "uart.h"
#include "clib.h"
#include "semphr.h"
#include "task.h"
#include "stm32f4xx_hal_dma.h"

#define USARTx_TX_DMA_STREAM DMA2_Stream7
#define USARTx_TX_DMA_CHANNEL DMA_CHANNEL_4
#define USARTx_RX_DMA_STREAM DMA2_Stream5
#define USARTx_RX_DMA_CHANNEL DMA_CHANNEL_4

#define DMAx_CLK_ENABLE()                __DMA2_CLK_ENABLE()

#define USARTx_DMA_TX_IRQn                DMA2_Stream7_IRQn
#define USARTx_DMA_RX_IRQn                DMA2_Stream5_IRQn
#define USARTx_DMA_TX_IRQHandler          DMA2_Stream7_IRQHandler
#define USARTx_DMA_RX_IRQHandler          DMA2_Stream5_IRQHandler

UART_HandleTypeDef UartHandle;
static DMA_HandleTypeDef hdma_tx;
static DMA_HandleTypeDef hdma_rx;
volatile xSemaphoreHandle _tx_wait_sem = NULL;
volatile xSemaphoreHandle _rx_wait_sem = NULL;
xQueueHandle rxQueue;

HAL_StatusTypeDef UART_init(USART_TypeDef *uart, uint32_t BaudRate){
    UartHandle = (UART_HandleTypeDef) {
        .Instance = uart,
        .Init = { .BaudRate = BaudRate,
            .WordLength = UART_WORDLENGTH_8B,
            .StopBits = UART_STOPBITS_1,
            .Parity = UART_PARITY_NONE,
            .HwFlowCtl = UART_HWCONTROL_NONE,
            .Mode = UART_MODE_TX_RX,
            .OverSampling = UART_OVERSAMPLING_16,
        }
    };
    _rx_wait_sem = xSemaphoreCreateBinary();
    _tx_wait_sem = xSemaphoreCreateBinary();
    rxQueue = xQueueCreate(64, 1);
    return HAL_UART_Init(&UartHandle);
}

HAL_StatusTypeDef UART_send(uint8_t* data, uint16_t length){ 
    //blocking call
    HAL_StatusTypeDef status = 0;
    if(!schestart){
        for(;length>0;length--){
            status = HAL_UART_Transmit(&UartHandle, data++, 1, 10000);
        }
    }else{
        UART_send_IT(data,length);
    }
    return status;
}

HAL_StatusTypeDef UART_recv(uint8_t* buffer, uint16_t length){
    return HAL_UART_Receive(&UartHandle, (uint8_t *)buffer, length, 10000);
}

void UART_send_IT(uint8_t* data, uint16_t length){
    HAL_UART_Transmit_IT(&UartHandle, data, length);
    while (!xSemaphoreTake(_tx_wait_sem, portMAX_DELAY));
}

void UART_recv_IT(uint8_t* buffer, uint16_t length){
    for(;length>0;buffer++,length--){
        xQueueReceive(rxQueue, buffer, portMAX_DELAY);
    }
}

// RX Interrupt can only start after FreeRTOS schedular starts, because the IRQ
// Handler uses FreeRTOS Queue API
void StartUartRXInterrupt(){
    __HAL_UART_ENABLE_IT(&UartHandle, UART_IT_RXNE);
    __HAL_UART_ENABLE_IT(&UartHandle, UART_IT_PE);
    __HAL_UART_ENABLE_IT(&UartHandle, UART_IT_ERR);
}

/**
 * @brief  Tx Transfer completed callback
 * @param  UartHandle: UART handle
 * @retval None
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle){
    static signed portBASE_TYPE xHigherPriorityTaskWoken;
    /* Set transmission flag: trasfer complete*/
    xSemaphoreGiveFromISR(_tx_wait_sem, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        taskYIELD();
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart){
    GPIO_InitTypeDef GPIO_InitStruct = {.Mode = GPIO_MODE_AF_PP, .Pull = GPIO_NOPULL, .Speed = GPIO_SPEED_FAST};
    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /*##-2- Configure peripheral GPIO ##########################################*/
    /*##-3- Configure the NVIC for UART ########################################*/
    if(huart->Instance == USART1){// tx/rx: PA9/PA10, PB6/PB7
        __USART1_CLK_ENABLE();
        __GPIOA_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(USART1_IRQn, 12, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }else if(huart->Instance == USART2){// tx/rx: PA2/PA3, PD5/PD6
        __USART2_CLK_ENABLE();
        __GPIOA_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(USART2_IRQn, 0, 1);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    }else if(huart->Instance == USART3){// tx/rx: PB10/PB11, PC10/PC11, PD8/PD9
        __USART3_CLK_ENABLE();
        __GPIOB_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(USART3_IRQn, 12, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    }else if(huart->Instance == UART4){// tx/rx: PA0/PA1, PC10/PC11
        __UART4_CLK_ENABLE();
        __GPIOA_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
        GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(UART4_IRQn, 0, 1);
        HAL_NVIC_EnableIRQ(UART4_IRQn);
    }else if(huart->Instance == UART5){// tx/rx: PC12/PD2
        __UART5_CLK_ENABLE();
        __GPIOC_CLK_ENABLE();
        __GPIOD_CLK_ENABLE();
        GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
        GPIO_InitStruct.Pin = GPIO_PIN_12;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
        GPIO_InitStruct.Pin = GPIO_PIN_2;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(UART5_IRQn, 12, 0);
        HAL_NVIC_EnableIRQ(UART5_IRQn);
    }else if(huart->Instance == USART6){// tx/rx: PC6/PC7, PG14/PG9
        __USART6_CLK_ENABLE();
        __GPIOC_CLK_ENABLE();
        GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
        GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(USART6_IRQn, 0, 1);
        HAL_NVIC_EnableIRQ(USART6_IRQn);
    }else if(huart->Instance == UART7){// tx/rx: PE8/PE7, PF7/PF6
        __UART7_CLK_ENABLE();
        __GPIOE_CLK_ENABLE();
        GPIO_InitStruct.Alternate = GPIO_AF8_UART7;
        GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_7;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(UART7_IRQn, 0, 1);
        HAL_NVIC_EnableIRQ(UART7_IRQn);
    }else if(huart->Instance == UART8){// tx/rx: PE1/PE0
        __UART8_CLK_ENABLE();
        __GPIOE_CLK_ENABLE();
        GPIO_InitStruct.Alternate = GPIO_AF8_UART8;
        GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_0;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(UART8_IRQn, 0, 1);
        HAL_NVIC_EnableIRQ(UART8_IRQn);
    }else return;

    DMAx_CLK_ENABLE();

    hdma_tx.Instance                 = USARTx_TX_DMA_STREAM;
    hdma_tx.Init.Channel             = USARTx_TX_DMA_CHANNEL;
    hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_tx.Init.Mode                = DMA_NORMAL;
    hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
    hdma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_tx.Init.MemBurst            = DMA_MBURST_INC4;
    hdma_tx.Init.PeriphBurst         = DMA_PBURST_INC4;
    HAL_DMA_Init(&hdma_tx);
    __HAL_LINKDMA(huart, hdmatx, hdma_tx);

    hdma_rx.Instance                 = USARTx_RX_DMA_STREAM;
    hdma_rx.Init.Channel             = USARTx_RX_DMA_CHANNEL;
    hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_rx.Init.Mode                = DMA_NORMAL;
    hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdma_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_rx.Init.MemBurst            = DMA_MBURST_INC4;
    hdma_rx.Init.PeriphBurst         = DMA_PBURST_INC4;
    HAL_DMA_Init(&hdma_rx);
    __HAL_LINKDMA(huart, hdmarx, hdma_rx);


    HAL_NVIC_SetPriority(USARTx_DMA_TX_IRQn, 11, 0);
    HAL_NVIC_EnableIRQ(USARTx_DMA_TX_IRQn);

    /* NVIC configuration for DMA transfer complete interrupt (USART1_RX) */
    HAL_NVIC_SetPriority(USARTx_DMA_RX_IRQn, 12, 0);
    HAL_NVIC_EnableIRQ(USARTx_DMA_RX_IRQn);
}

void USARTx_DMA_RX_IRQHandler(void)
{
  HAL_DMA_IRQHandler(UartHandle.hdmarx);
}

void USARTx_DMA_TX_IRQHandler(void)
{
  HAL_DMA_IRQHandler(UartHandle.hdmatx);
}

// Uses HAL Library
HAL_StatusTypeDef UART_Transmit_IT(UART_HandleTypeDef *huart);

static HAL_StatusTypeDef UART_Receive_IT(UART_HandleTypeDef *huart) {
    BaseType_t isHigherPriorityTaskWoken = 0;
    // To reduce interrupt latency, disable wordlength and parity configuration
    // check by default
#ifdef CONFIG_WORDLENGTH_AND_PARITY
    if(huart->Init.WordLength == UART_WORDLENGTH_9B) {
        if(huart->Init.Parity == UART_PARITY_NONE) {
            // Only properly work when queue item size is 2 byte or more
            uint16_t r = (uint16_t)(huart->Instance->DR & (uint16_t)0x01FF);
            xQueueSendToBackFromISR(rxQueue, &r, &isHigherPriorityTaskWoken);
        } else {
            uint8_t r = (uint8_t)(huart->Instance->DR & (uint8_t)0xFF);
            xQueueSendToBackFromISR(rxQueue, &r, &isHigherPriorityTaskWoken);
        }
    } else {
        if(huart->Init.Parity == UART_PARITY_NONE) {
#endif
            uint8_t r = (uint8_t)(huart->Instance->DR & (uint8_t)0xFF);
            xQueueSendToBackFromISR(rxQueue, &r, &isHigherPriorityTaskWoken);
#ifdef CONFIG_WORDLENGTH_AND_PARITY
        } else {
            uint8_t r = (uint8_t)(huart->Instance->DR & (uint8_t)0x7F);
            xQueueSendToBackFromISR(rxQueue, &r, &isHigherPriorityTaskWoken);
        }
    }
#endif
    if(isHigherPriorityTaskWoken) taskYIELD();
    return HAL_OK;
}

void UART_IRQHandler(UART_HandleTypeDef *huart) {
    uint32_t tmp1 = 0, tmp2 = 0;

    tmp1 = __HAL_UART_GET_FLAG(huart, UART_FLAG_PE);
    tmp2 = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_PE);
    /* UART parity error interrupt occurred */
    if((tmp1 != RESET) && (tmp2 != RESET)) {
        __HAL_UART_CLEAR_PEFLAG(huart);
        huart->ErrorCode |= HAL_UART_ERROR_PE;
    }

    tmp1 = __HAL_UART_GET_FLAG(huart, UART_FLAG_FE);
    tmp2 = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_ERR);
    /* UART frame error interrupt occurred */
    if((tmp1 != RESET) && (tmp2 != RESET)) {
        __HAL_UART_CLEAR_FEFLAG(huart);
        huart->State = HAL_UART_STATE_READY;
        // Still process data, check data integrity in another way
        UART_Receive_IT(huart);
    }

    tmp1 = __HAL_UART_GET_FLAG(huart, UART_FLAG_NE);
    tmp2 = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_ERR);
    /* UART noise error interrupt occurred */
    if((tmp1 != RESET) && (tmp2 != RESET)) {
        __HAL_UART_CLEAR_NEFLAG(huart);
        huart->ErrorCode |= HAL_UART_ERROR_NE;
    }

    tmp1 = __HAL_UART_GET_FLAG(huart, UART_FLAG_ORE);
    tmp2 = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_ERR);
    /* UART Over-Run interrupt occurred */
    if((tmp1 != RESET) && (tmp2 != RESET)) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        huart->ErrorCode |= HAL_UART_ERROR_ORE;
    }

    tmp1 = __HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE);
    tmp2 = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_RXNE);
    /* UART in mode Receiver */
    if((tmp1 != RESET) && (tmp2 != RESET)) {
        UART_Receive_IT(huart);
    }

    tmp1 = __HAL_UART_GET_FLAG(huart, UART_FLAG_TXE);
    tmp2 = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_TXE);
    /* UART in mode Transmitter */
    if((tmp1 != RESET) && (tmp2 != RESET)) {
        UART_Transmit_IT(huart);
    }

    // Ignore frame error
    if(huart->ErrorCode != (HAL_UART_ERROR_NONE & ~HAL_UART_ERROR_FE)) {
        /* Set the UART state ready to be able to start again the process */
        huart->State = HAL_UART_STATE_READY;
        HAL_UART_ErrorCallback(huart);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle){
    // TODO: Handle UART error?
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart){
    /*##-1- Reset peripherals ##################################################*/
    /*##-2- Disable peripherals and GPIO Clocks #################################*/
    /*##-3- Disable the NVIC for UART ##########################################*/
    if(huart->Instance == USART1){// tx/rx: PA9/PA10, PB6/PB7
        __USART1_FORCE_RESET();
        __USART1_RELEASE_RESET();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);
        HAL_NVIC_DisableIRQ(USART1_IRQn);
    }else if(huart->Instance == USART2){// tx/rx: PA2/PA3, PD5/PD6
        __USART2_FORCE_RESET();
        __USART2_RELEASE_RESET();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);
        HAL_NVIC_DisableIRQ(USART2_IRQn);
    }else if(huart->Instance == USART3){// tx/rx: PB10/PB11, PC10/PC11, PD8/PD9
        __USART3_FORCE_RESET();
        __USART3_RELEASE_RESET();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10 | GPIO_PIN_11);
        HAL_NVIC_DisableIRQ(USART3_IRQn);
    }else if(huart->Instance == UART4){// tx/rx: PA0/PA1, PC10/PC11
        __UART4_FORCE_RESET();
        __UART4_RELEASE_RESET();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0 | GPIO_PIN_1);
        HAL_NVIC_DisableIRQ(UART4_IRQn);
    }else if(huart->Instance == UART5){// tx/rx: PC12/PD2
        __UART5_FORCE_RESET();
        __UART5_RELEASE_RESET();
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);
        HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);
        HAL_NVIC_DisableIRQ(UART5_IRQn);
    }else if(huart->Instance == USART6){// tx/rx: PC6/PC7, PG14/PG9
        __USART6_FORCE_RESET();
        __USART6_RELEASE_RESET();
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6 | GPIO_PIN_7);
        HAL_NVIC_DisableIRQ(USART6_IRQn);
    }else if(huart->Instance == UART7){// tx/rx: PE8/PE7, PF7/PF6
        __UART7_FORCE_RESET();
        __UART7_RELEASE_RESET();
        HAL_GPIO_DeInit(GPIOE, GPIO_PIN_0 | GPIO_PIN_1);
        HAL_NVIC_DisableIRQ(UART7_IRQn);
    }else if(huart->Instance == UART8){// tx/rx: PE1/PE0
        __UART8_FORCE_RESET();
        __UART8_RELEASE_RESET();
        HAL_GPIO_DeInit(GPIOE, GPIO_PIN_0 | GPIO_PIN_1);
        HAL_NVIC_DisableIRQ(UART8_IRQn);
    }else return;
}

/**
 * @brief  These function handles UART interrupt request.
 * @param  None
 * @retval None
 * @Note   This function is redefined in "main.h" and related to DMA stream
 *         used for USART data transmission
 */
void USART1_IRQHandler(void){
    UART_IRQHandler(&UartHandle);
}
void USART2_IRQHandler(void){
    UART_IRQHandler(&UartHandle);
}
void USART3_IRQHandler(void){
    UART_IRQHandler(&UartHandle);
}
void UART4_IRQHandler(void){
    UART_IRQHandler(&UartHandle);
}
void UART5_IRQHandler(void){
    UART_IRQHandler(&UartHandle);
}
void USART6_IRQHandler(void){
    UART_IRQHandler(&UartHandle);
}
void UART7_IRQHandler(void){
    UART_IRQHandler(&UartHandle);
}
void UART8_IRQHandler(void){
    UART_IRQHandler(&UartHandle);
}

