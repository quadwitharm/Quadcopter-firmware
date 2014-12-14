#include "sensor/l3g4200d.h"
#include "main.h"

#include "task.h"
#include "semphr.h"

// FreeRTOS Task
xTaskHandle recvTaskHandle;
xTaskHandle processTaskHandle;
static xSemaphoreHandle attitudeLock;

static I2C_HandleTypeDef I2c_Handle;

struct L3G4200D L3G4200D;

/**
 * @brief  Initialize sensor read and processing task and associating hardware.
 * @param  None
 * @retval Success or not
 */
bool Init_GY801(){

    I2c_Handle.Instance = I2Cx;
    I2c_Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    I2c_Handle.Init.ClockSpeed = 400000;
    I2c_Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
    I2c_Handle.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
    I2c_Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
    I2c_Handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
    if(HAL_I2C_Init(&I2c_Handle) != HAL_OK){
        return false;
    }

    kputs("Initialized I2C\r\n");

    /* FreeRTOS Tasks and lock */
    attitudeLock = xSemaphoreCreateMutex();
    xTaskCreate(GY801_RecvTask,
            (signed portCHAR *)"IMU data fetch",
            512,
            NULL,
            tskIDLE_PRIORITY + 2,
            &recvTaskHandle);
    xTaskCreate(GY801_ProcessTask,
            (signed portCHAR *)"Attitude data proccess",
            512,
            NULL,
            tskIDLE_PRIORITY + 2,
            &processTaskHandle);

    /* This task will resume after GY801_RecvTask is Ready to fetch data */
    vTaskSuspend(processTaskHandle);

    return true;
}

/**
 * @brief  Task Control for Interrupt I/O
 * @param  None
 * @retval None
 */
static inline void ResumeRecvTask(){
}

bool I2C_Master_Transmit(uint16_t deviceAddr, uint8_t buf[], uint16_t size){
    /* Start the transmission process */
    while( HAL_OK
         != HAL_I2C_Master_Transmit_IT(&I2c_Handle, deviceAddr, buf, size) ) {
        /* Slave doesn't acknowledge it's address,
         * Master restarts communication */
        if (HAL_I2C_GetError(&I2c_Handle) != HAL_I2C_ERROR_AF) {
            return false;
        }
    }
    while (HAL_I2C_GetState(&I2c_Handle) != HAL_I2C_STATE_READY) {
        taskYIELD();
    }
    return true;
}

bool I2C_Master_Receive(uint16_t deviceAddr, uint8_t buf[], uint16_t size){
    // Polling
    /* Put I2C peripheral in reception process */
    while( HAL_OK
            != HAL_I2C_Master_Receive_IT(&I2c_Handle, deviceAddr, buf, size) ) {
        if (HAL_I2C_GetError(&I2c_Handle) != HAL_I2C_ERROR_AF) {
            return false;
        }
    }
    while (HAL_I2C_GetState(&I2c_Handle) != HAL_I2C_STATE_READY) {
        taskYIELD();
    }
    return true;
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *I2cHandle){
}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *I2cHandle){
}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle){
    kputs("I2C error!\r\n");
}

void I2Cx_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&I2c_Handle);
}

void I2Cx_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&I2c_Handle);
}

/**
 * @brief  Low-level peripheral initialize
 * @param  hi2c: HAL Handle
 * @retval None
 */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c) {
    I2Cx_SCL_GPIO_CLK_ENABLE();
    I2Cx_SDA_GPIO_CLK_ENABLE();
    I2Cx_CLK_ENABLE();

    GPIO_InitTypeDef  GPIO_InitStruct;
    GPIO_InitStruct.Pin       = I2Cx_SCL_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = I2Cx_SCL_AF;
    HAL_GPIO_Init(I2Cx_SCL_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = I2Cx_SDA_PIN;
    GPIO_InitStruct.Alternate = I2Cx_SDA_AF;
    HAL_GPIO_Init(I2Cx_SDA_GPIO_PORT, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(I2Cx_ER_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(I2Cx_ER_IRQn);
    HAL_NVIC_SetPriority(I2Cx_EV_IRQn, 0, 2);
    HAL_NVIC_EnableIRQ(I2Cx_EV_IRQn);
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c) {
    I2Cx_FORCE_RESET();
    I2Cx_RELEASE_RESET();

    HAL_GPIO_DeInit(I2Cx_SCL_GPIO_PORT, I2Cx_SCL_PIN);
    HAL_GPIO_DeInit(I2Cx_SDA_GPIO_PORT, I2Cx_SDA_PIN);

    HAL_NVIC_DisableIRQ(I2Cx_ER_IRQn);
    HAL_NVIC_DisableIRQ(I2Cx_EV_IRQn);
}

void Write_L3G4200D(uint8_t Register, uint8_t content){
    uint8_t buf[2];
    buf[0] = Register;
    buf[1] = content;
    while(!I2C_Master_Transmit(L3G42000_START, buf, 2));
}

void READ_L3G4200D(uint8_t addr,uint8_t buf[]){
    buf[0] = addr;
    do{
        I2C_Master_Transmit(L3G42000_START, buf, 1);
    }while(!I2C_Master_Receive(L3G42000_START, buf, 1));
};

void GY801_RecvTask(void *arg){
    /* Initialize sensors on GY-801 */
    kputs("Setting Control Register\r\n");
    Write_L3G4200D(CTRL_REG1, 0b01001111);
    Write_L3G4200D(CTRL_REG4, 0b10000000);
    Write_L3G4200D(CTRL_REG5, 0b11000000);
    Write_L3G4200D(FIFO_CTRL_REG, 0b01010000);

    kputs("Control Register for L3G4200D had been set\r\n");
    vTaskResume(processTaskHandle);

    while(1){
        xSemaphoreTake( attitudeLock, ( portTickType ) portMAX_DELAY );
        uint8_t FIFO_STATUS;
        READ_L3G4200D(FIFO_SRC_REG, &FIFO_STATUS);
        if(FIFO_STATUS & 0b00100000){
            xSemaphoreGive( attitudeLock );
            continue;
        }
//        READ_L3G4200D(STATUS_REG, &FIFO_STATUS);
//        if(FIFO_STATUS & 0b00001000){

        READ_L3G4200D(OUT_X_H, &L3G4200D.XH);
        READ_L3G4200D(OUT_X_L, &L3G4200D.XL);

        READ_L3G4200D(OUT_Y_H, &L3G4200D.YH);
        READ_L3G4200D(OUT_Y_L, &L3G4200D.YL);

        READ_L3G4200D(OUT_Z_H, &L3G4200D.ZH);
        READ_L3G4200D(OUT_Z_L, &L3G4200D.ZL);

        xSemaphoreGive( attitudeLock );
        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);
    }

}
void GY801_ProcessTask(void *arg){
    kputs("Process Task Start\r\n");
    while(1){
        xSemaphoreTake( attitudeLock, ( portTickType ) portMAX_DELAY );
        kputs("X: \r\n");
        printBinary_uint8(L3G4200D.XH);
        kputs(" ");
        printBinary_uint8(L3G4200D.XL);
        kputs("\r\n");

        kputs("Y: \r\n");
        printBinary_uint8(L3G4200D.YH);
        kputs(" ");
        printBinary_uint8(L3G4200D.YL);
        kputs("\r\n");

        kputs("Z: \r\n");
        printBinary_uint8(L3G4200D.ZH);
        kputs(" ");
        printBinary_uint8(L3G4200D.ZL);
        kputs("\r\n");

        xSemaphoreGive( attitudeLock );
        for(int i = 0 ; i < 1000000; ++i);
    }
}
