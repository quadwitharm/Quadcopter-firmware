#include "sensor/i2c.h"

#include "stm32f4xx_hal_gpio.h"

#include "task.h"


static I2C_HandleTypeDef I2c_Handle;

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

void I2C_PowerOn(){
    HAL_GPIO_WritePin(I2Cx_POWER_GPIO_PORT, I2Cx_POWER_PIN, GPIO_PIN_SET);
}

void I2C_PowerOff(){
    HAL_GPIO_WritePin(I2Cx_POWER_GPIO_PORT, I2Cx_POWER_PIN, GPIO_PIN_RESET);
}

void I2C_PowerInit(){
    __GPIOG_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.Pin = I2Cx_POWER_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;

    HAL_GPIO_Init(I2Cx_POWER_GPIO_PORT, &GPIO_InitStructure);
    HAL_GPIO_TogglePin(I2Cx_POWER_GPIO_PORT, I2Cx_POWER_PIN);
}

bool I2C_Init(){
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
    return true;
}
