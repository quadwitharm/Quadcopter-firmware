#include "gy801.h"

I2C_HandleTypeDef I2c_Handle;

void Init_GY801(){

    I2c_Handle.Instance = I2Cx;

    I2c_Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    I2c_Handle.Init.ClockSpeed = 400000;
    I2c_Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
    I2c_Handle.Init.DutyCycle = I2C_DUTYCYCLE_2;
    I2c_Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
    I2c_Handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
    I2c_Handle.Init.OwnAddress1 = 0x0;
    I2c_Handle.Init.OwnAddress2 = 0xFE;

    if(HAL_I2C_Init(&I2c_Handle) != HAL_OK){
        // Something wrong
        // TODO
    }
}

void I2C_Master_Trasmit(uint16_t startAddr, uint16_t offset, uint8_t buf[], uint16_t size){
    /* Start the transmission process */
    /* While the I2C in reception process, user can transmit data through
       "aTxBuffer" buffer */
    while(HAL_I2C_Master_Transmit_IT(&I2c_Handle,
                (uint16_t) startAddr + offset,
                (uint8_t*)buf, size)!= HAL_OK)
    {
        /* Error_Handler() function is called when Timout error occurs.
           When Acknowledge failure ocucurs (Slave don't acknowledge it's address)
           Master restarts communication */
        if (HAL_I2C_GetError(&I2c_Handle) != HAL_I2C_ERROR_AF)
        {
            //Timeout
            while(1);
        }
    }
    while (HAL_I2C_GetState(&I2c_Handle) != HAL_I2C_STATE_READY)
    {
        // TODO: Yield,Use Interrupt
    }
}

void I2C_Master_Receive(uint16_t startAddr, uint16_t offset, uint8_t buf[], uint16_t size){
    // TODO: Yield,Use Interrupt
    /* Put I2C peripheral in reception process */
    while(HAL_I2C_Master_Receive_IT(&I2c_Handle,
                (uint16_t) startAddr + offset,
                (uint8_t *)buf, size) != HAL_OK)
    {
        /* Error_Handler() function is called when Timout error occurs.
           When Acknowledge failure ocucurs (Slave don't acknowledge it's address)
           Master restarts communication */
        if (HAL_I2C_GetError(&I2c_Handle) != HAL_I2C_ERROR_AF)
        {
            //Timeout
            while(1);
        }
    }
}
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
{
}

void I2Cx_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&I2c_Handle);
}

void I2Cx_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&I2c_Handle);
}

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
