#include "gy801.h"
#include "main.h"

I2C_HandleTypeDef I2c_Handle;

void YieldGY801Task();
void ResumeGY801Task();

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
    return true;
}

bool I2C_Master_Trasmit(uint16_t deviceAddr, uint8_t buf[], uint16_t size){
    /* Start the transmission process */
    while(HAL_I2C_Master_Transmit_IT(&I2c_Handle, deviceAddr, buf, size) != HAL_OK) {
        /* When Acknowledge failure ocucurs (Slave don't acknowledge it's address) Master restarts communication */
        if (HAL_I2C_GetError(&I2c_Handle) != HAL_I2C_ERROR_AF) {
            return false;
        }
    }
    while (HAL_I2C_GetState(&I2c_Handle) != HAL_I2C_STATE_READY) {
        YieldGY801Task();
    }
    return true;
}

bool I2C_Master_Receive(uint16_t deviceAddr, uint8_t buf[], uint16_t size){
    // Polling
    /* Put I2C peripheral in reception process */
    while(HAL_I2C_Master_Receive(&I2c_Handle, deviceAddr, buf, size, 100000) != HAL_OK) {
        if (HAL_I2C_GetError(&I2c_Handle) != HAL_I2C_ERROR_AF) {
            // Timeout
            return false;
        }
    }
    return true;
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

void Write_L3G4200D(uint8_t Register, uint8_t content){
    uint8_t buf[2];
    buf[0] = Register;
    buf[1] = content;
    I2C_Master_Trasmit(L3G42000_START, buf, 2);
}

void READ_I2C(uint16_t start,uint8_t addr,uint8_t buf[]){
        buf[0] = addr;
        I2C_Master_Trasmit(start,buf,1);
        I2C_Master_Receive(start,buf,1);
};

void GY801_Task(void){
    Write_L3G4200D(CTRL_REG1, 0x4F);
    Write_L3G4200D(CTRL_REG4, 0x80);
    kputs("Control Register for L3G400D had been set\r\n");
    uint8_t buf[2];
    while(1){
        kputs("Start data fetch!\r\n");

        kputs("Who_AM_I: ");
        READ_I2C(L3G42000_START, WHO_AM_I, buf);
        printBinary(buf[0]);
        kputs("\r\n");

        kputs("X: \r\n");
        READ_I2C(L3G42000_START, OUT_X_H, buf);
        printBinary(buf[0]);
        READ_I2C(L3G42000_START, OUT_X_L, buf + 1);
        printBinary(buf[1]);
        kputs("\r\n");

        kputs("Y: \r\n");
        READ_I2C(L3G42000_START, OUT_Y_H, buf);
        printBinary(buf[0]);
        READ_I2C(L3G42000_START, OUT_Y_L, buf + 1);
        printBinary(buf[1]);
        kputs("\r\n");

        kputs("Z: \r\n");
        READ_I2C(L3G42000_START, OUT_Z_H, buf);
        printBinary(buf[0]);
        READ_I2C(L3G42000_START, OUT_Z_L, buf + 1);
        printBinary(buf[1]);
        kputs("\r\n");

        for(int i = 0 ; i < 1000000; ++i);
        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);
    }

}
