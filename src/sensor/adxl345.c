#include "sensor/adxl345.h"
#include "sensor/i2c.h"

#include "task.h"
#include "semphr.h"

static xSemaphoreHandle ADXL345_Lock;
struct ADXL345 ADXL345;

void Write_ADXL345(uint8_t Register, uint8_t content){
    uint8_t buf[2];
    buf[0] = Register;
    buf[1] = content;
    while(!I2C_Master_Transmit(ADXL345_START, buf, 2));
}

void READ_ADXL345(uint8_t addr,uint8_t buf[]){
    buf[0] = addr;
    do{
        I2C_Master_Transmit(ADXL345_START, buf, 1);
    }while(!I2C_Master_Receive(ADXL345_START, buf, 1));
};

void ADXL345_Init(){
    /* Init control register */
    kputs("Setting Control Register for ADXL345\r\n");
    Write_ADXL345(POWER_CTL  , 0b00000000);
    Write_ADXL345(DATA_FORMAT, 0b00000000);
    Write_ADXL345(FIFO_CTL   , 0b00000000);
    Write_ADXL345(BW_RATE    , 0b00000000);
    Write_ADXL345(POWER_CTL  , 0b00000000);
    kputs("Control Register for ADXL345 had been set\r\n");
}

void ADXL345_Recv(void *arg){
    while(1){
        xSemaphoreTake( ADXL345_Lock, ( portTickType ) portMAX_DELAY );
        /* TODO: check data is available or not */
        /* Data not available yet */
        if(1){
            xSemaphoreGive( ADXL345_Lock );
            continue;
        }
        /* TODO: read data */
        xSemaphoreGive( ADXL345_Lock );
    }
}

void ADXL345_Process(void *arg){
    xSemaphoreTake( ADXL345_Lock, ( portTickType ) portMAX_DELAY );
    /* TODO: process data */
    xSemaphoreGive( ADXL345_Lock );
}
