#include "sensor/adxl345.h"
#include "sensor/sensor.h"
#include "sensor/i2c.h"

#include "task.h"
#include "semphr.h"

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

    /* Standby mode */
    Write_ADXL345(POWER_CTL  , 0b00000000);

    /* Full resolution mode, +-16g */
    Write_ADXL345(DATA_FORMAT, 0b00001011);

    /* Bypass mode */
    Write_ADXL345(FIFO_CTL   , 0b00000000);

    /* Output data rate 800Hz */
    Write_ADXL345(BW_RATE    , 0b00001101);

    /* Measurement mode */
    Write_ADXL345(POWER_CTL  , 0b00001000);

    kputs("Control Register for ADXL345 had been set\r\n");
}

void ADXL345_Recv(void *arg){
    uint8_t status;
    READ_ADXL345(INT_SOURCE, &status);

    if (!(status & 0b10000000)) {
        return;
    }

    READ_ADXL345(DATAX1, &ADXL345.uint8.XH);
    READ_ADXL345(DATAX0, &ADXL345.uint8.XL);

    READ_ADXL345(DATAY1, &ADXL345.uint8.YH);
    READ_ADXL345(DATAY0, &ADXL345.uint8.YL);

    READ_ADXL345(DATAZ1, &ADXL345.uint8.ZH);
    READ_ADXL345(DATAZ0, &ADXL345.uint8.ZL);
}
