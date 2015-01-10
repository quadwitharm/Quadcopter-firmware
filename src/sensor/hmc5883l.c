#include "sensor/hmc5883l.h"
#include "sensor/sensor.h"
#include "sensor/i2c.h"

#include "task.h"
#include "semphr.h"

struct HMC5883L HMC5883L;

void Write_HMC5883L(uint8_t Register, uint8_t content){
    uint8_t buf[2];
    buf[0] = Register;
    buf[1] = content;
    while(!I2C_Master_Transmit(HMC5883L_START, buf, 2));
}

void READ_HMC5883L(uint8_t addr,uint8_t buf[]){
    buf[0] = addr;
    do{
        I2C_Master_Transmit(HMC5883L_START, buf, 1);
    }while(!I2C_Master_Receive(HMC5883L_START, buf, 1));
};

void HMC5883L_Init(){
    kputs("Setting Control Register for HMC5883L\r\n");

    /* average 8 per measurement output, output data rate is 75Hz */
    Write_HMC5883L(CRegA, 0b01111000);

    /* +- 1.3Ga */
    Write_HMC5883L(CRegB, 0b00100000);

    /* single-measurement mode */
    Write_HMC5883L(ModeReg, 0b00000001);

    kputs("Control Register for HMC5883L had been set\r\n");
}

void HMC5883L_Recv(void *arg){
    /* TODO: check if data availible*/

    READ_HMC5883L(DataXMSB, &HMC5883L.uint8.XH);
    READ_HMC5883L(DataXLSB, &HMC5883L.uint8.XL);

    READ_HMC5883L(DataYMSB, &HMC5883L.uint8.YH);
    READ_HMC5883L(DataYLSB, &HMC5883L.uint8.YL);

    READ_HMC5883L(DataZMSB, &HMC5883L.uint8.ZH);
    READ_HMC5883L(DataZLSB, &HMC5883L.uint8.ZL);

    setDataReady(HMC58831_DRDY_BIT);
}
