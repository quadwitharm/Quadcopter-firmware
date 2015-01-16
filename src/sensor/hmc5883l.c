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

void READ_HMC5883L(uint8_t addr,uint8_t buf[], uint8_t size){
    buf[0] = addr;
    do{
        I2C_Master_Transmit(HMC5883L_START, buf, size);
    }while(!I2C_Master_Receive(HMC5883L_START, buf, size));
};

void HMC5883L_Init(){
    kputs("Setting Control Register for HMC5883L\r\n");

    /* average 8 per measurement output, output data rate is 15Hz */
    Write_HMC5883L(CRegA, 0b01110000);

    /* +- 1.3 Ga, 1090 Gain */
    Write_HMC5883L(CRegB, 0b00100000);

    /* continue-measurement mode */
    Write_HMC5883L(ModeReg, 0b00000000);

    kputs("Control Register for HMC5883L had been set\r\n");
}

void HMC5883L_Recv(){
    //uint8_t status;
    //READ_HMC5883L(StatusReg, &status, 1);

    //if (status & 0x00000010){
    //    return;
    //}
    //uint8_t tmpbuff[6];
    //READ_HMC5883L(DataXMSB, tmpbuff, 6);
    uint8_t tmpbuf[2];
    READ_HMC5883L(DataXMSB, tmpbuf, 1);
    HMC5883L.uint8.XH = tmpbuf[0];
    READ_HMC5883L(DataXLSB, tmpbuf, 1);
    HMC5883L.uint8.XL = tmpbuf[0];

    READ_HMC5883L(DataZMSB, tmpbuf, 1);
    HMC5883L.uint8.ZH = tmpbuf[0];
    READ_HMC5883L(DataZLSB, tmpbuf, 1);
    HMC5883L.uint8.ZL = tmpbuf[0];

    READ_HMC5883L(DataYMSB, tmpbuf, 1);
    HMC5883L.uint8.YH = tmpbuf[0];
    READ_HMC5883L(DataYLSB, tmpbuf, 1);
    HMC5883L.uint8.YL = tmpbuf[0];

}
