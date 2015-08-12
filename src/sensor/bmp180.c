#include "sensor/bmp180.h"
#include "sensor/sensor.h"
#include "sensor/i2c.h"

#include "task.h"
#include "semphr.h"

static xSemaphoreHandle BMP180_Lock;
struct BMP180 BMP180;
long UT;
long UP;

void Write_BMP180(uint8_t Register, uint8_t content){
    uint8_t buf[2];
    buf[0] = Register;
    buf[1] = content;
    while(!I2C_Master_Transmit(BMP180_START, buf, 2));
}

void READ_BMP180(uint8_t addr, uint8_t buf[], uint8_t size){
    buf[0] = addr;
    do{
        I2C_Master_Transmit(BMP180_START, buf, size);
    }while(!I2C_Master_Receive(BMP180_START, buf, size));
};

void BMP180_Init(){
    BMP180_Lock = xSemaphoreCreateMutex();

    kputs("Setting Control Register for BMP180\r\n");

    READ_BMP180(AC1HREG, &BMP180.uint8.AC1H, 1);
    READ_BMP180(AC1LREG, &BMP180.uint8.AC1L, 1);
    READ_BMP180(AC2HREG, &BMP180.uint8.AC2H, 1);
    READ_BMP180(AC2LREG, &BMP180.uint8.AC2L, 1);
    READ_BMP180(AC3HREG, &BMP180.uint8.AC3H, 1);
    READ_BMP180(AC3LREG, &BMP180.uint8.AC3L, 1);
    READ_BMP180(AC4HREG, &BMP180.uint8.AC4H, 1);
    READ_BMP180(AC4LREG, &BMP180.uint8.AC4L, 1);
    READ_BMP180(AC5HREG, &BMP180.uint8.AC5H, 1);
    READ_BMP180(AC5LREG, &BMP180.uint8.AC5L, 1);
    READ_BMP180(AC6HREG, &BMP180.uint8.AC6H, 1);
    READ_BMP180(AC6LREG, &BMP180.uint8.AC6L, 1);
    READ_BMP180(B1HREG, &BMP180.uint8.B1H, 1);
    READ_BMP180(B1LREG, &BMP180.uint8.B1L, 1);
    READ_BMP180(B2HREG, &BMP180.uint8.B2H, 1);
    READ_BMP180(B2LREG, &BMP180.uint8.B2L, 1);
    READ_BMP180(MBHREG, &BMP180.uint8.MBH, 1);
    READ_BMP180(MBLREG, &BMP180.uint8.MBL, 1);
    READ_BMP180(MCHREG, &BMP180.uint8.MCH, 1);
    READ_BMP180(MCLREG, &BMP180.uint8.MCL, 1);
    READ_BMP180(MDHREG, &BMP180.uint8.MDH, 1);
    READ_BMP180(MDLREG, &BMP180.uint8.MDL, 1);

    kputs("Control Register for BMP180 had been set\r\n");
}

void BMP180_Recv(){

    // Read UT	
    Write_BMP180(PRESSURE3, TEMPERATURE);
    vTaskDelay(5);
    uint8_t dataT[2];
    READ_BMP180(0xF6, &dataT[0], 1);
    READ_BMP180(0xF7, &dataT[1], 1);
    UT = dataT[0] << 8 | dataT[1];

    Write_BMP180(PRESSURE3, PRESSURE0 | (OverSampling << 6));
    vTaskDelay(26);
    uint8_t dataP[3] = {0};
    READ_BMP180(0xF6, &dataP[0], 1);
    READ_BMP180(0xF7, &dataP[1], 1);
    READ_BMP180(0xF8, &dataP[2], 1);
    UP = dataP[0] << 16 | dataP[1] << 8 | dataP[2] >> (8-OverSampling);

    /* Calcualte Temperature */
    long x1 = (((long)UT - (long)BMP180.int16.AC6) * (long)BMP180.int16.AC5) >> 15;
    long x2 = ((long)BMP180.int16.MC << 11) / (x1 + BMP180.int16.MD);
    long B5 = x1 + x2;
    BMP180.Temperature = (B5 + 8) >>4;

    /* Calculate BMP180.Pressure */
    // Calculate B3
    long B6 = B5 - 4000;
    x1 = (BMP180.int16.B2 * ((B6 * B6) >> 12)) >> 11;
    x2 = (BMP180.int16.AC2 * B6) >> 11;
    long x3 = x1 + x2;
    long B3 = ((((((long)BMP180.int16.AC1)<<2) + x3)<<OverSampling) + 2)>>2;

    // Calculate B4
    x1 = (BMP180.int16.AC3 * B6) >> 13;
    x2 = (BMP180.int16.B1 * ((B6 * B6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2)>>2;
    unsigned long B4 = (BMP180.int16.AC4 * (unsigned long)(x3 + 32768)) >> 15;

    unsigned long B7 = ((unsigned long)(UP-B3) * (50000 >> OverSampling));
    if (B7 < 0x80000000)
        BMP180.Pressure = (B7<<1)/B4;
    else
        BMP180.Pressure = (B7/B4)<<1;

    x1 = (BMP180.Pressure>>8)*(BMP180.Pressure>>8);
    x1 = (x1*3038)>>16;
    x2 = (-7357 * BMP180.Pressure)>>16;
    BMP180.Pressure += (x1 + x2 + 3791)>>4;
}
