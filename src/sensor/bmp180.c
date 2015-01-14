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

    uint8_t data[2] = {0};
    READ_BMP180(AC1HREG, data, 2);
    BMP180.uint8.AC1H = data[0];
    BMP180.uint8.AC1L = data[1];

    READ_BMP180(AC2HREG, data, 2);
    BMP180.uint8.AC2H = data[0];
    BMP180.uint8.AC2L = data[1];

    READ_BMP180(AC3HREG, data, 2);
    BMP180.uint8.AC3H = data[0];
    BMP180.uint8.AC3L = data[1];

    READ_BMP180(AC4HREG, data, 2);
    BMP180.uint8.AC4H = data[0];
    BMP180.uint8.AC4L = data[1];

    READ_BMP180(AC5HREG, data, 2);
    BMP180.uint8.AC5H = data[0];
    BMP180.uint8.AC5L = data[1];

    READ_BMP180(AC6HREG, data, 2);
    BMP180.uint8.AC6H = data[0];
    BMP180.uint8.AC6L = data[1];

    READ_BMP180(B1HREG, data, 2);
    BMP180.uint8.B1H = data[0];
    BMP180.uint8.B1L = data[1];

    READ_BMP180(B2HREG, data, 2);
    BMP180.uint8.B2H = data[0];
    BMP180.uint8.B2L = data[1];

    READ_BMP180(MBHREG, data, 2);
    BMP180.uint8.MBH = data[0];
    BMP180.uint8.MBL = data[1];

    READ_BMP180(MCHREG, data, 2);
    BMP180.uint8.MCH = data[0];
    BMP180.uint8.MCL = data[1];

    READ_BMP180(MDHREG, data, 2);
    BMP180.uint8.MDH = data[0];
    BMP180.uint8.MDL = data[1];

    kputs("Control Register for BMP180 had been set\r\n");
}

void BMP180_Recv(void *arg){

	// Read UT	
	Write_BMP180(CRLREG, TEMPERATURE);
	vTaskDelay(5);
	uint8_t dataT[2] = {0};
	READ_BMP180(0xF6, dataT, 2);

	UT = dataT[0] << 8 | dataT[1];

	// Read UP, some error, cannot read continueously
	// this will make the I2C susspend...Orz
	/*
	Write_BMP180(CRLREG, PRESSURE0 | (OverSampling << 6));
	vTaskDelay(26);
	uint8_t dataP[3] = {0};
	READ_BMP180(0xF6, dataP, 3);

	UP = dataP[0] << 16 | dataP[1] << 8 | dataP[2] >> (8-OverSampling);
	*/

    // Origin Processs

    /* Calcualte Temperature */
    long x1 = (((long)UT - (long)BMP180.int16.AC6) * (long)BMP180.int16.AC5) >> 15;
    long x2 = ((long)BMP180.int16.MC << 11) / (x1 + BMP180.int16.MD);
    long B5 = x1 + x2;
    // temp = (B5 + 8) >>4;
    // Temperature = temp;
 
    /* Calculate Pressure */
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
    long B4 = (BMP180.int16.AC4 * (unsigned long)(x3 + 32768)) >> 15;

    long B7 = ((unsigned long)(UP-B3) * (50000 >> OverSampling));
    long press = 0;
    if (B7 < 0x80000000)
        press = (B7<<1)/B4;
    else
        press = (B7/B4)<<1;

    x1 = (press>>8)*(press>>8);
    x1 = (x1*3038)>>16;
    x2 = (-7357 * press)>>16;
    press += (x1 + x2 + 3791)>>4;
    // Pressure = press in Pa;
}
