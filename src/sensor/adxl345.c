#include "sensor/adxl345.h"
#include "sensor/sensor.h"
#include "sensor/i2c.h"

#include "task.h"
#include "semphr.h"

static xSemaphoreHandle ADXL345_Lock;
static bool dataAvailable = false;
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
    ADXL345_Lock = xSemaphoreCreateMutex();
    /* Init control register */
    kputs("Setting Control Register for ADXL345\r\n");

    /* standby mode */
    Write_ADXL345(POWER_CTL  , 0b00000000);

    /* full resolution mode, +-16g */
    Write_ADXL345(DATA_FORMAT, 0b00001011);

    /* bypass mode */
    Write_ADXL345(FIFO_CTL   , 0b00000000);

    /* output data rate 100Hz */
    Write_ADXL345(BW_RATE    , 0b00001010);

    /* measurement mode */
    Write_ADXL345(POWER_CTL  , 0b00001000);

    kputs("Control Register for ADXL345 had been set\r\n");
}

void ADXL345_Recv(void *arg){
    while(1){
        xSemaphoreTake( ADXL345_Lock, ( portTickType ) portMAX_DELAY );
        uint8_t _STATUS;
	READ_ADXL345(INT_SOURCE, &_STATUS);

	/* Data not available yet */
        if(!(_STATUS & 0b10000000)){
            xSemaphoreGive( ADXL345_Lock );
            continue;
        }
        
	READ_ADXL345(DATAX1, &ADXL345.uint8.XH);
	READ_ADXL345(DATAX0, &ADXL345.uint8.XL);

	READ_ADXL345(DATAY1, &ADXL345.uint8.YH);
	READ_ADXL345(DATAY0, &ADXL345.uint8.YL);

	READ_ADXL345(DATAZ1, &ADXL345.uint8.ZH);
	READ_ADXL345(DATAZ0, &ADXL345.uint8.ZL);

	dataAvailable = true;

        xSemaphoreGive( ADXL345_Lock );
    }
}

void _printFloat(float out){
	if(out < 0){
		kputc('-');
		out = -out;
	}
	kputs(itoa(out, 10));
	kputc('.');
	kputs(itoa((out - (int)out) * 100000, 10));
}

void ADXL345_Process(void *arg){
    xSemaphoreTake( ADXL345_Lock, ( portTickType ) portMAX_DELAY );
    
    if (!dataAvailable){
        xSemaphoreGive( ADXL345_Lock );
	return;
    }
    dataAvailable = false;

    aAttitude.row = ADXL345.int16.X;
    aAttitude.pitch = ADXL345.int16.Y;
    aAttitude.yaw = ADXL345.int16.Z;
	
    _printFloat(aAttitude.row);
    kputc(',');
    _printFloat(aAttitude.pitch);
    kputc(',');
    _printFloat(aAttitude.yaw);
    kputs("\r\n");

    xSemaphoreGive( ADXL345_Lock );
}
