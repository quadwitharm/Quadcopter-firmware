#include "sensor/hmc5883l.h"
#include "sensor/sensor.h"
#include "sensor/i2c.h"

#include "task.h"
#include "semphr.h"

static xSemaphoreHandle HMC5883L_Lock;
static bool dataAvailable = false;
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
    HMC5883L_Lock = xSemaphoreCreateMutex();

    kputs("Setting Control Register for HMC5883L\r\n");

    /* average 8 per measurement output, output data rate is 15Hz */
    Write_HMC5883L(CRegA, 0b00011000);

    /* +- 1.3 Ga, 1090 Gain */
    Write_HMC5883L(CRegB, 0b00100000);

    /* single-measurement mode */
    Write_HMC5883L(ModeReg, 0b00000001);

    kputs("Control Register for HMC5883L had been set\r\n");
}

void HMC5883L_Recv(void *arg){
    while(1){
        xSemaphoreTake( HMC5883L_Lock, ( portTickType ) portMAX_DELAY );
        uint8_t READY_STATUS;
        READ_HMC5883L(StatusReg, &READY_STATUS, 1);

        /* Data not available yet */
        if(READY_STATUS & 0b00000010){
            xSemaphoreGive( HMC5883L_Lock );
            continue;
        }

	uint8_t tmpbuff[6] = { 0 };
	READ_HMC5883L(DataXMSB, tmpbuff, 6);

	HMC5883L.uint8.XH = tmpbuff[0];
	HMC5883L.uint8.XL = tmpbuff[1];

	HMC5883L.uint8.YH = tmpbuff[2];
	HMC5883L.uint8.YL = tmpbuff[3];

	HMC5883L.uint8.ZH = tmpbuff[4];
	HMC5883L.uint8.ZL = tmpbuff[5];

    	/* single-measurement mode */
    	Write_HMC5883L(ModeReg, 0b00000001);

        dataAvailable = true;
        xSemaphoreGive( HMC5883L_Lock );
    }
}
void HMC5883L_Process(void *arg){
    xSemaphoreTake( HMC5883L_Lock, ( portTickType ) portMAX_DELAY );

    /* Assume data will be proccessed before next read to data */
    if(!dataAvailable){
        xSemaphoreGive( HMC5883L_Lock );
        return;
    }
    dataAvailable = false;

    xSemaphoreGive( HMC5883L_Lock );
}
