#include "sensor/l3g4200d.h"
#include "sensor/sensor.h"
#include "sensor/i2c.h"

#include "task.h"
#include "semphr.h"

static xSemaphoreHandle L3G4200D_Lock;
static bool dataAvailable = false;
struct L3G4200D L3G4200D;

void Write_L3G4200D(uint8_t Register, uint8_t content){
    uint8_t buf[2];
    buf[0] = Register;
    buf[1] = content;
    while(!I2C_Master_Transmit(L3G4200D_START, buf, 2));
}

void READ_L3G4200D(uint8_t addr,uint8_t buf[]){
    buf[0] = addr;
    do{
        I2C_Master_Transmit(L3G4200D_START, buf, 1);
    }while(!I2C_Master_Receive(L3G4200D_START, buf, 1));
};

void L3G4200D_Init(){
    L3G4200D_Lock = xSemaphoreCreateMutex();

    kputs("Setting Control Register for L3G4200D\r\n");


    /* ODR 800Hz, Cut-off: 30 */
    Write_L3G4200D(CTRL_REG1, 0b11001111);
    /* Block data update, 250dps (0.00875 * value degree per second) */
    Write_L3G4200D(CTRL_REG4, 0b10000000);
    /* Enable FIFO & reboot memory content */
    Write_L3G4200D(CTRL_REG5, 0b11000000);
    /* Stream mode, Watermark level: 16 */
    Write_L3G4200D(FIFO_CTRL_REG, 0b01010000);

    Write_L3G4200D(FIFO_CTRL_REG, 0b01010000);
    kputs("Control Register for L3G4200D had been set\r\n");
}

void L3G4200D_Recv(void *arg){
    while(1){
        xSemaphoreTake( L3G4200D_Lock, ( portTickType ) portMAX_DELAY );
        uint8_t FIFO_STATUS;
        READ_L3G4200D(FIFO_SRC_REG, &FIFO_STATUS);

        /* Data not available yet */
        if(FIFO_STATUS & 0b00100000){
            xSemaphoreGive( L3G4200D_Lock );
            continue;
        }

        READ_L3G4200D(OUT_X_H, &L3G4200D.uint8.XH);
        READ_L3G4200D(OUT_X_L, &L3G4200D.uint8.XL);

        READ_L3G4200D(OUT_Y_H, &L3G4200D.uint8.YH);
        READ_L3G4200D(OUT_Y_L, &L3G4200D.uint8.YL);

        READ_L3G4200D(OUT_Z_H, &L3G4200D.uint8.ZH);
        READ_L3G4200D(OUT_Z_L, &L3G4200D.uint8.ZL);

        dataAvailable = true;
        xSemaphoreGive( L3G4200D_Lock );
    }
}
void L3G4200D_Process(void *arg){
    xSemaphoreTake( L3G4200D_Lock, ( portTickType ) portMAX_DELAY );

    /* Assume data will be proccessed before next read to data */
    if(!dataAvailable){
        xSemaphoreGive( L3G4200D_Lock );
        return;
    }
    dataAvailable = false;

    L3G4200D.uint8.XL = L3G4200D.uint8.XL & 0b11111100;
    L3G4200D.uint8.YL = L3G4200D.uint8.YL & 0b11111100;
    L3G4200D.uint8.ZL = L3G4200D.uint8.ZL & 0b11111100;

    vAttitude.row   = L3G4200D.int16.X;// * 0.00875;
    vAttitude.pitch = L3G4200D.int16.Y;// * 0.00875;
    vAttitude.yaw   = L3G4200D.int16.Z;// * 0.00875;

    xAttitude.row   += vAttitude.row * .00125;
    xAttitude.pitch += vAttitude.pitch * .00125;
    xAttitude.yaw   += vAttitude.yaw * .00125;

    xSemaphoreGive( L3G4200D_Lock );
}
