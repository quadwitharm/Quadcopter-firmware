#include "sensor/l3g4200d.h"
#include "sensor/sensor.h"
#include "sensor/i2c.h"

#include "task.h"
#include "semphr.h"

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
    kputs("Setting Control Register for L3G4200D\r\n");

    /* ODR 800Hz, Cut-off: 30 */
    Write_L3G4200D(CTRL_REG1, 0b11001111);

    /* Block data update, 250dps (0.00875 * value degree per second) */
    Write_L3G4200D(CTRL_REG4, 0b10000000);

    /* Enable FIFO & reboot memory content */
    Write_L3G4200D(CTRL_REG5, 0b11000000);

    /* Stream mode, Watermark level: 16 */
    Write_L3G4200D(FIFO_CTRL_REG, 0b01010000);

    kputs("Control Register for L3G4200D had been set\r\n");
}

void L3G4200D_Recv(void *arg){
    uint8_t FIFO_STATUS;
    READ_L3G4200D(FIFO_SRC_REG, &FIFO_STATUS);
    /* Data not available yet */
    if(FIFO_STATUS & 0b00100000){
        return;
    }

    READ_L3G4200D(OUT_X_H, &L3G4200D.uint8.XH);
    READ_L3G4200D(OUT_X_L, &L3G4200D.uint8.XL);

    READ_L3G4200D(OUT_Y_H, &L3G4200D.uint8.YH);
    READ_L3G4200D(OUT_Y_L, &L3G4200D.uint8.YL);

    READ_L3G4200D(OUT_Z_H, &L3G4200D.uint8.ZH);
    READ_L3G4200D(OUT_Z_L, &L3G4200D.uint8.ZL);
    setDataReady(L3G4200D_DRDY_BIT);
}
