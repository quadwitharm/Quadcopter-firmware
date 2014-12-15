#ifndef __ADXL345_H__
#define __ADXL345_H__
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
    struct ADXL345{
        uint8_t XH;
        uint8_t XL;
        uint8_t YH;
        uint8_t YL;
        uint8_t ZH;
        uint8_t ZL;
    };

    void ADXL345_Init();
    void ADXL345_Recv(void*);
    void ADXL345_Process(void*);

    /* Device Address */
#define ADXL345_START   0xA6
#define DEVID           0x00
#define THRESH_TAP      0x1E
#define OFSX            0x1E
#define OFSY            0x1F
#define OFSZ            0x20
#define DUR             0x21
#define Latent          0x22
#define Window          0x23
#define THRESH_ACT      0x24
#define THRESH_INACT    0x25
#define TIME_INACT      0x26
#define ACT_INACT_CTL   0x27
#define THRESH_FF       0x28
#define TIME_FF         0x29
#define TAP_AXES        0x2A
#define ACT_TAP_STATUS  0x2B
#define BW_RATE         0x2C
#define POWER_CTL       0x2D
#define INT_ENABLE      0x2E
#define INT_MAP         0x2F
#define INT_SOURCE      0x30
#define DATA_FORMAT     0x31
#define DATAX0          0x32
#define DATAX1          0x33
#define DATAY0          0x34
#define DATAY1          0x35
#define DATAZ0          0x36
#define DATAZ1          0x37
#define FIFO_CTL        0x38
#define FIFO_STATUS     0x39

#ifdef __cplusplus
}
#endif
#endif
