#ifndef __BMP180_H__
#define __BMP180_H__
#ifdef __cplusplus
extern "C" {
#endif

#include "clib.h"

    struct BMP180{
        union {
            /* Assume little endian */
            struct {
                uint8_t AC1L;
                uint8_t AC1H;
                uint8_t AC2L;
                uint8_t AC2H;
                uint8_t AC3L;
                uint8_t AC3H;
                uint8_t AC4L;
                uint8_t AC4H;
                uint8_t AC5L;
                uint8_t AC5H;
                uint8_t AC6L;
                uint8_t AC6H;
                uint8_t B1L;
                uint8_t B1H;
                uint8_t B2L;
                uint8_t B2H;
                uint8_t MBL;
                uint8_t MBH;
                uint8_t MCL;
                uint8_t MCH;
                uint8_t MDL;
                uint8_t MDH;
            }uint8;
            struct {
                int16_t AC1;
                int16_t AC2;
                int16_t AC3;
                uint16_t AC4;
                uint16_t AC5;
                uint16_t AC6;
                int16_t B1;
                int16_t B2;
                int16_t MB;
                int16_t MC;
                int16_t MD;
            }int16;
        };
        long Pressure;
        long Temperature;
    };
    extern struct BMP180 BMP180;

    void BMP180_Init();
    void BMP180_Recv();

    /* Device Address */
#define BMP180_START 0xEE

#define AC1HREG      0xAA
#define AC1LREG      0xAB
#define AC2HREG	     0xAC
#define AC2LREG      0xAD
#define AC3HREG      0xAE
#define AC3LREG      0xAF
#define AC4HREG	     0xB0
#define AC4LREG      0xB1
#define AC5HREG      0xB2
#define AC5LREG      0xB3
#define AC6HREG	     0xB4
#define AC6LREG      0xB5
#define B1HREG       0xB6
#define B1LREG       0xB7
#define B2HREG	     0xB8
#define B2LREG       0xB9
#define MBHREG       0xBA
#define MBLREG       0xBB
#define MCHREG	     0xBC
#define MCLREG       0xBD
#define MDHREG	     0xBE
#define MDLREG       0xBF

#define TEMP_MSB     0xF6
#define TEMP_LSB     0xF7
#define TEMP_XLSB    0xF8

#define TEMPERATURE  0x2E
#define PRESSURE0    0x34
#define PRESSURE1    0x74
#define PRESSURE2    0xB4
#define PRESSURE3    0xF4
#define CONTROL_REG  0xF4
#define OverSampling 3

#ifdef __cplusplus
}
#endif
#endif
