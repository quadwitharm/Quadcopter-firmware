#ifndef __L3G4200D_H__
#define __L3G4200D_H__
#ifdef __cplusplus
extern "C" {
#endif

#include "clib.h"

    struct L3G4200D{
        union{
            struct{
                uint8_t XL;
                uint8_t XH;
                uint8_t YL;
                uint8_t YH;
                uint8_t ZL;
                uint8_t ZH;
            }uint8;
            struct{
                int16_t X;
                int16_t Y;
                int16_t Z;
            }int16;
        };
    };
    extern struct L3G4200D L3G4200D;

    void L3G4200D_Init();
    void L3G4200D_Recv(void*);
    void L3G4200D_Process(void*);

    /* Device Address */
#define L3G4200D_START  0xD2 // SDO connected
#define WHO_AM_I        0x0F // 0b11010011
#define CTRL_REG1       0x20
#define CTRL_REG2       0x21
#define CTRL_REG3       0x22
#define CTRL_REG4       0x23
#define CTRL_REG5       0x24
#define REFERENCE       0x25
#define OUT_TEMP        0x26
#define STATUS_REG      0x27
#define OUT_X_L         0x28
#define OUT_X_H         0x29
#define OUT_Y_L         0x2A
#define OUT_Y_H         0x2B
#define OUT_Z_L         0x2C
#define OUT_Z_H         0x2D
#define FIFO_CTRL_REG   0x2E
#define FIFO_SRC_REG    0x2F
#define INT1_CFG        0x30
#define INT1_SRC        0x31
#define INT1_TSH_XH     0x32
#define INT1_TSH_XL     0x33
#define INT1_TSH_YH     0x34
#define INT1_TSH_YL     0x35
#define INT1_TSH_ZH     0x36
#define INT1_TSH_ZL     0x37
#define INT1_DURATION   0x38


#ifdef __cplusplus
}
#endif
#endif
