#ifndef __HMC5883L_H__
#define __HMC5883L_H__
#ifdef __cplusplus
extern "C" {
#endif

#include "clib.h"
    struct HMC5883L{
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
    extern struct HMC5883L HMC5883L;

    void HMC5883L_Init();
    void HMC5883L_Recv();
    void HMC5883L_Process(void*);

    /* Device Address */
#define HMC5883L_START 0x3C // Write

#define CRegA       0x00
#define CRegB       0x01
#define ModeReg     0x02
#define DataXMSB    0x03
#define DataXLSB    0x04
#define DataZMSB    0x05
#define DataZLSB    0x06
#define DataYMSB    0x07
#define DataYLSB    0x08
#define StatusReg   0x09
#define IDRegA      0x10
#define IDRegB      0x11
#define IDRegC      0x12

#ifdef __cplusplus
}
#endif
#endif
