#ifndef __HMC58831_H__
#define __HMC58831_H__
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
    struct HMC58831{
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

    void HMC58831_Init();
    void HMC58831_Recv(void*);
    void HMC58831_Process(void*);

    /* Device Address */
//#define _START
#define CRegA 		0x00
#define CRegB 		0x01
#define ModeReg 	0x02
#define DataXMSB	0x03
#define DataXLSB	0x04
#define DataZMSB	0x05
#define DataZLSB	0x06
#define DataYMSB	0x07
#define DataYLSB	0x08
#define StatusReg	0x09
#define IDRegA		0x10
#define IDRegB		0x11
#define IDRegC		0x12

#ifdef __cplusplus
}
#endif
#endif
