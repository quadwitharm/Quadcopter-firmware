#ifndef __AXDL345_H__
#define __AXDL345_H__
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
    struct AXDL345{
        uint8_t XH;
        uint8_t XL;
        uint8_t YH;
        uint8_t YL;
        uint8_t ZH;
        uint8_t ZL;
    };

    void AXDL345L_Init();
    void AXDL345L_Recv(void*);
    void AXDL345L_Process(void*);

    /* Device Address */
#define _START


#ifdef __cplusplus
}
#endif
#endif
