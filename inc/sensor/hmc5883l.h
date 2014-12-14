#ifndef __HMC58831_H__
#define __HMC58831_H__
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
    struct HMC58831{
        uint8_t XH;
        uint8_t XL;
        uint8_t YH;
        uint8_t YL;
        uint8_t ZH;
        uint8_t ZL;
    };

    void HMC58831_Init();
    void HMC58831_Recv(void*);
    void HMC58831_Process(void*);

    /* Device Address */
#define _START


#ifdef __cplusplus
}
#endif
#endif
