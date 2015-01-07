#ifndef __BMP180_H__
#define __BMP180_H__
#ifdef __cplusplus
extern "C" {
#endif

#include "clib.h"

    struct BMP180{
        uint8_t XH;
        uint8_t XL;
        uint8_t YH;
        uint8_t YL;
        uint8_t ZH;
        uint8_t ZL;
    };

    void BMP180_Init();
    void BMP180_Recv(void*);
    void BMP180_Process(void*);

    /* Device Address */
#define BMP180_START 0xEE


#ifdef __cplusplus
}
#endif
#endif
