#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f429i_discovery.h"
#include "stm32f4xx_hal.h"
#include "clib.h"

//as handler array index
#define SPI_TX 0
#define SPI_RX 1

bool SPI_init(void);

void SPI_sendRecv(int nspi,uint8_t *txData,uint8_t *rxData, uint16_t length);
void SPI_sendRecv_IT(int nspi,uint8_t *txData,uint8_t *rxData, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */
