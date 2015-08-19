#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f429i_discovery.h"
#include "stm32f4xx_hal.h"
#include "clib.h"

bool SPI_init(void);

void SPI_sendRecv(int nspi,uint8_t *txData,uint8_t *rxData, uint16_t length);
void SPI_sendRecv_IT(int nspi,uint8_t *txData,uint8_t *rxData, uint16_t length);
void SPI_sendRecv_POLL(int nspi,uint8_t *txData,uint8_t *rxData, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */
