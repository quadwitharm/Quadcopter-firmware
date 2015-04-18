#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f429i_discovery.h"
#include "stm32f4xx_hal.h"
#include "clib.h"

bool SPI_init(void);
void SPI_send(uint8_t* data, uint16_t length);
void SPI_recv(uint8_t* data, uint16_t length);
void SPI_send_IT(uint8_t* data, uint16_t length);
void SPI_recv_IT(uint8_t* buffer, uint16_t length);
void SPI_send_POLL(uint8_t* data, uint16_t length);
void SPI_recv_POLL(uint8_t* buffer, uint16_t length);
void StartSPIRXInterrupt();

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */
