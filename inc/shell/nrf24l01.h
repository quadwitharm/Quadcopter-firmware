#ifndef __NRF24L01_H__
#define __NRF24L01_H__

#include "spi.h"

#define DEVICE_SEND 0
#define DEVICE_RECV 1

void NRF24L01_PowerUp(int deviceNum);
void NRF24L01_PowerDown(int deviceNum);
void NRF24L01_RXMode(int deviceNum);
void NRF24L01_TXMode(int deviceNum);
void NRF24L01_SetFrequency(int deviceNum, uint8_t channel);
void NRF24L01_SetOutputPower(int deviceNum, uint8_t level);
void NRF24L01_IRQ(int deviceNum);
void NRF24L01_SPI_IRQ(int deviceNum);
void NRF24L01_Transmit(int deviceNum, uint8_t buf[], uint32_t size);
void NRF24L01_Receive(int deviceNum, uint8_t buf[], uint32_t size);

#endif
