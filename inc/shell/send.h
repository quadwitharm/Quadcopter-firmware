#ifndef __SEND_H__
#define __SEND_H__

#include "clib.h"
#include "uart.h"

void SendCommand_3(uint8_t head,uint8_t head2,uint8_t content[],int len);
void SendCommand_2(uint8_t head,uint8_t content[],int len);

#endif
