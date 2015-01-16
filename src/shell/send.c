#include "shell/send.h"
#include "shell/b64.h"
#include "task.h"
#include "uart.h"

void SendCommand_3(uint8_t head,uint8_t head2,uint8_t content[],int len){
    taskENTER_CRITICAL();
    uint8_t inbuf[len + 5];
    inbuf[0] = head;
    inbuf[1] = head2;
    memcpy(inbuf+2,content,len);

    uint8_t checksum = 0;
    for(int i = 0;i < len + 2;++i){
        checksum += inbuf[i];
    }
    inbuf[len + 2] = checksum;

    int outlen = getB64EncodeLen(len + 3);
    uint8_t outbuf[outlen];
    b64Encode(inbuf,outbuf,len+3);
    outbuf[outlen] = 0xFF;

    UART_send(outbuf, outlen + 1);
    taskEXIT_CRITICAL();
}

void SendCommand_2(uint8_t head,uint8_t content[],int len){
    taskENTER_CRITICAL();
    uint8_t inbuf[len + 4];
    inbuf[0] = head;
    memcpy(inbuf+1,content,len);

    uint8_t checksum = 0;
    for(int i = 0;i < len + 1;++i){
        checksum += inbuf[i];
    }
    inbuf[len + 1] = checksum;

    int outlen = getB64EncodeLen(len + 2);
    uint8_t outbuf[outlen+1];
    b64Encode(inbuf,outbuf,len+2);
    outbuf[outlen] = 0xFF;

    UART_send(outbuf, outlen + 1);
    taskEXIT_CRITICAL();
}

