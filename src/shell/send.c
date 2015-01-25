#include "shell/send.h"
#include "shell/b64.h"
#include "sensor/sensor.h"
#include "controller/control_api.h"
#include "task.h"
#include "uart.h"
#include "semphr.h"

xSemaphoreHandle txSemaphore;

void SendInfoTask(void *args){
    while(1){
        sendControlInfo();
        sendSensorInfo();
    }
}

void Init_IO(){
    txSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(txSemaphore);
}

bool Init_SendInfoTask(){
    // FreeRTOS Task
    bool ret = xTaskCreate(SendInfoTask,
            (portCHAR *)"ControllerTask",
            256,
            NULL,
            tskIDLE_PRIORITY + 1,
            NULL);
    if(ret != pdPASS)return false;
    return ret;
}

/**
 * @brief      Send a command that have two headers
 * @param[in]  head  First byte of header
 * @param[in]  head2 Second byte of header
 * @param[in]  content Content of command
 * @param[in]  len Length of content
 * @retval None
 */
void SendCommand_3(uint8_t head,uint8_t head2,uint8_t content[],int len){
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

    xSemaphoreTake(txSemaphore, portMAX_DELAY);
    UART_send(outbuf, outlen + 1);
    xSemaphoreGive(txSemaphore);
}

/**
 * @brief      Send a command that have one header
 * @param[in]  head  First byte of header
 * @param[in]  content Content of command
 * @param[in]  len Length of content
 * @retval None
 */
void SendCommand_2(uint8_t head,uint8_t content[],int len){
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

    xSemaphoreTake(txSemaphore, portMAX_DELAY);
    UART_send(outbuf, outlen + 1);
    xSemaphoreGive(txSemaphore);
}

