#include "spi.h"
#include "clib.h"

#include "semphr.h"

bool SPI_init(void){
    /* Init SPI1, SPI2 with following pin:
     * SPI1:
     *  + SCK:  PB3
     *  + MISO: PB4
     *  + MOSI: PB5
     *
     * SPI2:
     *  + SCK:  PB13
     *  + MISO: PB14
     *  + MOSI: PB15
     *
     *  Configure the interrupts but do not enable.
     * */
}

void SPI_send(uint8_t* data, uint16_t length){
    /*
     * Use polling mode if FreeRTOS hasn't startup yet,
     * otherwise else interrupt mode
     */
}

void SPI_recv(uint8_t* data, uint16_t length){
    /*
     * Use polling mode if FreeRTOS hasn't startup yet,
     * otherwise else interrupt mode
     */
}

void SPI_send_IT(uint8_t* data, uint16_t length){

}

void SPI_recv_IT(uint8_t* buffer, uint16_t length){

}

void SPI_send_POLL(uint8_t* data, uint16_t length){

}

void SPI_recv_POLL(uint8_t* buffer, uint16_t length){

}

void StartSPIRXInterrupt(){

}
