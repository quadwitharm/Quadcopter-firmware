#include "shell/nrf24l01.h"

#include "spi.h"

/* NRF24L01 registers */

#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD       0x1c
#define FEATURE     0x1d

/* NRF24L01 SPI Commands */

#define R_REGISTER(regaddr) ((regaddr) & 0b00011111)
#define W_REGISTER(regaddr) (((regaddr) & 0b00011111) | 0b00100000)
#define R_RX_PAYLOAD        0b01100001
#define W_TX_PAYLOAD        0b10100000
#define FLUSH_TX            0b11100001
#define FLUSH_RX            0b11100010
#define REUSE_TX_PL         0b11100011
#define ACTIVATE            0b01010000
#define R_RX_PL_WID         0b01100000
#define W_ACK_PAYLOAD(pipe) (0b10101000 | (pipe))
#define W_TX_PAYLOAD_NOACK  /* missing bit in datasheet */
#define NOP                 0b11111111

#define RF_CHANNEL_1 100
#define RF_CHANNEL_2 200

/*
 * @retval STATUS register
 */
static uint8_t NRF24L01_WriteBuf(int deviceNum, uint8_t cmd, uint8_t *buf, uint32_t size){
    uint8_t inbuf[33];
    uint8_t outbuf[33];
    outbuf[0] = cmd;
    memcpy(outbuf + 1, buf, size);

    /* XXX: Not completed API
     * CSN = 0;
     * TransmitReceive_IT( spi, outbuf, inbuf, size+1 );
     * CSN = 1;
     */
    (void)outbuf, (void)inbuf;

    return inbuf[0]; /* STATUS register */
}

/*
 * @retval STATUS register
 */
static uint8_t NRF24L01_ReadBuf(int deviceNum, uint8_t cmd, uint8_t *buf, uint32_t size){
    uint8_t inbuf[33];
    uint8_t outbuf[33];
    outbuf[0] = cmd;

    /* XXX: Not completed API
     * CSN = 0;
     * TransmitReceive_IT( spi, outbuf, inbuf, size+1 );
     * CSN = 1;
     */
    (void)outbuf, (void)inbuf;
    memcpy(buf, inbuf, size);

    return inbuf[0]; /* STATUS register */
}

/*
 * @retval STATUS register
 */
static uint8_t NRF24L01_WriteReg(int deviceNum, uint8_t cmd, uint8_t value){
    uint8_t inbuf[2];
    uint8_t outbuf[2] = { cmd, value };

    /* XXX: Not completed API
     * CSN = 0;
     * TransmitReceive_IT( spi, outbuf, inbuf, 2 );
     * CSN = 1;
     */
    (void)outbuf, (void)inbuf;

    return inbuf[0];
}

/*
 * @retval STATUS register
 */
static uint8_t NRF24L01_ReadReg(int deviceNum, uint8_t cmd, uint8_t *val){
    uint8_t inbuf[2];
    uint8_t outbuf[2] = { cmd };

    /* XXX: Not completed API
     * CSN = 0;
     * TransmitReceive_IT( spi, outbuf, inbuf, size + 1 );
     * CSN = 1;
     */
    (void)outbuf, (void)inbuf;

    *val = inbuf[1];
    return inbuf[0];
}


/*
 * @brief   Trasition from PowerDown mode to Standby-I mode
 * @retval  None
 */
void NRF24L01_PowerUp(int deviceNum){
    /*
     * TODO:
     * set PWR_UP = 1
     * wait for 1.5ms
     */
}

/*
 * @brief   PowerDown the NRF24L01 device
 * @retval  None
 * */
void NRF24L01_PowerDown(int deviceNum){
    /*
     * TODO:
     * set PWR_UP = 0
     */
}

void NRF24L01_RXMode(int deviceNum){

    /* Reserved | RX_DR | no TX_DS | no MAX_RT | CRC | <-2Byte | PWR_UP | RX */
    NRF24L01_WriteReg(deviceNum, W_REGISTER(CONFIG), 0b00111111);
    /* Reserved * 3 | no PLL_LOCK | 2MBps | 0 dBm | LNA gain */
    NRF24L01_WriteReg(deviceNum, W_REGISTER(RF_SETUP), 0b00001111);
    /* Channel */
    NRF24L01_WriteReg(deviceNum, W_REGISTER(RF_CH), RF_CHANNEL_1);
    /* Dynamic payload length */
    NRF24L01_WriteReg(deviceNum, W_REGISTER(DYNPD), 0b00000001);
    /* Enable auto acknowledgement on pipe 0 */
    NRF24L01_WriteReg(deviceNum, W_REGISTER(EN_AA), 0b00000001);
    /* Enable RX Address*/
    NRF24L01_WriteReg(deviceNum, W_REGISTER(EN_RXADDR), 0b00000001);


}

void NRF24L01_TXMode(int deviceNum){
    /*
     * TODO:
     * Set CE = 0
     */

    /* Reserved | no RX_DR | TX_DS | MAX_RT | CRC | <-2Byte | PWR_UP | TX */
    NRF24L01_WriteReg(deviceNum, W_REGISTER(CONFIG), 0b01001110);
    /* Reserved * 3 | no PLL_LOCK | 2MBps | 0 dBm | LNA gain */
    NRF24L01_WriteReg(deviceNum, W_REGISTER(RF_SETUP), 0b00001111);
    /* Channel */
    NRF24L01_WriteReg(deviceNum, W_REGISTER(RF_CH), RF_CHANNEL_1);
    /* Retransmission: 500us, 3 times */
    NRF24L01_WriteReg(deviceNum, W_REGISTER(SETUP_RETR), 0b00010011);
    /* Dynamic payload length */
    NRF24L01_WriteReg(deviceNum, W_REGISTER(DYNPD), 0b00000001);
    /*
     * TODO:
     * Set CE = 1
     */
}

/*
 * @brief   PowerDown the NRF24L01 device
 * @param[in] channel: Output frequency is 2400 + channel MHz (deviceNumfrom 0 ~ 125)
 * @retval  None
 * */
void NRF24L01_SetFrequency(int deviceNum, uint8_t channel){
    NRF24L01_WriteReg(deviceNum, W_REGISTER(RF_CH), channel & 0b01111111);
}

/*
 * @brief   PowerDown the NRF24L01 device
 * @param[in] level: from 0~3, higher level implies higher output power
 *   3:   0dBm, 11.3mA
 *   2:  -6dBm,  9.0mA
 *   1: -12dBm,  7.5mA
 *   0: -18dBm,  7.0mA
 * @retval  None
 * */
void NRF24L01_SetOutputPower(int deviceNum, uint8_t level){
    if(level > 3){
        return; /* Invalid level */
    }
    NRF24L01_WriteReg(deviceNum, W_REGISTER(RF_SETUP), 0b00001001 | (level << 1));
}

void NRF24L01_IRQ(int deviceNum){
    /*
     * TODO: check irq type
     * - TX_DS:
     *   write 1 to TX_DS
     *   release lock
     * - RX_DR:
     *   check length
     *   send receive command
     * - MAX_RT
     *   not been consider at this time
     */
}

void NRF24L01_SPI_IRQ(int deviceNum){
    /*
     * TODO:
     *  Push data to queue
     */
}

/*
 * the buffer size is limited under 32 bytes
 */
void NRF24L01_TransmitPacket(int deviceNum, uint8_t buf[], uint32_t size){
    uint8_t fifo_status, status;
    while(1){
        NRF24L01_ReadReg(deviceNum, R_REGISTER(FIFO_STATUS), &fifo_status);
        if(FIFO_STATUS & 0b00100000){ // TX FIFO full, busy waiting
            continue;
        }

    }
    status = NRF24L01_WriteBuf(deviceNum, W_TX_PAYLOAD, buf, size);
}

void NRF24L01_Transmit(int deviceNum, uint8_t buf[], uint32_t size){
    /*
     * TODO:
     * lock
     */
    while(size > 0){
        if(size > 32){
            NRF24L01_WriteBuf(deviceNum, W_TX_PAYLOAD, buf, 32);
            size -= 32;
            buf += 32;
        }else{
            NRF24L01_WriteBuf(deviceNum, W_TX_PAYLOAD, buf, size);
            size = 0;
        }
    }
}


void NRF24L01_Receive(int deviceNum, uint8_t buf[], uint32_t size){
    /*
     * TODO: receive from queue
     */
}
