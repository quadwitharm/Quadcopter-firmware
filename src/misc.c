#include "main.h"
#include "uart.h"
#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *   where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line){
    /* Infinite loop */
    while (1);
}

#endif

/**
 * @brief  k* I/O is for debugging, not interrupt based.
 */
void kputs(const char *str){
    while(*str){
        UART_send((uint8_t *)str++,1);
    }
}
void kputc(const char c){
    UART_send((uint8_t *)&c,1);
}
void kgets(char buf[],int len){
    int i = 0;
    while(i < len - 1){
        UART_recv((uint8_t *)buf+i,1);
        ++i;
        if(buf[i] == '\n'){
            break;
        }
    }
    buf[i] = '\0';
}
char kgetc(){
    char msg;
    UART_recv((uint8_t *)&msg,1);
    return msg;
}

/**
 * @brief  Interrupt based I/O
 */
void puts(const char *str){
    while(*str){
        send_byte(*str++);
    }
}
void putc(const char c){
    send_byte(c);
}
void gets(char buf[],int len){
    int i = 0;
    while(i < len - 1){
        buf[i] = recv_byte();
        ++i;
        if(buf[i] == '\n'){
            break;
        }
    }
    buf[i] = '\0';
}
char getc(){
    return recv_byte();
}


/**
 * @brief  Print the binary representation for 8, 16, 32 bit data
 */
void printBinary_uint8(uint8_t c){
    for(uint8_t i = 1u << 7; i != 0;i >>= 1){
        kputc( (c & i) ? '1' : '0');
    }
    kputc(' ');
}

void printBinary_uint16(uint16_t c){
    for(uint16_t i = 1u << 15; i != 0;i >>= 1){
        kputc( (c & i) ? '1' : '0');
    }
    kputc(' ');
}

void printBinary_uint32(uint32_t c){
    for(uint32_t i = 1u << 31; i != 0;i >>= 1){
        kputc( (c & i) ? '1' : '0');
    }
    kputc(' ');
}

void printFloat(float a){
    if(a < 0){ kputs("-"); a = -a;}else{kputs(" ");}
    kputs(itoa(a, 10));
    kputs(".");
    kputs( itoa( (100000 * a - 100000 * (int)a ) ,10) );
}

/* TODO: use external buffer */
char *itoa(int num, unsigned int base){
    static char buf[32]={0};
    int i;
    if(num==0){
        buf[30]='0';
        return &buf[30];
    }
    int negative=(num<0);
    if(negative) num=-num;
    for(i=30; i>=0&&num; --i, num/=base)
        buf[i] = "0123456789ABCDEF"[num % base];
    if(negative){
        buf[i]='-';
        --i;
    }
    return buf+i+1;
}
