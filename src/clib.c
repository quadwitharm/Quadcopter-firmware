#include "clib.h"

#include <stdarg.h>

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

/*
 * Not include stdio.h because of conflict of function name.
 */
int vsnprintf (char * s, size_t n, const char * format, va_list arg );

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
int kprintf(const char *format, ...){
    char outbuf[128];
    va_list args;
    va_start(args,format);
    int ret = vsnprintf(outbuf,128,format,args);
    kputs(outbuf);
    va_end(args);
    return ret;
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
int printf(const char *format, ...){
    char outbuf[128];
    va_list args;
    va_start(args,format);
    int ret = vsnprintf(outbuf,128,format,args);
    puts(outbuf);
    va_end(args);
    return ret;
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
