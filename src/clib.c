#include "clib.h"

#include <stdarg.h>

#include "uart.h"
#include "shell/send.h"
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

void kputs(const char *str){
    const uint8_t head = 0x03;
    int len = 0;
    while(str[len]) len++;
    SendCommand_2(head, (uint8_t *)str, len);
}
void kputc(const char c){
    const uint8_t head = 0x03;
    SendCommand_2(head, (uint8_t *)&c, 1);
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

