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

void kputs(const char *str){
    while(*str){
        send_byte(*str++);
    }
}

void kputc(const char c){
    send_byte(c);
}

void kgets(char buf[],int len){
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

char kgetc(){
    return recv_byte();
}
