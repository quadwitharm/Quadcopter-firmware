#ifndef __MAIN_H__
#define __MAIN_H__

/*
 * 
 */

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdbool.h>
#include "stm32f429i_discovery.h"
#include "FreeRTOS.h"

// misc.c
void assert_failed(uint8_t* file, uint32_t line);
void kputs(const char *);
void kputc(char);
void kgets(char buf[],int len);
char kgetc();
void printBinary_uint8(uint8_t c);
void printBinary_uint16(uint16_t c);
void printBinary_uint32(uint32_t c);
void printFloat(float a);
char *itoa(int num, unsigned int base);
void printFloat(float a);

#ifdef __cplusplus
}
#endif

#endif
