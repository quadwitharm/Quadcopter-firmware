#ifndef __CLIB_H__
#define __CLIB_H__

/*
 * 
 */

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdbool.h>
#include "stm32f429i_discovery.h"
#include "FreeRTOS.h"

void assert_failed(uint8_t* file, uint32_t line);


void kputs(const char *);
void kputc(char);
void kgets(char buf[],int len);
char kgetc();
int kprintf(const char *format, ...);

void puts(const char *str);
void putc(const char c);
void gets(char buf[],int len);
char getc();
int printf(const char *format, ...);

void printBinary_uint8(uint8_t c);
void printBinary_uint16(uint16_t c);
void printBinary_uint32(uint32_t c);

/* Followig function uses library implementation */
int strcmp ( const char * str1, const char * str2 );
int atoi (const char * str);
void * memcpy ( void * destination, const void * source, size_t num );

#ifdef __cplusplus
}
#endif

#endif
