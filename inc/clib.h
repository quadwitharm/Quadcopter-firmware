#ifndef __CLIB_H__
#define __CLIB_H__

/*
 * Includes type defines, text I/O operation and some string uilities
 */

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdbool.h>
#include <stdarg.h>
#include "stm32f429i_discovery.h"
#include "FreeRTOS.h"

void assert_failed(uint8_t* file, uint32_t line);

void kputs(const char *);
void kputc(char);
void kgets(char buf[],int len);
char kgetc();
int kprintf(const char *format, ...);

/*
 * Followig function uses library implementation
 * Not include stdio.h because of conflict of function name.
 */
int strcmp ( const char * str1, const char * str2 );
int atoi (const char * str);
void * memcpy ( void * destination, const void * source, size_t num );
size_t strlen ( const char * str );
int vsnprintf (char * s, size_t n, const char * format, va_list arg );


#ifdef __cplusplus
}
#endif

#endif
