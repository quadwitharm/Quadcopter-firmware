#ifndef __MAIN_H__
#define __MAIN_H__

/*
 * 
 */

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f429i_discovery.h"
// misc.c
void assert_failed(uint8_t* file, uint32_t line);
void kputs(const char *);
void kgets(char buf[],int len);
char kgetc();

#ifdef __cplusplus
}

#endif
#endif
