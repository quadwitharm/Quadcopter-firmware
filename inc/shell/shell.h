#ifndef __SHELL_H__
#define __SHELL_H__

#include "clib.h"

#define STR_CLEAR_SCREEN "\x1b[H\x1b[2J";

bool InitShell();

typedef int (*commandfunc_t)(int,char**);
typedef struct command_t{
    commandfunc_t pFunc;
    const char *name;
    const char *description;
}command_t;

#endif
