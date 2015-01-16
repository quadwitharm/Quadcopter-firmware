#ifndef __TEXTCOMMAND_H__
#define __TEXTCOMMAND_H__

// Unused in this time

#include "clib.h"

#define STR_CLEAR_SCREEN "\x1b[H\x1b[2J";

typedef int (*commandfunc_t)(int,char**);
typedef struct command_t{
    commandfunc_t pFunc;
    const char *name;
    const char *description;
}command_t;

int parseCommand(char *line, char **argv);
commandfunc_t findCommand(char *name);

int help_command(int argc, char **argv);
int echo_command(int,char**);
int current_attitude_command(int argc, char **argv);
void handleTextCommand();


#endif
