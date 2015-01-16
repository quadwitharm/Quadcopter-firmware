#include "shell/textcommand.h"
#include "uart.h"
#include "sensor/sensor.h"

#define BUFSIZE 128
#define ARGV_SIZE 20

#define COMMAND_PROMPT "shell > "
#define MKCL(n, desc) {.pFunc = n ## _command, .name = #n, .description = desc}
command_t CommandList[] = {
    MKCL(help, "Display command reference"),
    MKCL(echo, "Display a line of text"),
    MKCL(current_attitude, "Output filtered roll, pitch, yaw value"),

};
void handleTextCommand(uint8_t *buf){
    uint16_t len = (((uint16_t)buf[1]) << 8) + buf[0];
    char *argv[ARGV_SIZE];
    char *line = (char *)buf + 2;
    (void)len;

    int argc = parseCommand(line, argv);
    commandfunc_t toExec = findCommand(argv[0]);
    if(toExec == NULL){
        kputs("Command not found!\r\n");
    }else{
        toExec(argc,argv);
    }
}

int parseCommand(char *line, char **argv){
    int count = 0;
    bool squote = false, dquote = false;
    char *p = line;
    for(char *cur = line; *cur; ++cur){
        switch(*cur){
            case '\'':
                squote = !squote; break;
            case '"' :
                dquote = !dquote; break;
            case ' ' :
                /* if argument too much, the later text will be last argument */
                if( !squote && !dquote && count < ARGV_SIZE - 1){
                    *cur = '\0';
                    argv[count++] = p;
                    p = cur + 1;
                }
                break;
        }
    }
    argv[count++] = p;
    return count;
}

commandfunc_t findCommand(char *name){
    for(int i = 0;i < sizeof(CommandList) / sizeof(CommandList[0]);++i){
        if(strcmp(CommandList[i].name, name) == 0)
            return CommandList[i].pFunc;
    }
    return NULL;
}

int help_command(int argc, char **argv){
    for(int i = 0;i < sizeof(CommandList) / sizeof(CommandList[0]);++i){
        kprintf("[%s] : %s\r\n", CommandList[i].name, CommandList[i].description);
    }
    return 0;
}

int echo_command(int argc, char **argv){
    for(int i = 1;i < argc;++i){
        kprintf("%s ",argv[i]);
    }
    kprintf("\r\n");
    return 0;
}

extern float mFR,mBL,mFL,mBR;
int current_attitude_command(int argc, char **argv){
    int max = 100; /* default is 100 */
    if(argc == 2) max = atoi(argv[1]);
    for(int i = 0;i < max;++i){
        kprintf("%f,%f,%f,",xAttitude.roll,xAttitude.pitch,xAttitude.yaw);
        kprintf("%f,%f,%f,%f\r\n",mFR,mBL,mFL,mBR);
    }
    return 0;
}
