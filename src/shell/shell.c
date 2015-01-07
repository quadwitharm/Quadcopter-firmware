#include "shell/shell.h"
#include "task.h"

#include "sensor/sensor.h"

#define BUFSIZE 128
#define ARGV_SIZE 20

static xTaskHandle shellTaskHandle;

static void ShellTask(void *);
static int parseCommand(char *line, char **argv);
static commandfunc_t findCommand(char *name);

static int help_command(int argc, char **argv);
static int echo_command(int,char**);
static int current_attitude_command(int argc, char **argv);

#define COMMAND_PROMPT "shell > "
#define MKCL(n, desc) {.pFunc = n ## _command, .name = #n, .description = desc}
command_t CommandList[] = {
    MKCL(help, "Display command reference"),
    MKCL(echo, "Display a line of text"),
    MKCL(current_attitude, "Output filtered roll, pitch, yaw value"),
};

bool InitShell(){
    portBASE_TYPE ret = xTaskCreate(ShellTask,
            (portCHAR *)"Shell",
            512,
            NULL,
            tskIDLE_PRIORITY + 1,
            &shellTaskHandle);
    return ret;
}

static void ShellTask(void *args){
    char line[BUFSIZE];
    char *argv[ARGV_SIZE];
    while(1){
        puts( COMMAND_PROMPT );
        gets(line, BUFSIZE);

        int argc = parseCommand(line, argv);
        commandfunc_t toExec = findCommand(argv[0]);
        if(toExec == NULL){
            puts("Command not found!\r\n");
        }else{
            toExec(argc,argv);
        }
    }
}

static int parseCommand(char *line, char **argv){
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

static commandfunc_t findCommand(char *name){
    for(int i = 0;i < sizeof(CommandList) / sizeof(CommandList[0]);++i){
        if(strcmp(CommandList[i].name, name) == 0)
            return CommandList[i].pFunc;
    }
    return NULL;
}

static int help_command(int argc, char **argv){
    for(int i = 0;i < sizeof(CommandList) / sizeof(CommandList[0]);++i){
        printf("%s : %s\r\n", CommandList[i].name, CommandList[i].description);
    }
    return 0;
}

static int echo_command(int argc, char **argv){
    for(int i = 1;i < argc;++i){
        printf("[%s] ",argv[i]);
    }
    printf("\r\n");
    return 0;
}

static int current_attitude_command(int argc, char **argv){
    int max = 100; /* default is 100 */
    if(argc == 2) max = atoi(argv[1]);
    for(int i = 0;i < max;++i){
        kprintf("%f,%f,%f\r\n",xAttitude.roll,xAttitude.pitch,xAttitude.yaw);
        vTaskDelay(20);
    }
    return 0;
}
