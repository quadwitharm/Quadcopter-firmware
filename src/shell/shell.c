#include "shell/shell.h"
#include "task.h"

#include "sensor/sensor.h"
#include "uart.h"
#include "controller/control_api.h"

#define BUFSIZE 128
#define ARGV_SIZE 20

static xTaskHandle shellTaskHandle;

static void ShellTask(void *);
static void handlePID();
static void handleChangeSetPoint();
static void handleTextCommand();
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
    uint8_t type;
    // Interrupt based I/O
    //puts("Welcome to quadcopter shell!\r\n");
    while(1){
        //puts( COMMAND_PROMPT );
        //gets(line, BUFSIZE);
        UART_recv_IT(&type,1);//get command type

        switch(type){
            case 0x0:
                setControllerEnable(false);
                //UART_send_IT((uint8_t []){0x00,0x00},2);
                break;
            case 0x1:
                setControllerEnable(true);
                //UART_send_IT((uint8_t []){0x00,0x01},2);
                break;
            case 0x2:
                SensorEnable(false);
                setControllerEnable(false);
                break;
            case 0x3:
                SensorEnable(true);
                break;
            case 0x4:
                handlePID();
                break;
            case 0x5:
                handleChangeSetPoint();
                break;
            case 0x6:
                handleTextCommand();
                break;
        }
    }
}

static void handlePID(){
    uint8_t which;
    float buf[3];

    UART_recv_IT(&which,1);
    UART_recv_IT((uint8_t *)buf,12);
    setPidParameter(which,KP,buf[0]);
    setPidParameter(which,KI,buf[1]);
    setPidParameter(which,KD,buf[2]);
    //UART_send_IT((uint8_t []){0x00,0x04},2);
}

static void handleChangeSetPoint(){
    uint8_t op;
    float buf[3];

    UART_recv_IT(&op,1);
    if(op == 0x0){
        UART_recv_IT((uint8_t *)buf,12);
        setSetPoint(ROLL_C,buf[0]);
        setSetPoint(PITCH_C,buf[1]);
        setSetPoint(YAW_C,buf[2]);
    }else if(op == 0x1){
        UART_recv_IT((uint8_t *)buf,4);
        setSetPoint(THR_C,buf[0]);
    }else{
        
    }
    //UART_send_IT((uint8_t []){0x00,0x05},2);
}   

static void handleTextCommand(){
    uint16_t len;
    char line[BUFSIZE];
    char *argv[ARGV_SIZE];
    
    UART_recv_IT((uint8_t*)&len,2);
    UART_recv_IT((uint8_t*)line,len);

    int argc = parseCommand(line, argv);
    commandfunc_t toExec = findCommand(argv[0]);
    if(toExec == NULL){
        puts("Command not found!\r\n");
    }else{
        toExec(argc,argv);
    }
    //UART_send_IT((uint8_t []){0x00,0x06},2);
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
        printf("[%s] : %s\r\n", CommandList[i].name, CommandList[i].description);
    }
    return 0;
}

static int echo_command(int argc, char **argv){
    for(int i = 1;i < argc;++i){
        printf("%s ",argv[i]);
    }
    printf("\r\n");
    return 0;
}

extern float mFR,mBL,mFL,mBR;
static int current_attitude_command(int argc, char **argv){
    int max = 100; /* default is 100 */
    if(argc == 2) max = atoi(argv[1]);
    for(int i = 0;i < max;++i){
        kprintf("%f,%f,%f,",xAttitude.roll,xAttitude.pitch,xAttitude.yaw);
        kprintf("%f,%f,%f,%f\r\n",mFR,mBL,mFL,mBR);
        vTaskDelay(20);
    }
    return 0;
}
