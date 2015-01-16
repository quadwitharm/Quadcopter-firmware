#include "shell/shell.h"
#include "shell/textcommand.h"
#include "task.h"

#include "sensor/sensor.h"
#include "uart.h"
#include "controller/control_api.h"

static xTaskHandle shellTaskHandle;

static void ShellTask(void *);
static void handlePID();
static void handleChangeSetPoint();

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
    //puts("Welcome to quadcopter shell!\r\n");
    while(1){
        //gets(line, BUFSIZE);
        UART_recv_IT(&type,1);//get command type

        switch(type){
            case 0x0:
                setControllerEnable(false);
                break;
            case 0x1:
                setControllerEnable(true);
                break;
            case 0x2:
            case 0x3:break;
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
}

