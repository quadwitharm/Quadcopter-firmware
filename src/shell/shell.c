#include "shell/shell.h"
#include "shell/textcommand.h"
#include "shell/b64.h"
#include "task.h"

#include "sensor/sensor.h"
#include "uart.h"
#include "controller/control_api.h"

#define BUFSIZE 128
#define ARGV_SIZE 20
static xTaskHandle shellTaskHandle;

static void ShellTask(void *);

static void handlePID(uint8_t *buf);
static void handleChangeSetPoint(uint8_t *buf);

bool InitShell(){
    portBASE_TYPE ret = xTaskCreate(ShellTask,
            (portCHAR *)"Shell",
            512,
            NULL,
            tskIDLE_PRIORITY + 5,
            &shellTaskHandle);
    return ret;
}

static void ShellTask(void *args){
    uint8_t buf[64]; // A size enough for all command
    kputs("Shell Enabled.");
    while(1){
        uint8_t *cur = buf;
        do{
            UART_recv_IT(cur,1);
        }while(*cur++ != (uint8_t)0x86);
        int len = cur - buf - 1;

        int outlen = getB64DecodeLen(len);
        uint8_t cmdbuf[outlen];
        if(b64Decode(buf,cmdbuf,len)){
            uint8_t checksum = 0;
            for(int i = 0;i < outlen - 1;++i){
                checksum += cmdbuf[i];
            }
            if( checksum == cmdbuf[outlen - 1]){
                switch(cmdbuf[0]){
                    case 0x0:
                        setControllerEnable(false);
                        break;
                    case 0x1:
                        setControllerEnable(true);
                        break;
                    case 0x2:
                        SensorEnable(false);
                        setControllerEnable(false);
                        break;
                    case 0x3:
                        SensorEnable(true);
                        break;
                    case 0x4:
                        handlePID(&cmdbuf[1]);
                        break;
                    case 0x5:
                        handleChangeSetPoint(&cmdbuf[1]);
                        break;
                    case 0x6:
                        handleTextCommand(&cmdbuf[1]);
                        break;
                }
                kprintf("Handled command: %d\r\n",cmdbuf[0]);
            }else{
                kprintf("Command checksum not match!\r\n");
            }
        }else{
            kprintf("Base64 Decode failed!\r\n");
        }
    }
}

static void handlePID(uint8_t *buf){
    uint8_t which = buf[0];
    float pid[3];

    memcpy(pid,buf+1,sizeof(pid));

    setPidParameter(which,KP,pid[0]);
    setPidParameter(which,KI,pid[1]);
    setPidParameter(which,KD,pid[2]);
    kprintf("%f %f %f",pid[0],pid[1],pid[2]);
}

static void handleChangeSetPoint(uint8_t *buf){
    uint8_t op = buf[0];
    float set[3];

    if(op == 0x0){
        memcpy(set,buf+1,sizeof(float) * 3);
        setSetPoint(ROLL_C,set[0]);
        setSetPoint(PITCH_C,set[1]);
        setSetPoint(YAW_C,set[2]);
    }else if(op == 0x1){
        memcpy(set,buf+1,sizeof(float));
        setSetPoint(THR_C,set[0]);
    }
    kprintf("%f %f %f",set[0],set[1],set[2]);
}

