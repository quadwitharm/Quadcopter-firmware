#include "flight_control.h"





void control_task(void){

    pid_context_t pid_roll,pid_pitch,pid_yaw;
    pid_context_t pid_roll_r,pid_pitch_r,pid_yaw_r;
    


    //init stuff here
    stablize_pid_init(&roll,&pitch,&yaw);
    rate_pid_init(&roll_r,&pitch_r,&yaw_r);

    while(1){

        //read data


        doPID(&pid_roll,doPID(&pid_roll_r,/*rc_roll*/,/*sensor*/),/*sensor*/);
        doPID(&pid_pitch,doPID(&pid_pitch_r,/*rc_pitch*/,/*sensor*/),/*sensor*/);
        doPID(&pid_yaw,/**/,/**/);





    }


}


