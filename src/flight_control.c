#include "flight_control.h"





void control_task(void){

    pid_context_t pid_roll,pid_pitch,pid_yaw;
    pid_context_t pid_rollrate,pid_pitchrate,pid_yawrate;
    


    //init stuff here

    while(1){

        //read data


        doPID(&pid_roll,doPID(&pid_rollrate,/*troll*/,/*roll info*/),/**/);
        doPID(&pid_pitchrate,doPID(&pid_roll,/*tpitch*/,/*roll info*/),/**/);
        doPID(&pid_yaw,/**/,/**/);





    }


}


