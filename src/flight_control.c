#include "flight_control.h"




void control_task(void){

    pid_context_t pid_roll,pid_pitch,pid_yaw;
    pid_context_t pid_roll_r,pid_pitch_r,pid_yaw_r;
    
    //pid results
    float roll_out,pitch_out,yaw_out;

    //mortor outputs
    float mFR,mBL,mFL,mBR;

    //init pids
    stablize_pid_init(&roll,&pitch,&yaw);
    rate_pid_init(&roll_r,&pitch_r,&yaw_r);

    while(1){

        //read data


        //calculate PIDs
        roll_out = doPID(&pid_roll,
            doPID(&pid_roll_r,/*rc_roll*/,/*sensor*/),/*sensor*/);
        pitch_out = doPID(&pid_pitch,
            doPID(&pid_pitch_r,/*rc_pitch*/,/*sensor*/),/*sensor*/);

        if(/*rate mode*/){
            yaw_out = doPID(&pid_yaw_r,/**/,/**/);
        }else if(/*stablized mode*/){
        yaw_out = doPID(&pid_yaw,
            doPID(&pid_yaw_r,/*rc_yaw*/,/*sensor*/),/*sensor*/);
        }

        //mortor values
        mFR = + roll_out - pitch_out + yaw_out;
        mFL = - roll_out - pitch_out - yaw_out;
        mBR = + roll_out + pitch_out - yaw_out;
        mBL = - roll_out + pitch_out + yaw_out;


    }


}


