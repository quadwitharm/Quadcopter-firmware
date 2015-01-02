#include "flight_control.h"




void control_task(void){

    pid_context_t pid_roll,pid_pitch,pid_yaw;
    pid_context_t pid_roll_r,pid_pitch_r,pid_yaw_r;
    pid_context_t pid_atti;
    
    //pid results
    float roll_out,pitch_out,yaw_out,atti_out;

    //mortor outputs
    float mFR,mBL,mFL,mBR;

    //init pids
    stablize_pid_init(&roll,&pitch,&yaw);
    rate_pid_init(&roll_r,&pitch_r,&yaw_r);

    while(1){

        //read data



        if(/*raw mode*/){
            //use rc command directly
            //a rate mode maybe?
        }else if(/*stablized mode*/){
            atti_out = runPID(&pid_atti,/**/,/**/);
        }

        //calculate rotational PIDs
        roll_out = runPID(&pid_roll,
            runPID(&pid_roll_r,/*rc_roll*/,/*sensor*/),/*sensor*/);
        pitch_out = runPID(&pid_pitch,
            runPID(&pid_pitch_r,/*rc_pitch*/,/*sensor*/),/*sensor*/);

        if(/*rate mode*/){
            yaw_out = runPID(&pid_yaw_r,/**/,/**/);
        }else if(/*stablized mode*/){
            yaw_out = runPID_warp(&pid_yaw,
                runPID(&pid_yaw_r,/*rc_yaw*/,/*sensor*/),
                /*sensor*/,180.0,-180.0);
        }

        //mortor values
        mFR = atti_out + roll_out - pitch_out + yaw_out;
        mFL = atti_out - roll_out - pitch_out - yaw_out;
        mBR = atti_out + roll_out + pitch_out - yaw_out;
        mBL = atti_out - roll_out + pitch_out + yaw_out;


    }


}


