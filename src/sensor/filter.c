#include "sensor/filter.h"
#include "sensor/sensor.h"

#define MAX(a,b) ( ((a) > (b)) ? (a) : (b) )
#define MIN(a,b) ( ((a) > (b)) ? (b) : (a) )

struct Angle3D ComplementaryFilter(struct Angle3D *gyro,struct Angle3D *fix){
    struct Angle3D ret;
    float a = gyro->roll, b = fix->roll;
    if( MAX(a,b) - MIN(a,b) < MIN(a,b)+360 - MAX(a,b) ){
        ret.roll = gyro -> roll * 0.99 + fix -> roll * 0.01;
    }else{
        if(gyro->roll > fix->roll)
            ret.roll = gyro -> roll * 0.99 + (360 + fix -> roll) * 0.01;
        else
            ret.roll = (360 + gyro -> roll) * 0.99 + fix -> roll * 0.01;
    }

    a = gyro->pitch, b = fix->pitch;
    if( MAX(a,b) - MIN(a,b) < MIN(a,b)+360 - MAX(a,b) ){
        ret.pitch = gyro -> pitch * 0.99 + fix -> pitch * 0.01;
    }else{
        if(gyro->pitch > fix->pitch)
            ret.pitch = gyro -> pitch * 0.99 + (360 + fix -> pitch) * 0.01;
        else
            ret.pitch = (360 + gyro -> pitch) * 0.99 + fix -> pitch * 0.01;
    }

    a = gyro->yaw, b = fix->yaw;
    if( MAX(a,b) - MIN(a,b) < MIN(a,b)+360 - MAX(a,b) ){
        ret.yaw = gyro -> yaw * 0.99 + fix -> yaw * 0.01;
    }else{
        if(gyro->yaw > fix->yaw)
            ret.yaw = gyro -> yaw * 0.99 + (360 + fix -> yaw) * 0.01;
        else
            ret.yaw = (360 + gyro -> yaw) * 0.99 + fix -> yaw * 0.01;
    }
    if(ret.roll  > 180)ret.roll  -= 360;
    if(ret.pitch > 180)ret.pitch -= 360;
    if(ret.yaw   > 180)ret.yaw   -= 360;
    if(ret.roll  < -180)ret.roll  += 360;
    if(ret.pitch < -180)ret.pitch += 360;
    if(ret.yaw   < -180)ret.yaw   += 360;
    return ret;
}

float KalmanFilter(struct KalmanParameter *K, float angle, float dr, float dt){

    /* Step 1 */
    K -> rate = dr - K -> bias;
    K -> angle += dt * K -> rate;

    /* Step 2: Update estimation error covariance */
    K -> P[0][0] += dt * (dt * K -> P[1][1] - K -> P[0][1] - K -> P[1][0] + K -> Q_angle);
    K -> P[0][1] -= dt * K -> P[1][1];
    K -> P[1][0] -= dt * K -> P[1][1];
    K -> P[1][1] += K -> Q_bias * dt;

    /* Step 3: Calculate angle and bias */
    float y = angle - K -> angle;

    /* Step 4: Calculate Kalman gain */
    float S = K -> P[0][0] + K -> R_measure;

    /* Step 5 */
    float _k[2];
    _k[0] = K -> P[0][0] / S;
    _k[1] = K -> P[1][1] / S;

    /* Step 6:  */
    K -> angle += _k[0] * y;
    K -> bias += _k[1] * y;

    /* Step 7: Calculate estimation error covariance */
    float P00 = K -> P[0][0];
    float P01 = K -> P[0][1];

    K -> P[0][0] -= _k[0] * P00;
    K -> P[0][1] -= _k[0] * P01;
    K -> P[1][0] -= _k[1] * P00;
    K -> P[1][1] -= _k[1] * P01;
    
    return K -> angle;
}
