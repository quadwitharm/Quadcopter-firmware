#ifndef __FILTER_H__
#define __FILTER_H__

struct KalmanParameter{
    float angle;
    float bias;
    float rate;
    float Q_angle;
    float Q_bias;
    float R_measure;
    float P[2][2];
};

struct Angle3D ComplementaryFilter(struct Angle3D *, struct Angle3D *);
float KalmanFilter(struct KalmanParameter *, float, float, float);

#endif
