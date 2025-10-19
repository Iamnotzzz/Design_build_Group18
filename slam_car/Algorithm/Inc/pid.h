#ifndef __PID_H
#define __PID_H

#include "main.h"
#include "filter.h"

typedef struct{
    float sum;
    float sum_max;
    float last_error;
}PID_I_Type;

typedef struct{
    uint8_t use_filter;
    IIRFilter lpf;
    float prev_ef;
    uint8_t inited;
}PID_D_Type;

typedef struct{
    float Kp;
    float Ki;
    float Kd;
    double last_update_time;
    float dead_zone;
    float output;
    float output_max;
    float reference;
    float feedback;
    PID_I_Type I;
    PID_D_Type D;
}PID_Type;


void PID_Filter_Auto_Init(PID_Type *pid, float Kp, float Ki, float Kd, float dead_zone, float output_max, float sum_max, int order, float fs, float fc);
void PID_Filter_UserParameter_Init(PID_Type *pid, float Kp, float Ki, float dead_zone, float Kd, float output_max, float sum_max, int order, const float* b, const float* a);
void PID_Filter_Reset(PID_Type *pid);
float PID_Filter_Calculate(PID_Type *pid, float feedback, float reference);









#endif
