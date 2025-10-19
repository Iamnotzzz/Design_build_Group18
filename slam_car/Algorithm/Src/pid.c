#include "pid.h"
#include "dwt.h"
#include <math.h>

static void I_Init(PID_I_Type *I, float sum_max)
{
    I->last_error = 0;
    I->sum = 0;
    I->sum_max = sum_max;
}

void I_Reset(PID_I_Type *I)
{
    I->last_error = 0;
    I->sum = 0;
}

static float I_Calculate(PID_I_Type *I, float feedback, float reference, float Ki, float dt_s)
{
    float const ei = 0.5f * (reference - feedback + I->last_error);
    I->last_error = reference - feedback;
    
    I->sum += Ki * ei * dt_s;
    
    if(I->sum_max != 0)
    {
        if(I->sum > I->sum_max) 
        {
            I->sum = I->sum_max;
        }
        else if(I->sum < -I->sum_max) 
        {
            I->sum = -I->sum_max;
        }
    }
    
    return I->sum;
}

static void Filter_D_UserParameter_Init(PID_D_Type *D, int order, const float* b, const float* a)
{
    if(order > 0)
    {
        IIRFilter_Init(&D->lpf, order, b, a);
        D->use_filter = 1;
    }
    else D->use_filter = 0;
    D->inited = 0;
    D->prev_ef = 0;
}

static void Filter_D_Auto_Init(PID_D_Type *D, int order, float fs, float fc)
{
    if(fs > 0 && fc > 0)
    {
        int result;
        result = IIRFilter_Auto_Init(&D->lpf, order, fs, fc);
        if(result == 0) D->use_filter = 1;
        else D->use_filter = 0;
    }
    else D->use_filter = 0;
    D->inited = 0;
    D->prev_ef = 0;
}

static void Filter_D_Reset(PID_D_Type *D)
{
    if(D->use_filter) IIRFilter_Reset(&D->lpf);
    D->inited = 0;
    D->prev_ef = 0;
}

static float Filter_D_Calculate(PID_D_Type *D, float feedback, float reference, float Kd, float dt_s)
{
    float ef;
    
    if(D->use_filter) ef = IIRFilter_Calculate(&D->lpf, reference - feedback);
    else ef = reference - feedback;
    
    float deriv = 0.0f;
    
    if (!D->inited) {
        D->inited = 1;
    } else {
        deriv = (ef - D->prev_ef) / dt_s;
    }
    
    D->prev_ef = ef;
    
    return Kd * deriv;
}

void PID_Filter_Auto_Init(PID_Type *pid, float Kp, float Ki, float Kd, float dead_zone, float output_max, float sum_max, int order, float fs, float fc)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->last_update_time = 0;
    pid->dead_zone = dead_zone;
    pid->output = 0;
    pid->output_max = output_max;
    pid->reference = 0;
    pid->feedback = 0;
    I_Init(&pid->I, sum_max);
    Filter_D_Auto_Init(&pid->D, order, fs, fc);
}

void PID_Filter_UserParameter_Init(PID_Type *pid, float Kp, float Ki, float Kd, float dead_zone, float output_max, float sum_max, int order, const float* b, const float* a)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->last_update_time = 0;
    pid->dead_zone = dead_zone;
    pid->output = 0;
    pid->output_max = output_max;
    pid->reference = 0;
    pid->feedback = 0;
    I_Init(&pid->I, sum_max);
    Filter_D_UserParameter_Init(&pid->D, order, b, a);
}

void PID_Filter_Reset(PID_Type *pid)
{
    pid->last_update_time = 0;
    pid->output = 0;
    pid->reference = 0;
    pid->feedback = 0;
    I_Reset(&pid->I);
    Filter_D_Reset(&pid->D);
}

float PID_Filter_Calculate(PID_Type *pid, float feedback, float reference)
{
    pid->reference = reference;
    pid->feedback = feedback;
    
    double current_time_us = DWT_Get_us();
    float dt_s = (float)((current_time_us - pid->last_update_time)/1000000.0);
    if (dt_s < 0.001f || pid->last_update_time == 0) dt_s = 0.001f;
    pid->last_update_time = current_time_us;
    
    if(fabsf(reference - feedback) < pid->dead_zone)
    {
        pid->output = 0;
        I_Calculate(&pid->I, reference, reference, pid->Ki, dt_s);
        Filter_D_Calculate(&pid->D, reference, reference, pid->Kd, dt_s);
    }
    else
    {
        pid->output = pid->Kp * (reference - feedback) + I_Calculate(&pid->I, feedback, reference, pid->Ki, dt_s) + Filter_D_Calculate(&pid->D, feedback, reference, pid->Kd, dt_s);
    }
    if(pid->output_max != 0)
    {
        if(pid->output > pid->output_max) pid->output = pid->output_max;
        else if(pid->output < -pid->output_max) pid->output = -pid->output_max;
    }
    return pid->output;
}


