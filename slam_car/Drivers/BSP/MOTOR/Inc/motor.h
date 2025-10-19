#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include "main.h"
#include "tim.h"
#include "pid.h"

#define motor_htim      htim3
#define encoder1_htim   htim5
#define encoder2_htim   htim4

#define AIN1_channel    TIM_CHANNEL_1
#define AIN2_channel    TIM_CHANNEL_2
#define BIN1_channel    TIM_CHANNEL_3
#define BIN2_channel    TIM_CHANNEL_4

typedef struct{
    int encoder_total;
    int encoder;
    int encoder_speed;
    float target_speed;
    float speed;
    double last_update_time;
    float output;
    PID_Type *location_pid;
    PID_Type *speed_pid;
} MOTOR_Struct;

void motor_enable(void);
void motor_disable(void);
void set_motor_speed(float left_speed, float right_speed);


// 电机数据
#define motor_diameter              68.0f      // mm
#define motor_ratio                 225.0f     // *4，减速比为56.25
#define huoer_pre                   13.0f      // 霍尔编码器分辨率13

#define motor_static_speed          0.001f

// 左轮
#define L_location_Kp               0.0f;
#define L_location_Ki               0.0f;
#define L_location_Kd               0.0f;
#define L_location_output_max       0.0f;
#define L_location_sum_max          0.0f;
#define L_location_order            0;
#define L_location_fs               0.0f;
#define L_location_fc               0.0f;

#define L_speed_Kp                  0.0f;
#define L_speed_Ki                  0.0f;
#define L_speed_Kd                  0.0f;
#define L_speed_output_max          0.0f;
#define L_speed_sum_max             0.0f;
#define L_speed_order               0;
#define L_speed_fs                  0.0f;
#define L_speed_fc                  0.0f;

// 右轮
#define R_location_Kp               0.0f;
#define R_location_Ki               0.0f;
#define R_location_Kd               0.0f;
#define R_location_output_max       0.0f;
#define R_location_sum_max          0.0f;
#define R_location_order            0;
#define R_location_fs               0.0f;
#define R_location_fc               0.0f;

#define R_speed_Kp                  0.0f;
#define R_speed_Ki                  0.0f;
#define R_speed_Kd                  0.0f;
#define R_speed_output_max          0.0f;
#define R_speed_sum_max             0.0f;
#define R_speed_order               0;
#define R_speed_fs                  0.0f;
#define R_speed_fc                  0.0f;


extern MOTOR_Struct l_motor;
extern MOTOR_Struct r_motor;

void Motor_init_base(MOTOR_Struct *motor, PID_Type *location_pid, PID_Type *speed_pid);
void motor_init(void);
void motor_update(void);
void motor_location_pid_cal(MOTOR_Struct *motor, float ref);
void motor_speed_pid_cal(MOTOR_Struct *motor);
void motor_angle_pid_cal(float ref);
void motor_speed_rate_diff_pid_cal(int l_motor_encoder_begin, int r_motor_encoder_begin, float rate);
void motor_output(void);

#endif
