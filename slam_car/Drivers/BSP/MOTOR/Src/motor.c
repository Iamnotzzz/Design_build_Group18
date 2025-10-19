#include "motor.h"
#include "dwt.h"
#include "NVIC_Callback.h"
#include "mpu6500_driver.h"

static void MOTOR_SetPulse(uint32_t channel, uint32_t pulse)
{
    switch(channel)
    {
        case TIM_CHANNEL_1:
            __HAL_TIM_SET_COMPARE(&motor_htim, TIM_CHANNEL_1, pulse);
            break;
        case TIM_CHANNEL_2:
            __HAL_TIM_SET_COMPARE(&motor_htim, TIM_CHANNEL_2, pulse);
            break;
        case TIM_CHANNEL_3:
            __HAL_TIM_SET_COMPARE(&motor_htim, TIM_CHANNEL_3, pulse);
            break;
        case TIM_CHANNEL_4:
            __HAL_TIM_SET_COMPARE(&motor_htim, TIM_CHANNEL_4, pulse);
            break;
        default:
            // 错误处理
            break;
    }
}

static uint32_t speed_to_pulse(float speed)
{
    // 限幅速度
    if(speed > 1.0f) speed = 1.0f;
    if(speed < -1.0f) speed = -1.0f;

    // 读取 TIM 周期
    uint32_t tim_period = motor_htim.Init.Period;

    // 映射到 0~Period
    return (uint32_t)(fabsf(speed) * tim_period);
}

void motor_enable(void)
{
    HAL_TIM_PWM_Start(&motor_htim, AIN1_channel);  // CH1
    HAL_TIM_PWM_Start(&motor_htim, AIN2_channel);  // CH2
    HAL_TIM_PWM_Start(&motor_htim, BIN1_channel);  // CH3
    HAL_TIM_PWM_Start(&motor_htim, BIN2_channel);  // CH4
    HAL_TIM_Encoder_Start(&encoder1_htim, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&encoder2_htim, TIM_CHANNEL_ALL);
    MOTOR_SetPulse(AIN1_channel, 0);
    MOTOR_SetPulse(AIN2_channel, 0);
    MOTOR_SetPulse(BIN1_channel, 0);
    MOTOR_SetPulse(BIN2_channel, 0);
}

void motor_disable(void)
{
    MOTOR_SetPulse(AIN1_channel, 0);
    MOTOR_SetPulse(AIN2_channel, 0);
    MOTOR_SetPulse(BIN1_channel, 0);
    MOTOR_SetPulse(BIN2_channel, 0);
    
    HAL_TIM_PWM_Stop(&motor_htim, AIN1_channel);
    HAL_TIM_PWM_Stop(&motor_htim, AIN2_channel);
    HAL_TIM_PWM_Stop(&motor_htim, BIN1_channel);
    HAL_TIM_PWM_Stop(&motor_htim, BIN2_channel);
}

void set_motor_speed(float left_speed, float right_speed)
{
    if(left_speed <= 0)
    {
        MOTOR_SetPulse(AIN1_channel, speed_to_pulse(-left_speed));
        MOTOR_SetPulse(AIN2_channel, 0);
    }
    else
    {
        MOTOR_SetPulse(AIN1_channel, 0);
        MOTOR_SetPulse(AIN2_channel, speed_to_pulse(left_speed));
    }
    
    if(right_speed >= 0)
    {
        MOTOR_SetPulse(BIN1_channel, speed_to_pulse(right_speed));
        MOTOR_SetPulse(BIN2_channel, 0);
    }
    else
    {
        MOTOR_SetPulse(BIN1_channel, 0);
        MOTOR_SetPulse(BIN2_channel, speed_to_pulse(-right_speed));
    }
}


// 角度
PID_Type angle_pid;
float angle_Kp            =   0.0005f;
float angle_Ki            =   0.0001f;
float angle_Kd            =   0.0f;
float angle_dead_zone     =   0.0f;
float angle_output_max    =   0.15f;
float angle_sum_max       =   0.001f;
int   angle_order         =   0;
float angle_fs            =   0.0f;
float angle_fc            =   0.0f;

// 差速补足
PID_Type speed_diff_pid;
float speed_diff_Kp            =   0.0005f;
float speed_diff_Ki            =   0.0001f;
float speed_diff_Kd            =   0.0f;
float speed_diff_dead_zone     =   0.0f;
float speed_diff_output_max    =   0.15f;
float speed_diff_sum_max       =   0.001f;
int   speed_diff_order         =   0;
float speed_diff_fs            =   0.0f;
float speed_diff_fc            =   0.0f;

// 左轮
MOTOR_Struct l_motor;
PID_Type l_location_pid;
float l_location_Kp            =   0.00006f;
float l_location_Ki            =   0.00001f;
float l_location_Kd            =   0.0f;
float l_location_dead_zone     =   0.0f;
float l_location_output_max    =   0.15f;
float l_location_sum_max       =   0.001f;
int   l_location_order         =   0;
float l_location_fs            =   0.0f;
float l_location_fc            =   0.0f;

PID_Type l_speed_pid;
float l_speed_Kp               =   15.0f;
float l_speed_Ki               =   20.0f;
float l_speed_Kd               =   0.0f;
float l_speed_dead_zone        =   0.0f;
float l_speed_output_max       =   1.0f;
float l_speed_sum_max          =   1.0f;
int   l_speed_order            =   0;
float l_speed_fs               =   0.0f;
float l_speed_fc               =   0.0f;

// 右轮
MOTOR_Struct r_motor;
PID_Type r_location_pid;
float r_location_Kp            =   0.00006f;
float r_location_Ki            =   0.00001f;
float r_location_Kd            =   0.0f;
float r_location_dead_zone     =   0.0f;
float r_location_output_max    =   0.15f;
float r_location_sum_max       =   0.001f;
int   r_location_order         =   0;
float r_location_fs            =   0.0f;
float r_location_fc            =   0.0f;

PID_Type r_speed_pid;
float r_speed_Kp               =   15.0f;
float r_speed_Ki               =   20.0f;
float r_speed_Kd               =   0.0f;
float r_speed_dead_zone        =   0.0f;
float r_speed_output_max       =   1.0f;
float r_speed_sum_max          =   1.0f;
int   r_speed_order            =   0;
float r_speed_fs               =   0.0f;
float r_speed_fc               =   0.0f;


void Motor_init_base(MOTOR_Struct *motor, PID_Type *location_pid, PID_Type *speed_pid)
{
    motor->encoder_total = 0;
    motor->encoder = 0;
    motor->speed = 0;
    motor->last_update_time = DWT_Get_us();
    motor->location_pid = location_pid;
    motor->speed_pid = speed_pid;
}

void motor_init(void)
{
    PID_Filter_Auto_Init(&l_location_pid, l_location_Kp, l_location_Ki, l_location_Kd, l_location_dead_zone, l_location_output_max, l_location_sum_max, l_location_order, l_location_fs, l_location_fc);
    PID_Filter_Auto_Init(&r_location_pid, r_location_Kp, r_location_Ki, r_location_Kd, r_location_dead_zone, r_location_output_max, r_location_sum_max, r_location_order, r_location_fs, r_location_fc);
    
    PID_Filter_Auto_Init(&l_speed_pid, l_speed_Kp, l_speed_Ki, l_speed_Kd, l_speed_dead_zone, l_speed_output_max, l_speed_sum_max, l_speed_order, l_speed_fs, l_speed_fc);
    PID_Filter_Auto_Init(&r_speed_pid, r_speed_Kp, r_speed_Ki, r_speed_Kd, r_speed_dead_zone, r_speed_output_max, r_speed_sum_max, r_speed_order, r_speed_fs, r_speed_fc);
    
    PID_Filter_Auto_Init(&angle_pid, angle_Kp, angle_Ki, angle_Kd, angle_dead_zone, angle_output_max, angle_sum_max, angle_order, angle_fs, angle_fc);
    PID_Filter_Auto_Init(&speed_diff_pid, speed_diff_Kp, speed_diff_Ki, speed_diff_Kd, speed_diff_dead_zone, speed_diff_output_max, speed_diff_sum_max, speed_diff_order, speed_diff_fs, speed_diff_fc);
    
    Motor_init_base(&l_motor, &l_location_pid, &l_speed_pid);
    Motor_init_base(&r_motor, &r_location_pid, &r_speed_pid);
}

void motor_update(void)
{
    l_motor.encoder_speed = encoder1_count - l_motor.encoder_total;
    r_motor.encoder_speed = encoder2_count - r_motor.encoder_total;
    l_motor.encoder += l_motor.encoder_speed;
    r_motor.encoder += r_motor.encoder_speed;
    
    double current_time = DWT_Get_us();
    l_motor.speed = (float)(l_motor.encoder_speed * motor_diameter) / motor_ratio / huoer_pre / (float)(current_time - l_motor.last_update_time) * 1000.0f;     // * 2 * PI
    r_motor.speed = (float)(r_motor.encoder_speed * motor_diameter) / motor_ratio / huoer_pre / (float)(current_time - r_motor.last_update_time) * 1000.0f;     // * 2 * PI
    l_motor.encoder_total = encoder1_count;
    r_motor.encoder_total = encoder2_count;
    
    l_motor.last_update_time = current_time;
    r_motor.last_update_time = current_time;
}

void motor_location_pid_cal(MOTOR_Struct *motor, float ref)
{
    motor->target_speed = PID_Filter_Calculate(motor->location_pid, motor->encoder, ref);
}

void motor_speed_pid_cal(MOTOR_Struct *motor)
{
    motor->output = PID_Filter_Calculate(motor->speed_pid, motor->speed, motor->target_speed);
}

void motor_angle_pid_cal(float ref)
{
    float fdb = mpu6500_data.yaw;
    float error = ref - fdb;
    while (error > 180.0f) 
    {
        ref -= 360.0f;
        error -= 360.0f;
    }
    while (error < -180.0f) 
    {
        ref += 360.0f;
        error += 360.0f;
    }
    
    if(error > 0)
    {
        r_motor.target_speed = PID_Filter_Calculate(&angle_pid, fdb, ref);
        l_motor.target_speed = -PID_Filter_Calculate(&angle_pid, fdb, ref);
    }
    else
    {
        r_motor.target_speed = PID_Filter_Calculate(&angle_pid, fdb, ref);
        l_motor.target_speed = -PID_Filter_Calculate(&angle_pid, fdb, ref);
    }
}

// rate为左轮速度比右轮速度
void motor_speed_rate_diff_pid_cal(int l_motor_encoder_begin, int r_motor_encoder_begin, float rate)
{
    int l_motor_encoder_diff = l_motor.encoder - l_motor_encoder_begin;
    int r_motor_encoder_diff = r_motor.encoder - r_motor_encoder_begin;
    int fdb = rate * r_motor_encoder_diff - l_motor_encoder_diff;
    float speed_error = PID_Filter_Calculate(&speed_diff_pid, fdb, 0.0f);
    
    r_motor.target_speed += speed_error / rate;
    l_motor.target_speed -= speed_error;
}

void motor_output(void)
{
    set_motor_speed(l_motor.output, r_motor.output);
}


