#include "FSM.h"
#include "dwt.h"
#include "mpu6500_driver.h"
#include "motor.h"
#include <math.h>
#include <stdlib.h>

Car_Struct car_state;

void Car_FSM_Init(void)
{
    car_state.last_stop_time_ms = DWT_Get_ms();
    car_state.State = STOP;
    car_state.cmd_id = 0;
    car_state.cmd_complete = 0;
    
    car_state.turn_data.yaw_begin = 0;
    car_state.turn_data.yaw_target = 0;
    car_state.turn_data.angle_change = 0;
    car_state.turn_data.finish_state_dead_zone = TURN_ANGLE_DEAD_ZONE;
    car_state.turn_data.state_close_finish_time_ms = 0;
    car_state.turn_data.state_start_time_ms = 0;
    car_state.turn_data.l_motor_encoder_start = 0;
    car_state.turn_data.r_motor_encoder_start = 0;
    
    car_state.straight_data.straight_distance_encoder = 0;
    car_state.straight_data.straight_distance_m = 0;
    car_state.straight_data.finish_state_dead_zone = STAIGHT_DISTANCE_DEAD_ZONE;
    car_state.straight_data.state_close_finish_time_ms = 0;
    car_state.straight_data.state_start_time_ms = 0;
    car_state.straight_data.l_motor_encoder_start = 0;
    car_state.straight_data.r_motor_encoder_start = 0;
    car_state.straight_data.l_motor_encoder_target = 0;
    car_state.straight_data.r_motor_encoder_target = 0;
}

void Car_FSM_Change(Car_State State)
{
    switch(State)
    {
        case STOP:
            l_motor.target_speed = 0;
            motor_speed_pid_cal(&l_motor);
            r_motor.target_speed = 0;
            motor_speed_pid_cal(&r_motor);
            motor_output();
            car_state.last_stop_time_ms = DWT_Get_ms();
            
            car_state.State = STOP;
            break;
        case TURN:
            car_state.turn_data.l_motor_encoder_start = l_motor.encoder;
            car_state.turn_data.r_motor_encoder_start = r_motor.encoder;
            car_state.turn_data.state_start_time_ms = DWT_Get_ms();
            car_state.turn_data.state_close_finish_time_ms = car_state.turn_data.state_start_time_ms;
            car_state.turn_data.yaw_begin = mpu6500_data.yaw;
            car_state.turn_data.yaw_target = car_state.turn_data.yaw_begin + car_state.turn_data.angle_change;
            while (car_state.turn_data.yaw_target > 180.0f) car_state.turn_data.yaw_target -= 360.0f;
            while (car_state.turn_data.yaw_target < -180.0f) car_state.turn_data.yaw_target += 360.0f;
            
            car_state.State = TURN;
            break;
        case STRAIGHT:
            car_state.straight_data.l_motor_encoder_start = l_motor.encoder;
            car_state.straight_data.r_motor_encoder_start = r_motor.encoder;
            
            car_state.straight_data.l_motor_encoder_target = car_state.straight_data.l_motor_encoder_start + car_state.straight_data.straight_distance_encoder;
            car_state.straight_data.r_motor_encoder_target = car_state.straight_data.r_motor_encoder_start + car_state.straight_data.straight_distance_encoder;
            
            car_state.straight_data.state_start_time_ms = DWT_Get_ms();
            car_state.straight_data.state_close_finish_time_ms = car_state.straight_data.state_start_time_ms;
            
            car_state.State = STRAIGHT;
            break;
    }
}

static void Car_FSM_Set_TURN(float target_angle_change)
{
    car_state.turn_data.angle_change = target_angle_change;
}

static void Car_FSM_Set_STRAIGHT(float target_straight_distance)
{
    car_state.straight_data.straight_distance_m = target_straight_distance;
    car_state.straight_data.straight_distance_encoder =  motor_ratio * huoer_pre * car_state.straight_data.straight_distance_m * 1000.0f / (motor_diameter * 2 * PI);
}

uint8_t Car_FSM_Cmd_Set(uint16_t cmd_id, float target_angle_change, float target_straight_distance)
{
    if(car_state.State != STOP)
    {
        return 1;
    }
    car_state.cmd_id = cmd_id;
    car_state.cmd_complete = 0;
    Car_FSM_Set_TURN(target_angle_change);
    Car_FSM_Set_STRAIGHT(target_straight_distance);
    Car_FSM_Change(TURN);
    return 0;
}

void Car_FSM_Execute(void)
{
    switch(car_state.State)
    {
        case STOP:
            l_motor.target_speed = 0;
            motor_speed_pid_cal(&l_motor);
            r_motor.target_speed = 0;
            motor_speed_pid_cal(&r_motor);
            motor_output();
            car_state.last_stop_time_ms = DWT_Get_ms();
            break;
        case TURN:
            if(fabsf(car_state.turn_data.yaw_target - mpu6500_data.yaw) > car_state.turn_data.finish_state_dead_zone || 
                     l_motor.speed > motor_static_speed || 
                     r_motor.speed > motor_static_speed)
            {
                car_state.turn_data.state_close_finish_time_ms = DWT_Get_ms();
            }
            else if(DWT_Get_ms() - car_state.turn_data.state_close_finish_time_ms > 200)
            {
                car_state.turn_data.state_close_finish_time_ms = DWT_Get_ms();
                Car_FSM_Change(STRAIGHT);
                break;
            }
            motor_angle_pid_cal(car_state.turn_data.yaw_target);
            motor_speed_rate_diff_pid_cal(car_state.turn_data.l_motor_encoder_start, car_state.turn_data.r_motor_encoder_start, -1);
            motor_speed_pid_cal(&l_motor);
            motor_speed_pid_cal(&r_motor);
            motor_output();
            break;
        case STRAIGHT:
            if(abs(car_state.straight_data.l_motor_encoder_target - l_motor.encoder) > car_state.straight_data.finish_state_dead_zone || 
               abs(car_state.straight_data.r_motor_encoder_target - r_motor.encoder) > car_state.straight_data.finish_state_dead_zone || 
               l_motor.speed > motor_static_speed || 
               r_motor.speed > motor_static_speed)
            {
                car_state.straight_data.state_close_finish_time_ms = DWT_Get_ms();
            }
            else if(DWT_Get_ms() - car_state.straight_data.state_close_finish_time_ms > 200)
            {
                car_state.straight_data.state_close_finish_time_ms = DWT_Get_ms();
                car_state.cmd_complete = 1;
                Car_FSM_Change(STOP);
                break;
            }
            motor_location_pid_cal(&l_motor, car_state.straight_data.l_motor_encoder_target);
            motor_location_pid_cal(&r_motor, car_state.straight_data.r_motor_encoder_target);
            motor_speed_rate_diff_pid_cal(car_state.straight_data.l_motor_encoder_start, car_state.straight_data.r_motor_encoder_start, 1);
            motor_speed_pid_cal(&l_motor);
            motor_speed_pid_cal(&r_motor);
            motor_output();
            break;
        default:
            Car_FSM_Change(STOP);
            l_motor.target_speed = 0;
            motor_speed_pid_cal(&l_motor);
            r_motor.target_speed = 0;
            motor_speed_pid_cal(&r_motor);
            break;
    }
}

