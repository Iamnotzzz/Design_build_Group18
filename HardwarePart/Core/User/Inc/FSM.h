#ifndef __FSM_H
#define __FSM_H

#include "main.h"

#define TURN_ANGLE_DEAD_ZONE              1.0f
#define STAIGHT_DISTANCE_DEAD_ZONE        20

typedef enum {
    STOP, 
    TURN, 
    STRAIGHT
} Car_State;

typedef struct {
    double state_start_time_ms;
    double state_close_finish_time_ms;
    
    float finish_state_dead_zone;
    
    float yaw_begin;
    float angle_change;
    
    float yaw_target;
    
    int l_motor_encoder_start;
    int r_motor_encoder_start;
} TURN_STATE_STRUCT;

typedef struct {
    double state_start_time_ms;
    double state_close_finish_time_ms;
    
    float finish_state_dead_zone;
    
    int l_motor_encoder_start;
    int r_motor_encoder_start;
    
    int l_motor_encoder_target;
    int r_motor_encoder_target;
    
    float straight_distance_m;
    int straight_distance_encoder;
} STRAIGHT_STATE_STRUCT;

typedef struct{
    uint16_t cmd_id;
    double last_stop_time_ms;
    Car_State State;
    
    TURN_STATE_STRUCT turn_data;
    STRAIGHT_STATE_STRUCT straight_data;
    
    uint8_t cmd_complete;
} Car_Struct;


extern Car_Struct car_state;

void Car_FSM_Init(void);
void Car_FSM_Change(Car_State State);
uint8_t Car_FSM_Cmd_Set(uint16_t cmd_id, float target_angle_change, float target_straight_distance);
void Car_FSM_Execute(void);





#endif
