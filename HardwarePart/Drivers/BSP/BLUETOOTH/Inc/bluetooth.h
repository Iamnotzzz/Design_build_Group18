#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "main.h"
#include "rplidar.h"

#define BLUETOOTH_huart             huart3
#define BLUETOOTH_MAX_DATA_LEN      20

typedef enum {
    BLUETOOTH_STATE_WAIT_HEADER1,
    BLUETOOTH_STATE_WAIT_HEADER2,
    BLUETOOTH_STATE_RECEIVE_LEN,
    BLUETOOTH_STATE_RECEIVE_DATA,
    BLUETOOTH_STATE_RECEIVE_CRC
} BLUETOOTH_Receive_State;

typedef struct{
    BLUETOOTH_Receive_State State;
    
    uint16_t Receive_data_len;
    
    uint8_t Receive_header[4];
    uint8_t Receive_header_count;
    
    uint8_t* Receive_data;
    uint8_t Receive_data_count;
    
    uint16_t Receive_data_CRC;
    uint8_t Receive_crc_count;
}BLUETOOTH_Receive_Data_Type;

typedef struct{
    uint16_t cmd_id;
    union {
        int turn_rad;
        uint8_t turn_data[4];
    }turn_rad_data;
    union {
        int straight_m;
        uint8_t straight_data[4];
    }straight_m_data;
    
} BLUETOOTH_Receive_Data;


extern uint8_t Complete_Bluetooth_Receive_Frame;

int BLUETOOTH_ReceiveData_Init(void);
void Bluetooth_Uart_Receive_Handler(uint8_t rxdata);
BLUETOOTH_Receive_Data BLUETOOTH_Receive_Handler(void);
uint8_t BLUETOOTH_Receive_Data_Decode2FSM(BLUETOOTH_Receive_Data bluetooth_data);



#define SEND_BEGIN_1        0x55
#define SEND_BEGIN_2        0xAA

typedef struct __attribute__((packed)){
    uint8_t Send_Begin_1;
    uint8_t Send_Begin_2;
    
    uint32_t time_us;
    
    uint16_t cmd_id;
    uint8_t status;
    
    int encoder_l;
    int encoder_r;
    
    float current_yaw;
    
    RPLIDAR_Send_Data rplidar_data;
} BLUETOOTH_Send_Data;

typedef struct {
    uint16_t current_send_size;
    uint16_t data_send_size;
    uint8_t Send_Complete;
    
    BLUETOOTH_Send_Data send_data;
} BLUETOOTH_Send_Data_Type;


extern BLUETOOTH_Send_Data_Type Bluetooth_Send_Data;

void BLUETOOTH_Reset_Send_Data(void);
uint8_t BLUETOOTH_Send_Data_to_Computer(void);

#endif
