#include "bluetooth.h"
#include "FSM.h"
#include "mpu6500_driver.h"
#include "motor.h"
#include "dwt.h"

#include "usart.h"
#include "NVIC_Callback.h"
#include <string.h>
#include <stdlib.h>

BLUETOOTH_Receive_Data_Type Bluetooth_data;
uint8_t Bluetooth_Receive_buff[BLUETOOTH_MAX_DATA_LEN];
uint8_t Complete_Bluetooth_Receive_Frame = 0;

int BLUETOOTH_ReceiveData_Init(void)
{
    // 清零结构体
    memset(&Bluetooth_data, 0, sizeof(BLUETOOTH_Receive_Data_Type));

    // 初始化状态
    Bluetooth_data.State = BLUETOOTH_STATE_WAIT_HEADER1;
    Bluetooth_data.Receive_data_len = 0;
    Bluetooth_data.Receive_header_count = 0;
    Bluetooth_data.Receive_data_count = 0;
    Bluetooth_data.Receive_crc_count = 0;
    
    Bluetooth_data.Receive_data_CRC = 0;

    // 动态分配缓存
    Bluetooth_data.Receive_data = (uint8_t *)malloc(BLUETOOTH_MAX_DATA_LEN);
    if (Bluetooth_data.Receive_data == NULL)
        return -1;

    // 初始化数据缓存
    memset(Bluetooth_data.Receive_data, 0, BLUETOOTH_MAX_DATA_LEN);

    return 0;
}

void BLUETOOTH_ReceiveData_Clear(void)
{
    if (Bluetooth_data.Receive_data != NULL)
        memset(Bluetooth_data.Receive_data, 0, BLUETOOTH_MAX_DATA_LEN);

    // 清零状态
    Bluetooth_data.State = BLUETOOTH_STATE_WAIT_HEADER1;
    Bluetooth_data.Receive_data_len = 0;
    Bluetooth_data.Receive_header_count = 0;
    Bluetooth_data.Receive_data_count = 0;
    Bluetooth_data.Receive_crc_count = 0;
    
    Bluetooth_data.Receive_data_CRC = 0;
}

void Bluetooth_Uart_Receive_Handler(uint8_t rxdata)
{
    switch(Bluetooth_data.State)
    {
        case BLUETOOTH_STATE_WAIT_HEADER1:
            if(rxdata == 0xAA)
            {
                Bluetooth_data.State = BLUETOOTH_STATE_WAIT_HEADER2;
                Bluetooth_data.Receive_header[0] = rxdata;
                Bluetooth_data.Receive_header_count++;
            }
            else
            {
                BLUETOOTH_ReceiveData_Clear();
            }
            break;
        case BLUETOOTH_STATE_WAIT_HEADER2:
            if(rxdata == 0x55)
            {
                Bluetooth_data.State = BLUETOOTH_STATE_RECEIVE_LEN;
                Bluetooth_data.Receive_header[1] = rxdata;
                Bluetooth_data.Receive_header_count++;
            }
            else
            {
                BLUETOOTH_ReceiveData_Clear();
            }
            break;
        case BLUETOOTH_STATE_RECEIVE_LEN:
            Bluetooth_data.Receive_header[Bluetooth_data.Receive_header_count] = rxdata;
            Bluetooth_data.Receive_header_count++;
            if(Bluetooth_data.Receive_header_count == 4)
            {
                Bluetooth_data.Receive_data_len = Bluetooth_data.Receive_header[2] | (Bluetooth_data.Receive_header[3] << 8);
                Bluetooth_data.State = BLUETOOTH_STATE_RECEIVE_DATA;
            }
            break;
        case BLUETOOTH_STATE_RECEIVE_DATA:
            Bluetooth_data.Receive_data[Bluetooth_data.Receive_data_count] = rxdata;
            Bluetooth_data.Receive_data_count++;
            if(Bluetooth_data.Receive_data_count == Bluetooth_data.Receive_data_len)
            {
                Bluetooth_data.State = BLUETOOTH_STATE_RECEIVE_CRC;
            }
            break;
        case BLUETOOTH_STATE_RECEIVE_CRC:
            Bluetooth_data.Receive_data_CRC |= rxdata << (Bluetooth_data.Receive_crc_count * 8);
            Bluetooth_data.Receive_crc_count++;
            if(Bluetooth_data.Receive_crc_count == 2)
            {
                if(1)
                {
                    memcpy(Bluetooth_Receive_buff, Bluetooth_data.Receive_data, Bluetooth_data.Receive_data_len);
                    Complete_Bluetooth_Receive_Frame = 1;
                }
                BLUETOOTH_ReceiveData_Clear();
            }
            break;
        default:
            BLUETOOTH_ReceiveData_Clear();
            break;
    }
}

BLUETOOTH_Receive_Data BLUETOOTH_Receive_Handler(void)
{
    uint8_t buffer[10];
    BLUETOOTH_Receive_Data receive_data;
    memcpy(buffer, Bluetooth_Receive_buff, 10);
    
    receive_data.cmd_id = buffer[0] | (buffer[1] << 8);
    
    memcpy(&receive_data.turn_rad_data.turn_data, buffer + 2, 4);
    memcpy(&receive_data.straight_m_data.straight_data, buffer + 6, 4);
    
    return receive_data;
}

uint8_t BLUETOOTH_Receive_Data_Decode2FSM(BLUETOOTH_Receive_Data bluetooth_data)
{
    return Car_FSM_Cmd_Set(bluetooth_data.cmd_id, bluetooth_data.turn_rad_data.turn_rad, bluetooth_data.straight_m_data.straight_m);
}




BLUETOOTH_Send_Data_Type Bluetooth_Send_Data;

void BLUETOOTH_Reset_Send_Data(void)
{
    Bluetooth_Send_Data.send_data.Send_Begin_1 = SEND_BEGIN_1;
    Bluetooth_Send_Data.send_data.Send_Begin_2 = SEND_BEGIN_2;
    
    Bluetooth_Send_Data.send_data.cmd_id = car_state.cmd_id;
    Bluetooth_Send_Data.send_data.status = car_state.State;
    Bluetooth_Send_Data.send_data.time_us = DWT_Get_32us();
    
    Bluetooth_Send_Data.send_data.current_yaw = mpu6500_data.yaw;
    Bluetooth_Send_Data.send_data.encoder_l = l_motor.encoder;
    Bluetooth_Send_Data.send_data.encoder_r = r_motor.encoder;
    
    Bluetooth_Send_Data.send_data.rplidar_data = last_rplidar_data;
    
    Bluetooth_Send_Data.data_send_size = sizeof(BLUETOOTH_Send_Data) - (SEND_DATA_MAX_COUNT - Bluetooth_Send_Data.send_data.rplidar_data.data_count) * sizeof(RPLIDAR_Scan_Data_Send);
    Bluetooth_Send_Data.current_send_size = 0;
    Bluetooth_Send_Data.Send_Complete = 0;
}

uint8_t BLUETOOTH_Send_Data_to_Computer(void)
{
    if(bluetooth_send_enable && !Bluetooth_Send_Data.Send_Complete)
    {
        bluetooth_send_enable = 0;
        if(Bluetooth_Send_Data.current_send_size + DMA_MAX_TX < Bluetooth_Send_Data.data_send_size)
        {
            if(HAL_UART_Transmit_DMA(&BLUETOOTH_huart, (uint8_t *)(&Bluetooth_Send_Data.send_data) + Bluetooth_Send_Data.current_send_size, DMA_MAX_TX) == HAL_OK)
            {
                Bluetooth_Send_Data.current_send_size += DMA_MAX_TX;
                return 0;
            }
            else 
            {
                return 1;
            }
        }
        else
        {
            if(HAL_UART_Transmit_DMA(&BLUETOOTH_huart, (uint8_t *)&Bluetooth_Send_Data.send_data + Bluetooth_Send_Data.current_send_size, Bluetooth_Send_Data.data_send_size - Bluetooth_Send_Data.current_send_size) == HAL_OK)
            {
                Bluetooth_Send_Data.current_send_size = Bluetooth_Send_Data.data_send_size;
                Bluetooth_Send_Data.Send_Complete = 1;
                return 0;
            }
            else 
            {
                return 1;
            }
        }
    }
    return 1;
}

