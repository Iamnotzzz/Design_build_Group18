#include "rplidar.h"
#include "usart.h"
#include <string.h>
#include <stdlib.h>

RPLIDAR_Receive_Data_Type Rplidar_data;
uint8_t Rplidar_Receive_buff[RPLIDAR_MAX_DATA_LEN];
uint8_t Complete_Receive_Frame = 0;

int RPLIDAR_ReceiveData_Init(void)
{
    // 停止当前动作
    Rplidar_Stop_Scan(HAL_MAX_DELAY);
    
    // 清零结构体
    memset(&Rplidar_data, 0, sizeof(RPLIDAR_Receive_Data_Type));

    // 初始化状态
    Rplidar_data.State = RPLIDAR_STATE_WAIT_HEADER1;
    Rplidar_data.Receive_data_len = 0;
    Rplidar_data.Type = TYPE_Receive_IDLE;
    Rplidar_data.Mode = MODE_Receive_Single;
    Rplidar_data.Receive_header_count = 0;
    Rplidar_data.Receive_data_count = 0;

    // 动态分配缓存
    Rplidar_data.Receive_data = (uint8_t *)malloc(RPLIDAR_MAX_DATA_LEN);
    if (Rplidar_data.Receive_data == NULL)
        return -1;

    // 初始化数据缓存
    memset(Rplidar_data.Receive_data, 0, RPLIDAR_MAX_DATA_LEN);

    return 0;
}

void RPLIDAR_ReceiveData_Clear(void)
{
    if (Rplidar_data.Receive_data != NULL)
        memset(Rplidar_data.Receive_data, 0, RPLIDAR_MAX_DATA_LEN);

    // 清零状态
    Rplidar_data.State = RPLIDAR_STATE_WAIT_HEADER1;
    Rplidar_data.Receive_data_len = 0;
    Rplidar_data.Type = TYPE_Receive_IDLE;
    Rplidar_data.Mode = MODE_Receive_Single;
    Rplidar_data.Receive_header_count = 0;
    Rplidar_data.Receive_data_count = 0;
}

HAL_StatusTypeDef Rplidar_Stop_Scan(uint32_t Timeout)
{
    uint8_t msg[] = {RPLIDAR_SEND_ID, RPLIDAR_STOP_CMD};
    return HAL_UART_Transmit(&RPLIDAR_huart, (uint8_t*)msg, sizeof(msg), Timeout);
}

HAL_StatusTypeDef Rplidar_Reset_Scan(uint32_t Timeout)
{
    uint8_t msg[] = {RPLIDAR_SEND_ID, RPLIDAR_RESET_CMD};
    return HAL_UART_Transmit(&RPLIDAR_huart, (uint8_t*)msg, sizeof(msg), Timeout);
}

HAL_StatusTypeDef Rplidar_Start_Scan(uint32_t Timeout)
{
    uint8_t msg[] = {RPLIDAR_SEND_ID, RPLIDAR_SCAN_CMD};
    return HAL_UART_Transmit(&RPLIDAR_huart, (uint8_t*)msg, sizeof(msg), Timeout);
}

HAL_StatusTypeDef Rplidar_Start_Express_Scan(uint32_t Timeout)
{
    uint8_t msg[] = {RPLIDAR_SEND_ID, RPLIDAR_EXPRESS_SCAN_CMD, 0x05, 0, 0, 0, 0, 0, 22};
    return HAL_UART_Transmit(&RPLIDAR_huart, (uint8_t*)msg, sizeof(msg), Timeout);
}

HAL_StatusTypeDef Rplidar_Get_INFO(uint32_t Timeout)
{
    uint8_t msg[] = {RPLIDAR_SEND_ID, RPLIDAR_GET_INFO_CMD};
    return HAL_UART_Transmit(&RPLIDAR_huart, (uint8_t*)msg, sizeof(msg), Timeout);
}

HAL_StatusTypeDef Rplidar_Get_Health(uint32_t Timeout)
{
    uint8_t msg[] = {RPLIDAR_SEND_ID, RPLIDAR_GET_HEALTH_CMD};
    return HAL_UART_Transmit(&RPLIDAR_huart, (uint8_t*)msg, sizeof(msg), Timeout);
}

HAL_StatusTypeDef Rplidar_Get_SampleRate(uint32_t Timeout)
{
    uint8_t msg[] = {RPLIDAR_SEND_ID, RPLIDAR_GET_SAMPLERATE_CMD};
    return HAL_UART_Transmit(&RPLIDAR_huart, (uint8_t*)msg, sizeof(msg), Timeout);
}

void Rplidar_Uart_Receive_Handler(uint8_t rx_data)
{
    switch(Rplidar_data.State)
    {
        case RPLIDAR_STATE_WAIT_HEADER1:
            if(rx_data == RPLIDAR_RECEIVE_ID1)
            {
                Rplidar_data.State = RPLIDAR_STATE_WAIT_HEADER2;
                Rplidar_data.Receive_header[Rplidar_data.Receive_header_count] = rx_data;
                Rplidar_data.Receive_header_count++;
            }
            else
            {
                RPLIDAR_ReceiveData_Clear();
            }
            break;
        case RPLIDAR_STATE_WAIT_HEADER2:
            if(rx_data == RPLIDAR_RECEIVE_ID2)
            {
                Rplidar_data.State = RPLIDAR_STATE_RECEIVE_HEADER;
                Rplidar_data.Receive_header[Rplidar_data.Receive_header_count] = rx_data;
                Rplidar_data.Receive_header_count++;
            }
            else
            {
                RPLIDAR_ReceiveData_Clear();
            }
            break;
        case RPLIDAR_STATE_RECEIVE_HEADER:
            Rplidar_data.Receive_header[Rplidar_data.Receive_header_count] = rx_data;
            Rplidar_data.Receive_header_count++;
            if(Rplidar_data.Receive_header_count == 7)
            {
                Rplidar_data.Receive_data_len = (Rplidar_data.Receive_header[2] | 
                                                (Rplidar_data.Receive_header[3] << 8) | 
                                                (Rplidar_data.Receive_header[4] << 16) | 
                                                (Rplidar_data.Receive_header[5] << 24)) & 
                                                0x3FFFFFFF;
                uint8_t mode = (Rplidar_data.Receive_header[5] & 0xC0) >> 6;
                Rplidar_data.Mode = (mode == MODE_Receive_Single || mode == MODE_Receive_Multiple) ? mode : MODE_Receive_Default;
                Rplidar_data.State = RPLIDAR_STATE_RECEIVE_DATA;
                switch(Rplidar_data.Receive_header[6])
                {
                    case RPLIDAR_RECEIVE_SCAN_ID:
                        Rplidar_data.Type = TYPE_Receive_Scan;
                        break;
                    case RPLIDAR_RECEIVE_EXPRESS_SCAN_ID:
                        Rplidar_data.Type = TYPE_Receive_Express_Scan;
                        break;
                    case RPLIDAR_RECEIVE_GET_INFO_ID:
                        Rplidar_data.Type = TYPE_Receive_Get_INFO;
                        break;
                    case RPLIDAR_RECEIVE_GET_HEALTH_ID:
                        Rplidar_data.Type = TYPE_Receive_Get_Health;
                        break;
                    case RPLIDAR_RECEIVE_GET_SAMPLERATE_ID:
                        Rplidar_data.Type = TYPE_Receive_Get_SampleRate;
                        break;
                    case RPLIDAR_RECEIVE_GET_LIDAR_CONF_ID:
                        Rplidar_data.Type = TYPE_Receive_Get_Lidar_Conf;
                        break;
                    default :
                        Rplidar_data.Type = TYPE_Receive_UNKNOWN;
                        break;
                }
            }
            break;
        case RPLIDAR_STATE_RECEIVE_DATA:
            if(Rplidar_data.Receive_data_count < Rplidar_data.Receive_data_len)
            {
                Rplidar_data.Receive_data[Rplidar_data.Receive_data_count] = rx_data;
                Rplidar_data.Receive_data_count++;
                if(Rplidar_data.Receive_data_count == Rplidar_data.Receive_data_len)
                {
                    memcpy(Rplidar_Receive_buff, Rplidar_data.Receive_data, Rplidar_data.Receive_data_len);
                    if(Rplidar_data.Mode != MODE_Receive_Multiple)
                    {
                        RPLIDAR_ReceiveData_Clear();
                    }
                    else
                    {
                        memset(Rplidar_data.Receive_data, 0, RPLIDAR_MAX_DATA_LEN);
                        Rplidar_data.Receive_data_count = 0;
                    }
                    Complete_Receive_Frame = 1;
                }
            }
            break;
        default :
            RPLIDAR_ReceiveData_Clear();
            break;
    }
}

RPLIDAR_Receive_Scan_Data RPLIDAR_Receive_Scan_Handler(void)
{
    uint8_t buffer[5];
    RPLIDAR_Receive_Scan_Data scan_data;
    memcpy(buffer, Rplidar_Receive_buff, 5);
    scan_data.S = buffer[0] & 0x01;
    if(scan_data.S == ((buffer[0] & 0x02) >> 1) || (buffer[1] & 0x01)) scan_data.Quality = 0;
    else scan_data.Quality = (buffer[0] & 0xFC) >> 2;
    
    scan_data.angle_q6 = (buffer[1] >> 1) | (buffer[2] << 7);
    scan_data.distance_q2 = (buffer[3]) | (buffer[4] << 8);
    
    scan_data.angle = scan_data.angle_q6 / 64.0f;
    scan_data.distance = scan_data.distance_q2 / 4.0f;
    
    RPLIDAR_Send_Scan_Data_add(scan_data);
    
    Complete_Receive_Frame = 0;
    return scan_data;
}

RPLIDAR_Receive_Express_Scan_Data RPLIDAR_Receive_Express_Scan_Handler(void)
{
    uint8_t buffer[84];
    RPLIDAR_Receive_Express_Scan_Data scan_data;
    memcpy(buffer, Rplidar_Receive_buff, 84);
    scan_data.Quality = 1;
    scan_data.S = (buffer[3] & 0x80) >> 7;
    uint8_t checksum = (buffer[0] & 0x0F) | ((buffer[1] & 0x0F) << 4);
    if(((buffer[0] & 0xF0) >> 4) != 0xA || ((buffer[1] & 0xF0) >> 4) != 0x5) scan_data.Quality = 0;
    
    scan_data.start_angle_q6 = (buffer[2]) | ((buffer[3] & 0x7F) << 8);
//    memcpy(&scan_data.distance_q2, buffer + 4, 80);
    uint8_t check = 0;
    for(uint8_t i = 2; i <= 83; i++)
    {
        if(i % 2 == 0 && i >= 4) scan_data.distance_q2[i/2 - 2] = buffer[i] | (buffer[i+1] << 8);
        check ^= buffer[i];
    }
    if(checksum != check) scan_data.Quality = 0;
    
    Complete_Receive_Frame = 0;
    return scan_data;
}

RPLIDAR_Receive_Get_INFO_Data RPLIDAR_Receive_Get_INFO_Handler(void)
{
    uint8_t buffer[20];
    RPLIDAR_Receive_Get_INFO_Data scan_data;
    memcpy(buffer, Rplidar_Receive_buff, 20);
    scan_data.SubModel = buffer[0] & 0x0F;
    scan_data.MajorModel = buffer[0] >> 4;
    scan_data.firmware_minor = buffer[1];
    scan_data.firmware_major = buffer[2];
    scan_data.hardware = buffer[3];
    memcpy(&scan_data.serialnumber, buffer + 4, 16);
    
    return scan_data;
}

RPLIDAR_Receive_Get_Health_Data RPLIDAR_Receive_Get_Health_Handler(void)
{
    uint8_t buffer[3];
    RPLIDAR_Receive_Get_Health_Data scan_data;
    memcpy(buffer, Rplidar_Receive_buff, 3);
    scan_data.status = (buffer[0] == RPLIDAR_Status_Better || buffer[0] == RPLIDAR_Status_Warning || buffer[0] == RPLIDAR_Status_Error) ? buffer[0] : RPLIDAR_Status_UNKNOWN;
    scan_data.error_code = buffer[1] | (buffer[2] << 8);
    
    return scan_data;
}

RPLIDAR_Receive_Get_SampleRate_Data RPLIDAR_Receive_Get_SampleRate_Handler(void)
{
    uint8_t buffer[4];
    RPLIDAR_Receive_Get_SampleRate_Data scan_data;
    memcpy(buffer, Rplidar_Receive_buff, 4);
    scan_data.standard_time = buffer[0] | (buffer[1] << 8);
    scan_data.express_time = buffer[2] | (buffer[3] << 8);
    
    return scan_data;
}



RPLIDAR_Send_Data current_rplidar_data;
RPLIDAR_Send_Data last_rplidar_data;
uint16_t last_angle_data;
uint16_t first_angle_data;
uint8_t state;


void RPLIDAR_Send_Data_Clear(RPLIDAR_Send_Data *send_data)
{
    send_data->data_count = 0;
    memset(&send_data->send_data, 0, sizeof(send_data->send_data));
}

void RPLIDAR_Send_Scan_Data_add(RPLIDAR_Receive_Scan_Data scan_data)
{
    if( /* scan_data.S == 1 || */
        (scan_data.angle_q6 >= first_angle_data && last_angle_data < first_angle_data) || 
        (first_angle_data > last_angle_data && last_angle_data > 320 * 64 && scan_data.angle_q6 < 40*64) || 
        (first_angle_data <= scan_data.angle_q6 && last_angle_data > 320 * 64 && scan_data.angle_q6 < 40*64)) 
    {
        memcpy(&last_rplidar_data, &current_rplidar_data, sizeof(RPLIDAR_Send_Data));
        RPLIDAR_Send_Data_Clear(&current_rplidar_data);
        first_angle_data = scan_data.angle_q6;
        last_angle_data = scan_data.angle_q6;
    }
    if(last_rplidar_data.data_count > 500 || last_rplidar_data.data_count < 350)
    {
        state = 0;
    }
    if (current_rplidar_data.data_count < SEND_DATA_MAX_COUNT)
    {
        current_rplidar_data.send_data[current_rplidar_data.data_count].Quality = scan_data.Quality;
        current_rplidar_data.send_data[current_rplidar_data.data_count].angle_q6 = scan_data.angle_q6 | 0x8000;
        current_rplidar_data.send_data[current_rplidar_data.data_count].distance_q2 = scan_data.distance_q2;
    }
    last_angle_data = scan_data.angle_q6;
    current_rplidar_data.data_count++;
    
    
}
