#ifndef __RPLIDAR_H
#define __RPLIDAR_H

#include "main.h"

#define RPLIDAR_huart                   huart6

#define RPLIDAR_SEND_ID                         0xA5
#define RPLIDAR_RECEIVE_ID1                     0xA5
#define RPLIDAR_RECEIVE_ID2                     0x5A

#define RECEIVE_MODE_SINGLE                     0x00
#define RECEIVE_MODE_MULTIPLE                   0x01

// 命令指令
#define RPLIDAR_STOP_CMD                        0x25        //停止扫描，无应答
#define RPLIDAR_RESET_CMD                       0x40        //测距核心软重启，无应答
#define RPLIDAR_SCAN_CMD                        0x20        //开始扫描采样，多次应答
#define RPLIDAR_EXPRESS_SCAN_CMD                0x82        //开始高速采样，多次应答
#define RPLIDAR_GET_INFO_CMD                    0x50        //设备信息获取，单次应答
#define RPLIDAR_GET_HEALTH_CMD                  0x52        //设备健康状态获取，单次应答
#define RPLIDAR_GET_SAMPLERATE_CMD              0x59        //激光测距用时获取，单次应答
#define RPLIDAR_GET_LIDAR_CONF_CMD              0x84        //设备配置信息获取，单次应答

// 应答ID
#define RPLIDAR_RECEIVE_SCAN_ID                 0x81        //开始扫描采样，多次应答
#define RPLIDAR_RECEIVE_EXPRESS_SCAN_ID         0x85        //开始高速采样，多次应答
#define RPLIDAR_RECEIVE_GET_INFO_ID             0x04        //设备信息获取，单次应答
#define RPLIDAR_RECEIVE_GET_HEALTH_ID           0x06        //设备健康状态获取，单次应答
#define RPLIDAR_RECEIVE_GET_SAMPLERATE_ID       0x15        //激光测距用时获取，单次应答
#define RPLIDAR_RECEIVE_GET_LIDAR_CONF_ID       0x20        //设备配置信息获取，单次应答

//最大接收长度
#define RPLIDAR_MAX_DATA_LEN                    100

typedef enum {
    RPLIDAR_STATE_WAIT_HEADER1,
    RPLIDAR_STATE_WAIT_HEADER2,
    RPLIDAR_STATE_RECEIVE_HEADER,
    RPLIDAR_STATE_RECEIVE_DATA
} RPLIDAR_Receive_State;

typedef enum {
    TYPE_Receive_IDLE,
    TYPE_Receive_Scan,
    TYPE_Receive_Express_Scan,
    TYPE_Receive_Get_INFO,
    TYPE_Receive_Get_Health,
    TYPE_Receive_Get_SampleRate,
    TYPE_Receive_Get_Lidar_Conf,
    TYPE_Receive_UNKNOWN
} RPLIDAR_Receive_Type;

typedef enum {
    MODE_Receive_Single,
    MODE_Receive_Multiple,
    MODE_Receive_Default
} RPLIDAR_Receive_Mode;

typedef struct{
    RPLIDAR_Receive_State State;
    
    uint32_t Receive_data_len;
    RPLIDAR_Receive_Type Type;
    RPLIDAR_Receive_Mode Mode;
    
    uint8_t Receive_header[7];
    uint8_t Receive_header_count;
    
    uint8_t* Receive_data;
    uint8_t Receive_data_count;
}RPLIDAR_Receive_Data_Type;



typedef struct{
    uint8_t S;
    uint8_t Quality;
    uint16_t angle_q6;
    uint16_t distance_q2;
    float angle;
    float distance;
}RPLIDAR_Receive_Scan_Data;

typedef struct{
    uint8_t S;
    uint8_t Quality;
    uint16_t start_angle_q6;
    uint16_t distance_q2[40];
}RPLIDAR_Receive_Express_Scan_Data;

typedef struct{
    uint8_t MajorModel;
    uint8_t SubModel;
    uint8_t firmware_minor;
    uint8_t firmware_major;
    uint8_t hardware;
    uint8_t serialnumber[16];
}RPLIDAR_Receive_Get_INFO_Data;

typedef enum {
    RPLIDAR_Status_Better,
    RPLIDAR_Status_Warning,
    RPLIDAR_Status_Error,
    RPLIDAR_Status_UNKNOWN
} RPLIDAR_Status;

typedef struct{
    RPLIDAR_Status status;
    uint16_t error_code;
}RPLIDAR_Receive_Get_Health_Data;

typedef struct{
    uint16_t standard_time;
    uint16_t express_time;
}RPLIDAR_Receive_Get_SampleRate_Data;


extern uint8_t Complete_Receive_Frame;


int RPLIDAR_ReceiveData_Init(void);
void RPLIDAR_ReceiveData_Clear(void);

HAL_StatusTypeDef Rplidar_Stop_Scan(uint32_t Timeout);
HAL_StatusTypeDef Rplidar_Reset_Scan(uint32_t Timeout);
HAL_StatusTypeDef Rplidar_Start_Scan(uint32_t Timeout);
HAL_StatusTypeDef Rplidar_Start_Express_Scan(uint32_t Timeout);
HAL_StatusTypeDef Rplidar_Get_INFO(uint32_t Timeout);
HAL_StatusTypeDef Rplidar_Get_Health(uint32_t Timeout);
HAL_StatusTypeDef Rplidar_Get_SampleRate(uint32_t Timeout);

void Rplidar_Uart_Receive_Handler(uint8_t rx_data);

RPLIDAR_Receive_Scan_Data RPLIDAR_Receive_Scan_Handler(void);
RPLIDAR_Receive_Express_Scan_Data RPLIDAR_Receive_Express_Scan_Handler(void);
RPLIDAR_Receive_Get_INFO_Data RPLIDAR_Receive_Get_INFO_Handler(void);
RPLIDAR_Receive_Get_Health_Data RPLIDAR_Receive_Get_Health_Handler(void);
RPLIDAR_Receive_Get_SampleRate_Data RPLIDAR_Receive_Get_SampleRate_Handler(void);





#define SEND_DATA_MAX_COUNT     600

typedef struct __attribute__((packed)){
    uint8_t Quality;
    uint16_t angle_q6;
    uint16_t distance_q2;
}RPLIDAR_Scan_Data_Send;

typedef struct __attribute__((packed)){
    uint16_t data_count;
    RPLIDAR_Scan_Data_Send send_data[SEND_DATA_MAX_COUNT];
} RPLIDAR_Send_Data;

extern RPLIDAR_Send_Data last_rplidar_data;

void RPLIDAR_Send_Data_Clear(RPLIDAR_Send_Data *send_data);
void RPLIDAR_Send_Scan_Data_add(RPLIDAR_Receive_Scan_Data scan_data);


#endif
