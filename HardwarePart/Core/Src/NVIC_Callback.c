#include "NVIC_Callback.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "dwt.h"
#include "mpu6500_driver.h"
#include "rplidar.h"
#include "bluetooth.h"
#include "motor.h"
#include "FreeRTOS.h"

uint8_t rx_rplidar_data;
uint8_t rx_bluetooth_data;

uint8_t bluetooth_send_enable;

uint8_t tim_ms;
int32_t encoder1_last_count;
int32_t encoder1_count;

int32_t encoder2_count;
int16_t encoder2_last_count;


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM6)
    {
        tim_ms++;
        uint32_t DWT_CYCCNT = get_DWT_CYCCNT();
        // --- 编码器1 (32位定时器, 不需要扩展) ---
        encoder1_count = __HAL_TIM_GET_COUNTER(&encoder1_htim);

        // --- 编码器2 (16位定时器, 需要扩展) ---
        uint16_t now_cnt2 = __HAL_TIM_GET_COUNTER(&encoder2_htim);
        int16_t diff2 = (int16_t)(now_cnt2 - encoder2_last_count);
        encoder2_count += diff2;
        encoder2_last_count = now_cnt2;
        
        if(DWT_CYCCNT < last_DWT_CYCCNT)
        {
            DWT_count++;
        }
        last_DWT_CYCCNT = DWT_CYCCNT;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == MPU6500_INT_Pin)
    {
        MPU_EXTI_flag = 1;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == BLUETOOTH_huart.Instance)
    {
        bluetooth_send_enable = 1; // 发送完成
    }
}

extern RPLIDAR_Receive_Scan_Data Rplidar_Data;
extern BLUETOOTH_Receive_Data Bluetooth_Data;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == RPLIDAR_huart.Instance)   // 判断是哪个串口
    {
        Rplidar_Uart_Receive_Handler(rx_rplidar_data);
        HAL_UART_Receive_IT(&RPLIDAR_huart, &rx_rplidar_data, 1);
    }
    if (huart->Instance == BLUETOOTH_huart.Instance)   // 判断是哪个串口
    {
        Bluetooth_Uart_Receive_Handler(rx_bluetooth_data);
        HAL_UART_Receive_IT(&BLUETOOTH_huart, &rx_bluetooth_data, 1);
    }
}


// HAL 库回调函数：传输完成
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)  // 如果是 I2C1
    {
        i2c_tx_complete = 1;
    }
}
// HAL 库回调函数：接收完成
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        i2c_rx_complete = 1;
    }
}
// HAL 库回调函数：错误处理
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        i2c_error = 1;
//        printf("I2C Error: 0x%lX\r\n", hi2c->ErrorCode);

        // 可选：总线恢复或复位 I2C 外设
//        __HAL_RCC_I2C1_FORCE_RESET();
//        HAL_Delay(1);
//        __HAL_RCC_I2C1_RELEASE_RESET();
//        MX_I2C1_Init();
    }
}

