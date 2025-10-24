/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>

#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "motor.h"
#include "pid.h"
#include "dwt.h"
#include "delay.h"
#include "mpu6500_driver.h"
#include "rplidar.h"
#include "bluetooth.h"
#include "FSM.h"

#include "NVIC_Callback.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MPU_Task */
osThreadId_t MPU_TaskHandle;
const osThreadAttr_t MPU_Task_attributes = {
  .name = "MPU_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for FSM_MOTOR_Task */
osThreadId_t FSM_MOTOR_TaskHandle;
const osThreadAttr_t FSM_MOTOR_Task_attributes = {
  .name = "FSM_MOTOR_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for BLUETOOTH_Task */
osThreadId_t BLUETOOTH_TaskHandle;
const osThreadAttr_t BLUETOOTH_Task_attributes = {
  .name = "BLUETOOTH_Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for RPLIDAR_Task */
osThreadId_t RPLIDAR_TaskHandle;
const osThreadAttr_t RPLIDAR_Task_attributes = {
  .name = "RPLIDAR_Task",
  .stack_size = 1536 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void MPU_Task_Start(void *argument);
void FSM_MOTOR_Task_Start(void *argument);
void BLUETOOTH_Task_Start(void *argument);
void RPLIDAR_Task_Start(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of MPU_Task */
  MPU_TaskHandle = osThreadNew(MPU_Task_Start, NULL, &MPU_Task_attributes);

  /* creation of FSM_MOTOR_Task */
  FSM_MOTOR_TaskHandle = osThreadNew(FSM_MOTOR_Task_Start, NULL, &FSM_MOTOR_Task_attributes);

  /* creation of BLUETOOTH_Task */
  BLUETOOTH_TaskHandle = osThreadNew(BLUETOOTH_Task_Start, NULL, &BLUETOOTH_Task_attributes);

  /* creation of RPLIDAR_Task */
  RPLIDAR_TaskHandle = osThreadNew(RPLIDAR_Task_Start, NULL, &RPLIDAR_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  
  HAL_TIM_Base_Start_IT(&htim6);
  
  
  
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_MPU_Task_Start */
/**
* @brief Function implementing the MPU_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MPU_Task_Start */
uint8_t MPU_init_result;
uint8_t MPU_result;

void MPU_Task_Start(void *argument)
{
  /* USER CODE BEGIN MPU_Task_Start */
  float pitch,roll,yaw;

  short accel[3],gyro[3];
  
  MPU_init_result = InitMPU6050();
  MPU_init_result = MPU6500_DMP_Init();
  
  /* Infinite loop */
  for(;;)
  {
    if(MPU_EXTI_flag)
    {
        MPU_EXTI_flag = 0;
        MPU_result = MPU6500_dmp_get_euler_angle(accel, gyro, &pitch, &roll, &yaw);
        if(MPU_result == 0)
        {
            memcpy(&mpu6500_data.accel, accel, sizeof(mpu6500_data.accel));
            memcpy(&mpu6500_data.gyro, gyro, sizeof(mpu6500_data.gyro));
            mpu6500_data.pitch = pitch;
            mpu6500_data.roll = roll;
            mpu6500_data.yaw = yaw;
        }
    }
    
//    if(i2c_error || hi2c1.ErrorCode != 0)
//    {
//        I2C_Bus_Recovery(&hi2c1);
//        if(init_result)
//        {
//            init_result = InitMPU6050();
//            init_result = MPU6500_DMP_Init();
//        }
//    }
    osDelay(1);
  }
  /* USER CODE END MPU_Task_Start */
}

/* USER CODE BEGIN Header_FSM_MOTOR_Task_Start */
/**
* @brief Function implementing the FSM_MOTOR_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FSM_MOTOR_Task_Start */
float l_speed = 0;
float r_speed = 0;
float l_location = 0;
float r_location = 0;
float target_angle = 0;
float target_location = 0;

void FSM_MOTOR_Task_Start(void *argument)
{
  /* USER CODE BEGIN FSM_MOTOR_Task_Start */
  motor_enable();
  
  set_motor_speed(0.0, 0.0);
  motor_init();
  Car_FSM_Init();
  /* Infinite loop */
  for(;;)
  {
    if(tim_ms == 10)
    {
        tim_ms = 0;
        motor_update();
        Car_FSM_Execute();
//        if(DWT_Get_us() - Start > 20000000.0)
//        {
////            motor_angle_pid_cal(target_angle);
//            motor_location_pid_cal(&l_motor, target_location);
//            motor_location_pid_cal(&r_motor, target_location);
//        }
//        motor_speed_pid_cal(&l_motor);
//        motor_speed_pid_cal(&r_motor);
//        motor_output();
    }
    osDelay(1);
  }
  /* USER CODE END FSM_MOTOR_Task_Start */
}

/* USER CODE BEGIN Header_BLUETOOTH_Task_Start */
/**
* @brief Function implementing the BLUETOOTH_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BLUETOOTH_Task_Start */
BLUETOOTH_Receive_Data Bluetooth_Data;

uint8_t bluetooth_state;
double Bluetooth_Start;
uint16_t bluetooth_send_error, bluetooth_send_count;

void BLUETOOTH_Task_Start(void *argument)
{
  /* USER CODE BEGIN BLUETOOTH_Task_Start */
  uint8_t bluetooth_begin = 1;
  
  BLUETOOTH_ReceiveData_Init();
  HAL_UART_Receive_IT(&BLUETOOTH_huart, &rx_bluetooth_data, 1);
  /* Infinite loop */
  for(;;)
  {
    if(DWT_Get_ms() - Bluetooth_Start > 200)
    {
        if((!bluetooth_send_enable || !Bluetooth_Send_Data.Send_Complete) && !bluetooth_begin)
        {
            bluetooth_send_error++;
        }
        else
        {
            bluetooth_send_count++;
            BLUETOOTH_Reset_Send_Data();
        }
        if(bluetooth_begin)
        {
            bluetooth_send_enable = 1;
            bluetooth_begin = 0;
        }
        Bluetooth_Start = DWT_Get_ms();
    }
    if(bluetooth_send_enable && !Bluetooth_Send_Data.Send_Complete)
    {
        bluetooth_state = BLUETOOTH_Send_Data_to_Computer();
    }
    
    if(Complete_Bluetooth_Receive_Frame)
    {
        Complete_Bluetooth_Receive_Frame = 0;
        Bluetooth_Data = BLUETOOTH_Receive_Handler();
        BLUETOOTH_Receive_Data_Decode2FSM(Bluetooth_Data);
    }
    osDelay(1);
  }
  /* USER CODE END BLUETOOTH_Task_Start */
}

/* USER CODE BEGIN Header_RPLIDAR_Task_Start */
/**
* @brief Function implementing the RPLIDAR_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RPLIDAR_Task_Start */
RPLIDAR_Receive_Scan_Data Rplidar_Data;

void RPLIDAR_Task_Start(void *argument)
{
  /* USER CODE BEGIN RPLIDAR_Task_Start */
  RPLIDAR_ReceiveData_Init();
  HAL_UART_Receive_IT(&RPLIDAR_huart, &rx_rplidar_data, 1);
  osDelay(1000);
  
//  Rplidar_Get_Health(HAL_MAX_DELAY);
//  while(!Complete_Receive_Frame);
//  Complete_Receive_Frame = 0;
//  RPLIDAR_Receive_Get_Health_Data Rplidar_health = RPLIDAR_Receive_Get_Health_Handler();
  
//  Rplidar_Reset_Scan(HAL_MAX_DELAY);
  
  Rplidar_Start_Scan(HAL_MAX_DELAY);
  /* Infinite loop */
  for(;;)
  {
    if(Complete_Receive_Frame)
    {
        Complete_Receive_Frame = 0;
        Rplidar_Data = RPLIDAR_Receive_Scan_Handler();
    }
    osDelay(1);
  }
  /* USER CODE END RPLIDAR_Task_Start */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

