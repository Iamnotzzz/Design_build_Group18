/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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
#include "i2c.h"

/* USER CODE BEGIN 0 */
#include "delay.h"
#include "dwt.h"

/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = MPU6500_I2C_SCL_Pin|MPU6500_I2C_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* I2C1 interrupt Init */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(MPU6500_I2C_SCL_GPIO_Port, MPU6500_I2C_SCL_Pin);

    HAL_GPIO_DeInit(MPU6500_I2C_SDA_GPIO_Port, MPU6500_I2C_SDA_Pin);

    /* I2C1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void I2C_Bus_Recovery(I2C_HandleTypeDef* i2cHandle)
{
    if(i2cHandle->Instance==I2C1)
    {
        GPIO_InitTypeDef GPIO_InitStruct = {0};

        // 1. ����SCL��SDAΪGPIO���
        __HAL_RCC_GPIOB_CLK_ENABLE();
        GPIO_InitStruct.Pin = MPU6500_I2C_SCL_Pin|MPU6500_I2C_SDA_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;  // ��©
        GPIO_InitStruct.Pull = GPIO_NOPULL;          // �����ⲿ��������
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        // 2. ģ��ʱ�����壬�����ͷ�SDA
        for (int i = 0; i < 20; i++)
        {
            HAL_GPIO_WritePin(GPIOB, MPU6500_I2C_SCL_Pin, GPIO_PIN_RESET); // SCL = 0
            Delay_ms(1);
            HAL_GPIO_WritePin(GPIOB, MPU6500_I2C_SCL_Pin, GPIO_PIN_SET);   // SCL = 1
            Delay_ms(1);
        }

        // 3. ����һ��STOP������SDA�ӵ͵��ߣ�
        HAL_GPIO_WritePin(GPIOB, MPU6500_I2C_SDA_Pin, GPIO_PIN_RESET); // SDA = 0
        Delay_ms(1);
        HAL_GPIO_WritePin(GPIOB, MPU6500_I2C_SCL_Pin, GPIO_PIN_SET);   // SCL = 1
        Delay_ms(1);
        HAL_GPIO_WritePin(GPIOB, MPU6500_I2C_SDA_Pin, GPIO_PIN_SET);   // SDA = 1
        Delay_ms(1);

        __HAL_RCC_I2C1_FORCE_RESET();
        HAL_Delay(5);
        __HAL_RCC_I2C1_RELEASE_RESET();

        // 4. �ָ�ΪI2C���ù���
        MX_I2C1_Init();
        // CubeMX ��ʼ�� I2C ʱ���������ã�����Ҫ�ֶ�����
    }
}

uint8_t i2c_tx_complete;
uint8_t i2c_rx_complete;
uint8_t i2c_error;

// 封装的 I2C 写函数（中断方式）
HAL_StatusTypeDef My_I2C_Mem_Write_IT(
    I2C_HandleTypeDef *hi2c,
    uint16_t DevAddress,      // 设备地址（注意：7位地址左移1位）
    uint16_t MemAddress,      // 寄存器地址
    uint16_t MemAddSize,      // 寄存器地址字节数 (I2C_MEMADD_SIZE_8BIT / 16BIT)
    uint8_t *pData,           // 数据指针
    uint16_t Size,            // 数据长度
    uint8_t timeout           // 超时时间
)
{
    i2c_tx_complete = 0;
    i2c_error |= 0;

    HAL_StatusTypeDef status = HAL_I2C_Mem_Write_IT(
        hi2c,
        DevAddress,
        MemAddress,
        MemAddSize,
        pData,
        Size
    );
    double start = DWT_Get_ms();
    while(!(i2c_tx_complete | i2c_error))
    {
        if(DWT_Get_ms() - start > timeout) break;
    }
    
//    if(i2c_error || !(i2c_tx_complete | i2c_error))
//    {
//        I2C_Bus_Recovery(hi2c);
//        i2c_error |= 0;
//        i2c_tx_complete = 0;
//        status = HAL_I2C_Mem_Write_IT(
//            hi2c,
//            DevAddress,
//            MemAddress,
//            MemAddSize,
//            pData,
//            Size
//        );
//        start = DWT_Get_ms();
//        while(!(i2c_tx_complete | i2c_error))
//        {
//            if(DWT_Get_ms() - start > timeout) break;
//        }
//    }

    return status;
}

// 封装的 I2C 读函数（中断方式）
HAL_StatusTypeDef My_I2C_Mem_Read_IT(
    I2C_HandleTypeDef *hi2c,
    uint16_t DevAddress,      // 设备地址（7位地址左移1位）
    uint16_t MemAddress,      // 寄存器地址
    uint16_t MemAddSize,      // 寄存器地址字节数 (I2C_MEMADD_SIZE_8BIT / 16BIT)
    uint8_t *pData,           // 数据缓冲区
    uint16_t Size,            // 数据长度
    uint8_t timeout           // 超时时间
)
{
    i2c_rx_complete = 0;
    i2c_error |= 0;

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read_IT(
        hi2c,
        DevAddress,
        MemAddress,
        MemAddSize,
        pData,
        Size
    );
    double start = DWT_Get_ms();
    while(!(i2c_rx_complete | i2c_error))
    {
        if(DWT_Get_ms() - start > timeout) break;
    }
    
//    if(i2c_error || !(i2c_rx_complete | i2c_error))
//    {
//        I2C_Bus_Recovery(hi2c);
//        i2c_error |= 0;
//        i2c_rx_complete = 0;
//        status = HAL_I2C_Mem_Read_IT(
//            hi2c,
//            DevAddress,
//            MemAddress,
//            MemAddSize,
//            pData,
//            Size
//        );
//        start = DWT_Get_ms();
//        while(!(i2c_rx_complete | i2c_error))
//        {
//            if(DWT_Get_ms() - start > timeout) break;
//        }
//    }

    return status;
}
/* USER CODE END 1 */
