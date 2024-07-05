/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
//void MX_MEMS_Init(void);

//void MX_MEMS_Process(void);
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct
{
  uint8_t minutes;
  uint8_t seconds;
  float pressure;
  int32_t acceleration_x_mg;
  int32_t acceleration_y_mg;
  int32_t acceleration_z_mg;
  int32_t angular_rate_x_mdps;
  int32_t angular_rate_y_mdps;
  int32_t angular_rate_z_mdps;
} offline_data_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define TMsg_EOF                0xF0
#define TMsg_BS                 0xF1
#define TMsg_BS_EOF             0xF2

#ifdef USE_USB_OTG_HS
#define TMsg_MaxLen             512
#else
#define TMsg_MaxLen             256
#endif

#define OFFLINE_DATA_SIZE  8
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
int _write(int file, char *ptr, int len);
void K2000(void);
void switchLedAll(double angle);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define My_adc_Pin GPIO_PIN_0
#define My_adc_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define L0_Pin GPIO_PIN_1
#define L0_GPIO_Port GPIOB
#define L1_Pin GPIO_PIN_2
#define L1_GPIO_Port GPIOB
#define L2_Pin GPIO_PIN_10
#define L2_GPIO_Port GPIOB
#define L3_Pin GPIO_PIN_11
#define L3_GPIO_Port GPIOB
#define L4_Pin GPIO_PIN_12
#define L4_GPIO_Port GPIOB
#define L5_Pin GPIO_PIN_13
#define L5_GPIO_Port GPIOB
#define L6_Pin GPIO_PIN_14
#define L6_GPIO_Port GPIOB
#define L7_Pin GPIO_PIN_15
#define L7_GPIO_Port GPIOB
#define SPI_CS_Pin GPIO_PIN_8
#define SPI_CS_GPIO_Port GPIOA
#define BP1_Pin GPIO_PIN_11
#define BP1_GPIO_Port GPIOA
#define BP1_EXTI_IRQn EXTI15_10_IRQn
#define BP2_Pin GPIO_PIN_12
#define BP2_GPIO_Port GPIOA
#define BP2_EXTI_IRQn EXTI15_10_IRQn
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
typedef struct
{
  uint32_t Len;
  uint8_t Data[TMsg_MaxLen];
} TMsg;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
