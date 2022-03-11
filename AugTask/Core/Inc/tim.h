/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim14;

/* USER CODE BEGIN Private defines */
typedef struct _rate_t
{
    uint16_t CAN1_Tx_Rate;
    uint8_t CAN1_0x201_Rate;
    uint8_t CAN1_0x202_Rate;
    uint8_t CAN1_0x203_Rate;
    uint8_t CAN1_0x204_Rate;
    
    uint8_t  DR16_Rate;
}rate_t;

typedef struct _obser_t
{
    rate_t Tx;
    rate_t Rx;
}obser_t;

/* USER CODE END Private defines */

void MX_TIM14_Init(void);

/* USER CODE BEGIN Prototypes */
extern obser_t Observer;
void TIM14_START(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
