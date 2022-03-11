/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "motor_chassis.h"
#include "motor.h"
#define F407_CAN_ID 0x200
#define CAN1_FIFO CAN_RX_FIFO0

uint8_t CAN1_0x201_Rate = 0;
uint8_t CAN1_0x202_Rate = 0;
uint8_t CAN1_0x203_Rate = 0;
uint8_t CAN1_0x204_Rate = 0;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = CAN1_RX_Pin|CAN1_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, CAN1_RX_Pin|CAN1_TX_Pin);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void CAN1_FilterInit(void)
{
    CAN_FilterTypeDef fcan; //定义接收过滤器类型结构体 fcan
	
    fcan.FilterBank = 0; //使用0号过滤器
    fcan.FilterMode = CAN_FILTERMODE_IDMASK; //采用掩码模式
    fcan.FilterScale = CAN_FILTERSCALE_32BIT; //设置32位宽
    
    fcan.FilterIdHigh = 0; //设置验证码为0，高低各4位
    fcan.FilterIdLow = 0;
    fcan.FilterMaskIdHigh = 0; //设置屏蔽码为0，高低各4位
    fcan.FilterMaskIdLow = 0;
    fcan.FilterFIFOAssignment = CAN1_FIFO; //FIFO0
	
    fcan.FilterActivation = ENABLE;// 激活过滤器
    fcan.SlaveStartFilterBank = 0; // 从过滤器CAN2
    
    HAL_CAN_ConfigFilter(&hcan1,&fcan); //配置过滤器
    HAL_CAN_Start(&hcan1); //开启CAN通信
    
    HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY); //使能中断
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) 
{
    UNUSED(hcan);
    uint8_t Data[8];
    CAN_RxHeaderTypeDef RxMsg;
	  if(hcan->Instance == CAN1)
		{
			HAL_CAN_GetRxMessage(&hcan1, CAN1_FIFO, &RxMsg, Data);
			switch(RxMsg.StdId)
        {
            case 0x201:
                CAN1_0x201_Rate++;
                Chassis.M3508[0].Rx.Speed   = Data[2] << 8 | Data[3];
                Chassis.M3508[0].Rx.Current = Data[4] << 8 | Data[5];
//                Motor_Chassis_CanTransmit();
            break;
            case 0x202:
                CAN1_0x202_Rate++;
                Chassis.M3508[1].Rx.Speed   = Data[2] << 8 | Data[3];
                Chassis.M3508[1].Rx.Current = Data[4] << 8 | Data[5];
            break;
            case 0x203:
                CAN1_0x203_Rate++;
                Chassis.M3508[2].Rx.Speed   = Data[2] << 8 | Data[3];
                Chassis.M3508[2].Rx.Current = Data[4] << 8 | Data[5];            
            break;
            case 0x204:
                CAN1_0x204_Rate++;
                Chassis.M3508[3].Rx.Speed   = Data[2] << 8 | Data[3];
                Chassis.M3508[3].Rx.Current = Data[4] << 8 | Data[5];
                
            break;

						default:
						break;

        }
		}
}

CAN_TxHeaderTypeDef CAN1TxMsg;
void CAN1_Transmit(uint16_t ID,uint8_t *pData)
{
	  uint32_t MailBox;
    CAN1TxMsg.StdId = ID;
    CAN1TxMsg.ExtId = 0;
    CAN1TxMsg.IDE = CAN_ID_STD;
    CAN1TxMsg.RTR = CAN_RTR_DATA;
    CAN1TxMsg.DLC = 8;
	  HAL_CAN_AddTxMessage(&hcan1,&CAN1TxMsg,pData,&MailBox); 
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
