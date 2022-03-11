/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "message.h"
#include "motor_chassis.h"
#include "can.h"
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
osEvent evt;
/* USER CODE END Variables */
osThreadId MessageHandle;
osThreadId ChassisMotorHandle;
osMessageQId MsgToCMHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void taskMessage(void const * argument);
void taskChassisMotor(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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

  /* Create the queue(s) */
  /* definition and creation of MsgToCM */
  osMessageQDef(MsgToCM, 16, uint32_t);
  MsgToCMHandle = osMessageCreate(osMessageQ(MsgToCM), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Message */
  osThreadDef(Message, taskMessage, osPriorityNormal, 0, 128);
  MessageHandle = osThreadCreate(osThread(Message), NULL);

  /* definition and creation of ChassisMotor */
  osThreadDef(ChassisMotor, taskChassisMotor, osPriorityRealtime, 0, 512);
  ChassisMotorHandle = osThreadCreate(osThread(ChassisMotor), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_taskMessage */
/**
  * @brief  Function implementing the Message thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_taskMessage */
void taskMessage(void const * argument)
{
  /* USER CODE BEGIN taskMessage */
  /* Infinite loop */
  for(;;)
  {
      RemoteData_t RDMsg;
      RemoteDataMsg_Process(&RDMsg);
      osMessagePut(MsgToCMHandle, (uint32_t)&RDMsg, 0);
      osDelay(10);
  }
  /* USER CODE END taskMessage */
}

/* USER CODE BEGIN Header_taskChassisMotor */
/**
* @brief Function implementing the ChassisMotor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_taskChassisMotor */
void taskChassisMotor(void const * argument)
{
  /* USER CODE BEGIN taskChassisMotor */
	RemoteData_t *RDMsg;
  Motor_Chassis_Init();
  /* Infinite loop */
  for(;;)
  {
		 evt = osMessageGet(MsgToCMHandle,0);
     if(evt.status == osEventMessage)
     {
        RDMsg = (RemoteData_t*)evt.value.v;
     } 
		 Chassis_Process(*RDMsg);
    osDelay(5);
  }
  /* USER CODE END taskChassisMotor */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
