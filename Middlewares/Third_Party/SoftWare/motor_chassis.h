/**
  ******************************************************************************
  * @file    
  * @author  sy
  * @brief   
  * @date     
  ******************************************************************************
  * @attention
  *
  * Copyright (c) CSU_RM_FYT
  * All rights reserved.
  *
  * This software component is licensed by SY under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  * opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MOTOR_CHASSIS_H
#define _MOTOR_CHASSIS_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "pid.h"
#include "message.h"
/* typedef -------------------------------------------------------------------*/
typedef struct _MotorData_t
{
    int16_t Angle;
    int16_t Speed;
    int16_t Current;
    uint8_t Tep;
    int16_t TarCurrent;
    int16_t Output;
}MotorData_t;

typedef struct _M3508_t
{
    int16_t TarSpeed;
    int16_t TarCurrent;
    MotorData_t Rx;
    MotorData_t LPf;
    PID_IncrementType PidSpeed;
    PID_IncrementType PidCurrent;
    int16_t Output;
}M3508_t;

typedef struct _Motor_Chassis_t
{
    M3508_t M3508[4];
    uint8_t CanData[8];
}Motor_Chassis_t;
/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
extern Motor_Chassis_t Chassis;
/* function ------------------------------------------------------------------*/


void Motor_Chassis_Init(void);
void Motor_Chassis_PidRun(void);
	
int16_t constrain_int16_t(int16_t amt, int16_t low, int16_t high); 
void Motor_Chassis_CanTransmit(void);
void Motor_Chassis_Speed_Reset(void);

void Chassis_Process(RemoteData_t RDMsg);
#ifdef __cplusplus
}
#endif

#endif /* */
  
/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
