/**
  ******************************************************************************
  * @file    
  * @author  sy
  * @brief
  * @date     
  ******************************************************************************
  * @attention
  *
  * Copyright (c) CSU_RM_FYT.
  * All rights reserved.
  *
  * This software component is licensed by SY under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  * opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* includes ------------------------------------------------------------------*/
#include "motor_chassis.h"
#include "message.h"
#include "can.h"
#include "pid.h"
#include "tim.h"
#include <string.h>
#include "usart.h"
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
Motor_Chassis_t Chassis = {0};

/* �ٶȻ� */
float Chassis_Speed_Kp   = 2.0f; /*< �ٶȻ�P���� */
float Chassis_Speed_Ki   = 0.0f; /*< �ٶȻ�I���� */
float Chassis_Speed_Kd   = 0.0f; /*< �ٶȻ�D���� */
float Chassis_Speed_Inc_Limit = 16000.0f; /*< �ٶȻ������޷� */

/* ������ */
float Chassis_Current_Kp = 2.0f; /*< ������P���� */
float Chassis_Current_Ki = 0.0f; /*< ������I���� */
float Chassis_Current_Kd = 0.0f; /*< ������D���� */
float Chassis_Current_Inc_Limit = 16000.0f; /*< �����������޷� */

float Chassis_Out_Limit = 16000.0f; /*< ����޷���3508������16384*/
/* function ------------------------------------------------------------------*/

/**
  * @brief  ��ȡ���̵����������
  * @param  ң������Ϣ�ṹ��
  * @retval void
  * @attention 
  */

void Motor_Chassis_GetMoveData(RemoteData_t RDMsg)
{
	 Chassis.M3508.TarSpeed = 850; //2 * RDMsg.Ch0;
}
    
    
/**
  * @brief  ���̵��PID��ʼ��
  * @param  void
  * @retval void
  * @attention 
  */
void Motor_Chassis_Init(void)
{
		  pid_init_increment(&Chassis.M3508.PidSpeed,Chassis_Speed_Kp,
                           Chassis_Speed_Ki,Chassis_Speed_Kd,Chassis_Speed_Inc_Limit);
			pid_init_increment(&Chassis.M3508.PidCurrent,Chassis_Current_Kp,
                           Chassis_Current_Ki,Chassis_Current_Kd,Chassis_Current_Inc_Limit);
}

/**
  * @brief  ���̵��PID���
  * @param  void
  * @retval void
  * @attention ��ͨ���ٶȻ����⻷������ֵ�ټ����������ͨ�����������ڻ���ֱ�ӿ����������rx,lpf???��
  */
void Motor_Chassis_PidRun(void)
{

    
        UART2_SendWave(5, 2, &Chassis.M3508.TarSpeed, &Chassis.M3508.Rx.Speed,
                             &Chassis.M3508.TarCurrent, &Chassis.M3508.Rx.Current,
                             &Chassis.M3508.LPf.Output);
    

    #ifdef CHASSIS_DEBUG
        Chassis_Init();
    #endif

        Chassis.M3508.LPf.Speed = 0.8 * Chassis.M3508.Rx.Speed + 0.2 * Chassis.M3508.LPf.Speed; 
        Chassis.M3508.TarCurrent = pid_increment_update(Chassis.M3508.TarSpeed, Chassis.M3508.LPf.Speed, &Chassis.M3508.PidSpeed);
        Chassis.M3508.LPf.TarCurrent = 0.8 * Chassis.M3508.TarCurrent + 0.2 * Chassis.M3508.LPf.TarCurrent;   
        Chassis.M3508.LPf.Current = 0.8 * Chassis.M3508.Rx.Current + 0.2 * Chassis.M3508.LPf.Current;
        Chassis.M3508.Output = pid_increment_update(Chassis.M3508.LPf.TarCurrent, Chassis.M3508.LPf.Current, &Chassis.M3508.PidCurrent);
        Chassis.M3508.LPf.Output = 0.8 * Chassis.M3508.Output + 0.2 * Chassis.M3508.LPf.Output;
}

/**
  * @brief  ���̵��CAN����
  * @param  void
  * @retval void
  * @attention ���ж�����
  */
void Motor_Chassis_CanTransmit(void)
{
			if(Observer.Tx.DR16_Rate>15) /*< ң����������������16ʱ�ſ������� */
			{
         /* ����޷� */
         Chassis.M3508.LPf.Output = constrain_int16_t(Chassis.M3508.LPf.Output,
                                               -Chassis_Out_Limit, Chassis_Out_Limit);
            
         /* CAN ��ֵ */
         Chassis.CanData[0]=(uint8_t)(Chassis.M3508.LPf.Output>>8);
        Chassis.CanData[1]=(uint8_t)(Chassis.M3508.LPf.Output);
			}
			else
      {
         Motor_Chassis_Speed_Reset();
      }
      CAN1_Transmit(0x200,Chassis.CanData);
}

int16_t constrain_int16_t(int16_t amt, int16_t low, int16_t high)
{
	return amt<low? low:(amt>high? high:amt); /*< �ʺű��ʽ���б�if else�� */
}


/**
  * @brief  ���̽���
  * @param  ����ָ��ṹ��
  * @retval void
* @attention 
  */
void Chassis_Process(RemoteData_t RDMsg)
{
    Motor_Chassis_GetMoveData(RDMsg);
    Motor_Chassis_PidRun();
}

/*********************************���̸�������*********************************/


/**
  * @brief  PID�������
  * @param  void
  * @retval void
  * @attention 
  */
void Motor_Chassis_Speed_Reset(void)
{
    memset(Chassis.CanData,0,sizeof(Chassis.CanData));
		Chassis.M3508.TarSpeed            = 0;
    Chassis.M3508.PidSpeed.errNow     = 0;
    Chassis.M3508.PidSpeed.errOld1    = 0;
    Chassis.M3508.PidSpeed.errOld2    = 0;
    Chassis.M3508.PidSpeed.ctrOut     = 0;
    Chassis.M3508.PidSpeed.dCtrOut    = 0;
    Chassis.M3508.PidCurrent.errNow   = 0;
    Chassis.M3508.PidCurrent.errOld1  = 0;
    Chassis.M3508.PidCurrent.errOld2  = 0;
    Chassis.M3508.PidCurrent.dCtrOut  = 0;
    Chassis.M3508.PidCurrent.ctrOut   = 0;
		memset(Chassis.CanData,0,sizeof(Chassis.CanData));
}

/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
