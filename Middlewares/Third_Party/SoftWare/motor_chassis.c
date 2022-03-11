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
//#define CHASSIS_DEBUG

//#define CHASSIS_WAVE

#define SUP   1
#define SMID  3
#define SDOWN 2
/* variables -----------------------------------------------------------------*/
Motor_Chassis_t Chassis = {0};

/* �ٶȻ� */
float Chassis_Speed_Kp[4]={1.0f,1.0f,1.0f,1.0f};/*< �ٶȻ�P���� */
float Chassis_Speed_Ki[4]={0.0f,0.0f,0.0f,0.0f};/*< �ٶȻ�I���� */
float Chassis_Speed_Kd[4]={0.0f,0.0f,0.0f,0.0f};/*< �ٶȻ�D���� */

float Chassis_Speed_Inc_Limit = 16000.0f; /*< �ٶȻ������޷� */

float Chassis_Out_Limit = 16000.0f; /*< ����޷���3508������16384*/

/* function ------------------------------------------------------------------*/

/**
  * @brief  ��ȡ��������������
  * @param  ң������Ϣ�ṹ��
  * @retval void
  * @attention 
  */

void Motor_Chassis_GetMoveData(RemoteData_t RDMsg)
{
	 Chassis.M3508[0].TarSpeed = 10*RDMsg.Ch0;
	 Chassis.M3508[1].TarSpeed = 10*RDMsg.Ch0;
}
    
    
/**
  * @brief  ������PID��ʼ��
  * @param  voi
  * @retval void
  * @attention 
  */
void Motor_Chassis_Init(void)
{
    uint8_t i;
	
    for(i=0;i<4;i++)
    {
        pid_init_increment(&Chassis.M3508[i].PidSpeed,Chassis_Speed_Kp[i],
                           Chassis_Speed_Ki[i],Chassis_Speed_Kd[i],Chassis_Speed_Inc_Limit);
    }
}

/**
  * @brief  ������PID���
  * @param  void
  * @retval void
  * @attention ��ͨ���ٶȻ�ֱ�ӿ����������rx,lpf???��
  */
void Motor_Chassis_PidRun(void)
{
    uint8_t i;

    #ifdef CHASSIS_WAVE
        UART2_SendWave(8, 2, &Chassis.M3508[0].TarSpeed, &Chassis.M3508[0].Rx.Speed,
														&Chassis.M3508[1].TarSpeed, &Chassis.M3508[1].Rx.Speed,
														&Chassis.M3508[2].TarSpeed, &Chassis.M3508[2].Rx.Speed,
														&Chassis.M3508[3].TarSpeed, &Chassis.M3508[3].Rx.Speed
                             );
    #endif

    #ifdef CHASSIS_DEBUG
           //Motor_Chassis_Init();
    #endif

    for (i = 0; i < 4; i++)     //rx speed lpf
    {
        Chassis.M3508[i].LPf.Speed = 0.8 * Chassis.M3508[i].Rx.Speed + 0.2 * Chassis.M3508[i].LPf.Speed;
    }   
    for (i = 0; i < 4; i++)     //speed loop
    {
        Chassis.M3508[i].Output = pid_increment_update(Chassis.M3508[i].TarSpeed, Chassis.M3508[i].LPf.Speed, &Chassis.M3508[i].PidSpeed);
    }
		for (i = 0; i < 4; i++)     //out lpf
    {
        Chassis.M3508[i].LPf.Output = 0.8 * Chassis.M3508[i].Output + 0.2 * Chassis.M3508[i].LPf.Output;
    }
}

/**
  * @brief  ������CAN����
  * @param  void
  * @retval void
  * @attention ���ж�����
  */
void Motor_Chassis_CanTransmit(void)
{
    uint8_t i;
    if(Observer.Tx.DR16_Rate>15) /*< ң����������������16ʱ�ſ������� */
    {
        for(i=0;i<4;i++)
        {
            /* ����޷� */
            Chassis.M3508[i].LPf.Output = constrain_int16_t(Chassis.M3508[i].LPf.Output,
                                                    -Chassis_Out_Limit, Chassis_Out_Limit);
            
            /* CAN ��ֵ */
            Chassis.CanData[2*i]=(uint8_t)(Chassis.M3508[i].LPf.Output>>8);
            Chassis.CanData[2*i+1]=(uint8_t)(Chassis.M3508[i].LPf.Output);
        }
    }
    else
    {
        Motor_Chassis_Speed_Reset(); /*< �ر�ң�����󣬷���Ŀ���ٶ�һֱ���ֵ�ǰ״̬��0�� */
    }
    CAN1_Transmit(0x200,Chassis.CanData);
}

int16_t constrain_int16_t(int16_t amt, int16_t low, int16_t high)
{
	return amt<low? low:(amt>high? high:amt); /*< �ʺű��ʽ���б�if else�� */
}


/**
  * @brief  �������
  * @param  ����ָ��ṹ��
  * @retval void
* @attention 
  */
void Chassis_Process(RemoteData_t RDMsg)
{
    Motor_Chassis_GetMoveData(RDMsg);
    Motor_Chassis_PidRun();
		Motor_Chassis_CanTransmit();
}

/*********************************���渨������*********************************/


/**
  * @brief  PID�������
  * @param  void
  * @retval void
  * @attention 
  */
void Motor_Chassis_Speed_Reset(void)
{
    uint8_t i;
    
    for(i = 0; i < 4; i++)
    {
        Chassis.M3508[i].TarSpeed            = 0;
        Chassis.M3508[i].PidSpeed.errNow     = 0;
        Chassis.M3508[i].PidSpeed.errOld1    = 0;
        Chassis.M3508[i].PidSpeed.errOld2    = 0;
        Chassis.M3508[i].PidSpeed.ctrOut     = 0;
        Chassis.M3508[i].PidSpeed.dCtrOut    = 0;
        Chassis.M3508[i].PidCurrent.errNow   = 0;
        Chassis.M3508[i].PidCurrent.errOld1  = 0;
        Chassis.M3508[i].PidCurrent.errOld2  = 0;
        Chassis.M3508[i].PidCurrent.dCtrOut  = 0;
        Chassis.M3508[i].PidCurrent.ctrOut   = 0;
    }
    memset(Chassis.CanData,0,sizeof(Chassis.CanData));
}

/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
