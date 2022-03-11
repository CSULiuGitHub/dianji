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

/* 速度环 */
float Chassis_Speed_Kp[4]={1.0f,1.0f,1.0f,1.0f};/*< 速度环P参数 */
float Chassis_Speed_Ki[4]={0.0f,0.0f,0.0f,0.0f};/*< 速度环I参数 */
float Chassis_Speed_Kd[4]={0.0f,0.0f,0.0f,0.0f};/*< 速度环D参数 */

float Chassis_Speed_Inc_Limit = 16000.0f; /*< 速度环增量限幅 */

float Chassis_Out_Limit = 16000.0f; /*< 输出限幅，3508最大接收16384*/

/* function ------------------------------------------------------------------*/

/**
  * @brief  获取翻面电机控制数据
  * @param  遥控器消息结构体
  * @retval void
  * @attention 
  */

void Motor_Chassis_GetMoveData(RemoteData_t RDMsg)
{
	 Chassis.M3508[0].TarSpeed = 10*RDMsg.Ch0;
	 Chassis.M3508[1].TarSpeed = 10*RDMsg.Ch0;
}
    
    
/**
  * @brief  翻面电机PID初始化
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
  * @brief  翻面电机PID输出
  * @param  void
  * @retval void
  * @attention （通过速度环直接控制输出）（rx,lpf???）
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
  * @brief  翻面电机CAN发送
  * @param  void
  * @retval void
  * @attention 放中断里面
  */
void Motor_Chassis_CanTransmit(void)
{
    uint8_t i;
    if(Observer.Tx.DR16_Rate>15) /*< 遥控器保护，数据量16时才开启控制 */
    {
        for(i=0;i<4;i++)
        {
            /* 输出限幅 */
            Chassis.M3508[i].LPf.Output = constrain_int16_t(Chassis.M3508[i].LPf.Output,
                                                    -Chassis_Out_Limit, Chassis_Out_Limit);
            
            /* CAN 赋值 */
            Chassis.CanData[2*i]=(uint8_t)(Chassis.M3508[i].LPf.Output>>8);
            Chassis.CanData[2*i+1]=(uint8_t)(Chassis.M3508[i].LPf.Output);
        }
    }
    else
    {
        Motor_Chassis_Speed_Reset(); /*< 关闭遥控器后，翻面目标速度一直保持当前状态（0） */
    }
    CAN1_Transmit(0x200,Chassis.CanData);
}

int16_t constrain_int16_t(int16_t amt, int16_t low, int16_t high)
{
	return amt<low? low:(amt>high? high:amt); /*< 问号表达式运行比if else快 */
}


/**
  * @brief  翻面进程
  * @param  控制指令结构体
  * @retval void
* @attention 
  */
void Chassis_Process(RemoteData_t RDMsg)
{
    Motor_Chassis_GetMoveData(RDMsg);
    Motor_Chassis_PidRun();
		Motor_Chassis_CanTransmit();
}

/*********************************翻面辅助函数*********************************/


/**
  * @brief  PID输出清零
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
