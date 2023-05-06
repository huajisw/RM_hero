/**
  ****************************(C) COPYRIGHT 2021 HRBUST_AIR****************************
  * @file    judge_task.c
	* @brief   裁判系统任务           
  * @author  JackyJuu , HRBUST_AIR_TEAM , website:www.airclub.tech
	* @Note 	 定期发送
  * @version V1.0.0
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     12-4-2020      JackyJuu            Done
  *  V1.2.0      4-2-2020      JackyJuu            Done	
  ****************************(C) COPYRIGHT 2021 HRBUST_AIR****************************
	* @describe OLED显示屏任务
*/
#include "judge_task.h"

#include "chassis_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"

#include "DJI_Remote_Control.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

extern Gimbal_t Gimbal;
extern Chassis_t Chassis;
extern Shoot_t Shoot;
uint8_t Graphic_Data[8];


void Judge_Task(void *pvParameters)
{
		while(1)
		{
				CAN1_Graphic_Data_Send(0x022,Chassis.Chassis_Mode,Shoot.Shoot_Mode,0,0);
				vTaskDelay(50);
		}
}
