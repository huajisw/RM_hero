/**
  ****************************(C) COPYRIGHT 2020 HRBUST_AIR****************************
  * @file    main.c
	* @brief   stm32��ʼ���Լ���ʼ����freeRTOS��h�ļ��������ȫ�ֺ궨��            
  * @author  JackyJuu , HRBUST_AIR_TEAM , website:www.airclub.tech
	* @Note 	 
  * @version V1.0.0
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     11-18-2020      JackyJuu            Done
  ****************************(C) COPYRIGHT 2020 HRBUST_AIR****************************
	* @describe �����������
*/
#include "main.h"
#include "delay.h"
#include "flash.h"
#include "power.h"

#include "wit_c_sdk.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "Start_Task.h"
#include "IMU_Task.h"
#include "usb_usart.h"
#include "PC_usart.h"

#include "DJI_Remote_Control.h"

int main(void)
{ 
	delay_init(configTICK_RATE_HZ);
	delay_ms(10);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); 
	__disable_irq();
	Usb_Init();	
	Led_Init();
	
	remote_control_init();
	
	Adc_Init();
	Power_24V_Init();

	UART7_Init();
	
	WT61_Init();

	TIM1_Init();
	TIM2_Init();
	TIM4_Init();
	TIM5_Init();
	TIM8_Init();
	
	CAN1_Init();
	CAN2_Init();
	
	key_init();
	Laser_Init();
	Shoot_key_init();
	__enable_irq();
	
	startTast();
	vTaskStartScheduler();//�����������
	while(1)             
	{


	}
}
	

