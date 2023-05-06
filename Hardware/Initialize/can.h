#ifndef __CAN_H__
#define __CAN_H__

#include "main.h"

#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

#include "FreeRTOS.h"

#define Firc_Motor_Left 0x201
#define Firc_Motor_Right 0x202
#define Firc_Motor_UP 0x203

#define Trigger_Motor_Can 0x207

#define Gimbal_Yaw_Can 0x205
#define Gimbal_Pitch_Can 0x206

#define Chassis_Motor_RI 0x201
#define Chassis_Motor_LI 0x202
#define Chassis_Motor_LB 0x203
#define Chassis_Motor_RB 0x204

#define JUDGE_OFFLINE_TIME_LIMIT 1000

typedef enum
{
	CAN1_RX = 0,
	CAN2_RX = 1
}CAN_Set_t;


//电容相关信息
typedef struct 
{
	int16_t Cap_I;//电容电流
	int16_t Cap_V;//电容电压
	uint16_t Cap_State;//电容状态
	uint32_t Timestamp;
}Super_C_Msg_t;

typedef struct 
{
	int16_t angle;//机械角度
	int16_t speed;//速度
	int16_t current;
	int16_t temp;
}Motor_Msg_t;

typedef __packed struct
{
		uint16_t shooter_id1_42mm_cooling_rate;
		uint16_t shooter_id1_42mm_cooling_limit;
		uint16_t shooter_id1_42mm_speed_limit;
		uint16_t chassis_power_limit;
}Judge_Game_Robot_Status_t;

typedef __packed struct
{
		float chassis_power; 
		uint16_t chassis_power_buffer;
		uint16_t shooter_id1_42mm_cooling_heat;
}Judge_Power_Heat_Data_t;

typedef struct
{
		Judge_Game_Robot_Status_t Judge_game_robot_status;
		Judge_Power_Heat_Data_t Judge_power_heat_data;
		uint32_t Timestamp; 
}Judge_Info_t;

void CAN1_Init(void);
void CAN2_Init(void);

void CAN1_Motor_Control(int16_t stdid,uint16_t num1,uint16_t num2,uint16_t num3,uint16_t num4);
void CAN2_Motor_Control(int16_t stdid,uint16_t num1,uint16_t num2,uint16_t num3,uint16_t num4);
void CAN1_SuperCap_Control(int16_t Stdid,uint16_t num1,uint16_t num2,uint16_t num3,uint16_t num4);
void CAN1_Graphic_Data_Send(int16_t Stdid,uint8_t num1,uint8_t num2,uint8_t num3,uint8_t num4);

void Can1ReceiveMsgProcess(CanRxMsg *can_receive_message);
void Can2ReceiveMsgProcess(CanRxMsg *can_receive_message);

void Angle_Count_Long(Motor_Msg_t *ms);

Motor_Msg_t *Get_DJI_Motor_Data(CAN_Set_t CAN_Set,uint16_t Motor_Stdid);
Super_C_Msg_t *Get_Cap_Data(void);
uint8_t Is_Judge_Online(void);
Judge_Info_t* Get_Judge_Info(void);

#endif 
