#ifndef __CAN_H__
#define __CAN_H__

#include "main.h"

#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

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


typedef enum
{
	CAN1_RX = 0,
	CAN2_RX = 1
}CAN_Set_t;

#define M2006_Speed_Change 0.0004629630

#define Fric_Motor_Speed_Change 0.0031415926

struct dipandianji
{
	int angle;//��е�Ƕ�
	int speed;//�ٶ�
	int dianliu;
};

//���������Ϣ
typedef struct 
{
	uint16_t Voilt_In;//�����ѹ
	uint16_t Voilt_C;//���ݵ�ѹ
	uint16_t Current_In;//�������
	uint16_t Target_power;//Ŀ�깦��
}Super_C_Msg_t;

typedef struct 
{
	int16_t angle;//��е�Ƕ�
	int16_t speed;//�ٶ�
	int16_t dianliu;
	int16_t temp;
	int16_t Last_Angle;
	int Angle_Long;
}Motor_Msg_t;

struct yuntaidianji
{
	int angle;//�����е�Ƕ�
	int speed;//����ٶ�
	int dianliu;//ʵ�ʵ�����С
	int temperature;//����¶�
};

void CAN1_Init(void);
void CAN2_Init(void);

void CAN1_Motor_Control(int16_t stdid,u16 num1,u16 num2,u16 num3,u16 num4);
void CAN2_Motor_Control(int16_t stdid,u16 num1,u16 num2,u16 num3,u16 num4);

void CAN2_Supercapacitors_Send_0x2E(int16_t Stdid,u16 num1,u16 num2,u16 num3,u16 num4);
void CAN2_Supercapacitors_Send_0x2F(int16_t Stdid,u16 num1,u16 num2,u16 num3,u16 num4);


void Can1ReceiveMsgProcess(CanRxMsg *can_receive_message);
void Can2ReceiveMsgProcess(CanRxMsg *can_receive_message);

void CAN1_Super_C_Send(int16_t Stdid,uint16_t data);

void Angle_Count_Long(Motor_Msg_t *ms);

Motor_Msg_t *Get_DJI_Motor_Data(CAN_Set_t CAN_Set,uint16_t Motor_Stdid);
Super_C_Msg_t *Get_Cap_Data(void);

#endif 
