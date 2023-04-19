/**
  ****************************(C) COPYRIGHT 2021 HRBUST_AIR****************************
  * @file    judge_task.c
	* @brief   ����ϵͳ����           
  * @author  JackyJuu , HRBUST_AIR_TEAM , website:www.airclub.tech
	* @Note 	 ���ڷ���
  * @version V1.0.0
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     12-4-2020      JackyJuu            Done
  *  V1.2.0      4-2-2020      JackyJuu            Done	
  ****************************(C) COPYRIGHT 2021 HRBUST_AIR****************************
	* @describe OLED��ʾ������
*/
#include "judge_task.h"

#include "chassis_task.h"

#include "Judge_Data.h"
#include "DJI_Remote_Control.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "key.h"

#define Robot_Com_Id 0x0103

Judge_Show_Msg_t Judge_Show_Msg;

uint8_t Chassis_Mode_Word[8][40] = {"Chassis_Zero",//�޿��ƣ�����0
																	"Chassis_Motionless",//�޿��ƣ��õ���Ƕȱ��ֵ�ǰλ��
																	"Chassis_RC_Control",//ң��������
																	"Chassis_Follow_Gimbal",//���̸�����̨
																	"Chassis_Follow_Chassis",//�Ҹ����Լ�
																	"Chassis_Spin_Right",
																	"Chassis_Spin_Left",
																	"Chassis_Spin"};//С����ģʽ

uint8_t Gimbal_Mode_Word[7][40] = {"Gimbal_Zero",//�޿��ƣ�����0
																	"Gimbal_Motionless",//�޿��ƣ��õ���Ƕȱ��ֵ�ǰλ��
																	"Gimbal_Motor_Control",//����Ƕȿ���
																	"Gimbal_IMU_Control",//�����ǽǶȿ���
																	"Gimbal_Vision_Control",//�Ӿ����ƽӿ�
																	"Gimbal_Follow_Chassis",//��̨������̣�yaw��̶�����
																	"Gimbal_Spin"};//С����ģʽ	

uint8_t Super_Word[40] = {"SuperC_Limit: %"};

extern DJI_Judge_t DJI_Judge; 
extern RC_Ctl_t RC_Ctl_Data;

uint8_t Judge_Test_Send_Data[6] = {0x11,0x22,0x33,0x44,0x55,0x66};
uint8_t Judge_Test_Graphic_Name[] = {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99};

//uint8_t	Judge_Test_Graphic_Data[200];

int test_size;

uint32_t test_judge_graphic_data;

int write_mode = 0;

//uint8_t	Judge_Test_Graphic_Data[] = "Hello World";
uint8_t Judge_Test_Graphic_Data_Send[45];

float test_nummmm = 66.66;
int32_t test_nummmmmm = 12888;

uint8_t	Judge_Super_Cap_Data_Send[] = "Super-Cap:";

uint8_t Judge_Test_Graphic_Data[3][2][30] = {{"DN1","Super-Cap:"},\
																						 {"DN2","Version_On"},\
																						 {"DN3","Version_Off"}};

int Judge_Set_Init[10][4] = {{3,20,750,780},//��������������ʾ
														 {3,20,750,740},//�Ӿ������Ƿ��鵽��ʾ
														 {2,15,800,400},//����ģʽ
														 {3,20,800,300},//��̨ģʽ
														 {3,20,800,300},
														 {3,20,800,300},
														 {3,20,800,300},
														 {3,20,800,300},
														 {3,20,800,300},
														 {3,20,800,300}};

int Location_x[10],Location_y[10];
int Data_Width[10],Data_Sizw[10];
int Judge_Num_Set;
int key_last;
														 
uint8_t Student_Judge_Send_Data[] = "FUCK YOU";
														 
int Judge_Data_Send_Update;
														 
void Judge_Send_Data_Update(uint8_t* Graphic_Update_Name)
{
	Judge_Character_operate_tpye_Set(&DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_client_custom_character,Graphic_Update_Name,Graphic_Add);
	Judge_Data_Send_To_Client(&DJI_Judge,0x0110,DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_Character_Data,Graphic_character_Long);	
	Judge_Character_operate_tpye_Set(&DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_client_custom_character,Graphic_Update_Name,Graphic_Add);
	Judge_Data_Send_To_Client(&DJI_Judge,0x0110,DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_Character_Data,Graphic_character_Long);	
}

//uint8_t Leida_Data[14] = {}
int Show_Num;

void Judge_Chassis_Mode_Show_Add(Chassis_Mode_Judge_Msg_Show_t* Judge_Msg_Show,int Locate_X,int Locate_Y)
{
		Show_Num = (int)*Judge_Msg_Show->Chassis_Mode_Show;
		Judge_Character_Data_Set(&DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_client_custom_character,Judge_Msg_Show->Chassis_Mode_Name,\
		Character,Graphic_Add,Judge_Chassis_Show_Floor,3,15,14,2,Locate_X,Locate_Y,Judge_Msg_Show->Chassis_Mode_Word_Add[Show_Num]);
		Judge_Data_Send_To_Client(&DJI_Judge,0x0110,DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_Character_Data,Graphic_character_Long);	
}

void Judge_Gimbal_Mode_Show_Add(Gimbal_Mode_Judge_Msg_Show_t* Judge_Msg_Show,int Locate_X,int Locate_Y)
{
		Show_Num = (int)*Judge_Msg_Show->Gimbal_Mode_Show;
		Judge_Character_Data_Set(&DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_client_custom_character,Judge_Msg_Show->Gimbal_Mode_Name,\
		Character,Graphic_Add,Judge_Gimbal_Show_Floor,3,15,14,2,Locate_X,Locate_Y,Judge_Msg_Show->Gimbal_Mode_Word_Add[Show_Num]);
		Judge_Data_Send_To_Client(&DJI_Judge,0x0110,DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_Character_Data,Graphic_character_Long);	
}


void Judge_Chassis_Mode_Show_Set(Chassis_Mode_Judge_Msg_Show_t* Judge_Msg_Show,int Locate_X,int Locate_Y)
{
		Show_Num = (int)*Judge_Msg_Show->Chassis_Mode_Show;
		Judge_Character_Data_Set(&DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_client_custom_character,Judge_Msg_Show->Chassis_Mode_Name,\
		Character,Graphic_Change,Judge_Chassis_Show_Floor,3,15,14,2,Locate_X,Locate_Y,Judge_Msg_Show->Chassis_Mode_Word_Add[Show_Num]);
		Judge_Data_Send_To_Client(&DJI_Judge,0x0110,DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_Character_Data,Graphic_character_Long);	
}

void Judge_Gimbal_Mode_Show_Set(Gimbal_Mode_Judge_Msg_Show_t* Judge_Msg_Show,int Locate_X,int Locate_Y)
{
		Show_Num = (int)*Judge_Msg_Show->Gimbal_Mode_Show;
		Judge_Character_Data_Set(&DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_client_custom_character,Judge_Msg_Show->Gimbal_Mode_Name,\
		Character,Graphic_Change,Judge_Gimbal_Show_Floor,3,15,14,2,Locate_X,Locate_Y,Judge_Msg_Show->Gimbal_Mode_Word_Add[Show_Num]);
		Judge_Data_Send_To_Client(&DJI_Judge,0x0110,DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_Character_Data,Graphic_character_Long);	
}

void Judge_Vision_Show_Add(int Vision_flag,int Locate_X,int Locate_Y)
{
	if(Vision_flag == 1)
	{
		Judge_Character_Data_Set(&DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_client_custom_character,(uint8_t*)"VSD",\
		Character,Graphic_Add,Judge_Vision_Floor,3,15,14,2,Locate_X,Locate_Y,Judge_Test_Graphic_Data[1][1]);
		Judge_Data_Send_To_Client(&DJI_Judge,0x0110,DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_Character_Data,Graphic_character_Long);
		
	}
	else if(Vision_flag == 0)
	{
		Judge_Character_Data_Set(&DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_client_custom_character,(uint8_t*)"VSD",\
		Character,Graphic_Add,Judge_Vision_Floor,3,15,14,2,Locate_X,Locate_Y,Judge_Test_Graphic_Data[2][1]);
		Judge_Data_Send_To_Client(&DJI_Judge,0x0110,DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_Character_Data,Graphic_character_Long);		
	}
}

void Judge_Vision_Show_Set(int Vision_flag,int Locate_X,int Locate_Y)
{
	if(Vision_flag == 1)
	{
		Judge_Character_Data_Set(&DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_client_custom_character,(uint8_t*)"VSD",\
		Character,Graphic_Change,Judge_Vision_Floor,3,15,14,2,Locate_X,Locate_Y,Judge_Test_Graphic_Data[1][1]);
		Judge_Data_Send_To_Client(&DJI_Judge,0x0110,DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_Character_Data,Graphic_character_Long);	
	}
	else if(Vision_flag == 0)
	{
		Judge_Character_Data_Set(&DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_client_custom_character,(uint8_t*)"VSD",\
		Character,Graphic_Change,Judge_Vision_Floor,3,15,14,2,Locate_X,Locate_Y,Judge_Test_Graphic_Data[2][1]);
		Judge_Data_Send_To_Client(&DJI_Judge,0x0110,DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_Character_Data,Graphic_character_Long);		
	}
}

void Judge_Shoot_Line_Add()
{
	
			Judge_Graphic_Data_Set((graphic_data_struct_t*)&DJI_Judge.DJI_Judge_Send_Msg.Judge_Graphic_Client_Seven.grapic_data_struct[0],(uint8_t*)"LI1",\
			Straight_Line,Graphic_Add,0,2,0,0,5,920,430,0,1000,430);
			Judge_Graphic_Data_Set((graphic_data_struct_t*)&DJI_Judge.DJI_Judge_Send_Msg.Judge_Graphic_Client_Seven.grapic_data_struct[1],(uint8_t*)"LI2",\
			Straight_Line,Graphic_Add,1,3,0,0,5,900,443,0,1020,443);
			Judge_Graphic_Data_Set((graphic_data_struct_t*)&DJI_Judge.DJI_Judge_Send_Msg.Judge_Graphic_Client_Seven.grapic_data_struct[2],(uint8_t*)"LI3",\
			Straight_Line,Graphic_Add,2,4,0,0,5,880,420,0,1040,420);
			Judge_Graphic_Data_Set((graphic_data_struct_t*)&DJI_Judge.DJI_Judge_Send_Msg.Judge_Graphic_Client_Seven.grapic_data_struct[3],(uint8_t*)"LI4",\
			Straight_Line,Graphic_Add,3,5,0,0,5,860,400,0,1060,400);
			Judge_Graphic_Data_Set((graphic_data_struct_t*)&DJI_Judge.DJI_Judge_Send_Msg.Judge_Graphic_Client_Seven.grapic_data_struct[4],(uint8_t*)"LI5",\
			Straight_Line,Graphic_Add,4,6,0,0,5,840,395,0,1080,395);
			Judge_Graphic_Data_Set((graphic_data_struct_t*)&DJI_Judge.DJI_Judge_Send_Msg.Judge_Graphic_Client_Seven.grapic_data_struct[5],(uint8_t*)"LI6",\
			Straight_Line,Graphic_Add,5,7,0,0,3,960,540,0,960,360);
			Judge_Data_Send_To_Client(&DJI_Judge,0x0104,DJI_Judge.DJI_Judge_Send_Msg.Judge_Graphic_Client_Seven.Judge_client_custom_graphic_seven_Data,105);

}

void Judge_Shoot_Line_Set()
{
	
			Judge_Graphic_Data_Set((graphic_data_struct_t*)&DJI_Judge.DJI_Judge_Send_Msg.Judge_Graphic_Client_Seven.grapic_data_struct[0],(uint8_t*)"LI1",\
			Straight_Line,Graphic_Change,0,2,0,0,5,920,430,0,1000,430);
			Judge_Graphic_Data_Set((graphic_data_struct_t*)&DJI_Judge.DJI_Judge_Send_Msg.Judge_Graphic_Client_Seven.grapic_data_struct[1],(uint8_t*)"LI2",\
			Straight_Line,Graphic_Change,1,3,0,0,5,900,443,0,1020,443);
			Judge_Graphic_Data_Set((graphic_data_struct_t*)&DJI_Judge.DJI_Judge_Send_Msg.Judge_Graphic_Client_Seven.grapic_data_struct[2],(uint8_t*)"LI3",\
			Straight_Line,Graphic_Change,2,4,0,0,5,880,420,0,1040,420);
			Judge_Graphic_Data_Set((graphic_data_struct_t*)&DJI_Judge.DJI_Judge_Send_Msg.Judge_Graphic_Client_Seven.grapic_data_struct[3],(uint8_t*)"LI4",\
			Straight_Line,Graphic_Change,3,5,0,0,5,860,400,0,1060,400);
			Judge_Graphic_Data_Set((graphic_data_struct_t*)&DJI_Judge.DJI_Judge_Send_Msg.Judge_Graphic_Client_Seven.grapic_data_struct[4],(uint8_t*)"LI5",\
			Straight_Line,Graphic_Change,4,6,0,0,5,840,395,0,1080,395);
			Judge_Graphic_Data_Set((graphic_data_struct_t*)&DJI_Judge.DJI_Judge_Send_Msg.Judge_Graphic_Client_Seven.grapic_data_struct[5],(uint8_t*)"LI6",\
			Straight_Line,Graphic_Change,5,7,0,0,3,960,540,0,960,360);
			Judge_Data_Send_To_Client(&DJI_Judge,0x0104,DJI_Judge.DJI_Judge_Send_Msg.Judge_Graphic_Client_Seven.Judge_client_custom_graphic_seven_Data,105);

}

void Judge_Super_C_Show_Add(Super_C_Judge_Msg_Show_t* Super_C_Judge_Msg_Show,int Locate_X,int Locate_Y)
{

		Super_C_Judge_Msg_Show->Super_C_Limit = (int)(((float)(Super_C_Judge_Msg_Show->Super_C_Msg_Show->Voilt_C) - 1400.00f) / ((float)(Super_C_Judge_Msg_Show->Super_C_Msg_Show->Voilt_In) - 1400.00f) * 100.00f);
		
		Judge_Character_Data_Set(&DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_client_custom_character,Super_C_Judge_Msg_Show->Super_C_Name,\
		Character,Graphic_Add,Judge_Super_C_Show_Floor,3,15,14,2,Locate_X,Locate_Y,Super_C_Judge_Msg_Show->Super_C_Word);
		Judge_Data_Send_To_Client(&DJI_Judge,0x0110,DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_Character_Data,Graphic_character_Long);		
		Judge_Word_Data_Set(&DJI_Judge.DJI_Judge_Send_Msg.Judge_Graphic_Client_Single.Word_data_struct,Judge_Test_Graphic_Name,\
		Integer,Graphic_Add,Judge_Super_C_Show_Floor,3,15,14,2,Locate_X + 183,Locate_Y,0,Super_C_Judge_Msg_Show->Super_C_Limit);		
		Judge_Data_Send_To_Client(&DJI_Judge,0x0110,DJI_Judge.DJI_Judge_Send_Msg.Judge_Graphic_Client_Single.Judge_client_custom_graphic_single_Data,15);		
	
}

void Judge_Super_C_Show_Set(Super_C_Judge_Msg_Show_t* Super_C_Judge_Msg_Show,int Locate_X,int Locate_Y)
{
		Super_C_Judge_Msg_Show->Super_C_Limit = (int)(((float)(Super_C_Judge_Msg_Show->Super_C_Msg_Show->Voilt_C) - 1400.00f) / ((float)(Super_C_Judge_Msg_Show->Super_C_Msg_Show->Voilt_In) - 1400.00f) * 100.00f);
		Judge_Character_Data_Set(&DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_client_custom_character,Super_C_Judge_Msg_Show->Super_C_Name,\
		Character,Graphic_Change,Judge_Super_C_Show_Floor,3,15,14,2,Locate_X,Locate_Y,Super_C_Judge_Msg_Show->Super_C_Word);
		Judge_Data_Send_To_Client(&DJI_Judge,0x0110,DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_Character_Data,Graphic_character_Long);		
		Judge_Word_Data_Set(&DJI_Judge.DJI_Judge_Send_Msg.Judge_Graphic_Client_Single.Word_data_struct,Judge_Test_Graphic_Name,\
		Integer,Graphic_Change,Judge_Super_C_Show_Floor,3,15,14,2,Locate_X + 183,Locate_Y,0,Super_C_Judge_Msg_Show->Super_C_Limit);		
		Judge_Data_Send_To_Client(&DJI_Judge,0x0110,DJI_Judge.DJI_Judge_Send_Msg.Judge_Graphic_Client_Single.Judge_client_custom_graphic_single_Data,15);		
}

void Robot_Mode_Judge_Show(Judge_Show_Msg_t* Judge_Show_Mode)
{
			Judge_Character_Data_Set(&DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_client_custom_character,Judge_Show_Mode->Chassis_Mode_Judge_Msg_Show.Chassis_Mode_Name,\
			Character,Graphic_Add,2,3,Data_Sizw[2],14,Data_Width[2],1700,700,Judge_Show_Mode->Chassis_Mode_Judge_Msg_Show.Chassis_Mode_Word_Add[1]);
			Judge_Data_Send_To_Client(&DJI_Judge,0x0110,DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_Character_Data,Graphic_character_Long);	
}

extern int Vision_Get_Flag;
static float Judge_Angle_Get;
int Data_Lennnn;
int super_C_Num_Test;

void Judge_Task(void *pvParameters)
{
	vTaskDelay(500);
	
	Judge_Show_Msg.Chassis_Mode_Judge_Msg_Show.Chassis_Mode_Show = Return_Chassis_Mode_Add();
	Judge_Show_Msg.Gimbal_Mode_Judge_Msg_Show.Gimbal_Mode_Show = Return_Gimbal_Mode_Add();
	Judge_Show_Msg.Super_C_Judge_Msg_Show.Super_C_Msg_Show = Get_Cap_Data();
	
	Judge_Show_Msg.Chassis_Mode_Judge_Msg_Show.Chassis_Mode_Name = (uint8_t*)"CJS";
	Judge_Show_Msg.Gimbal_Mode_Judge_Msg_Show.Gimbal_Mode_Name = (uint8_t*)"GJS";
	Judge_Show_Msg.Super_C_Judge_Msg_Show.Super_C_Name = (uint8_t*)"SCJ";
	Judge_Show_Msg.Super_C_Judge_Msg_Show.Super_C_Word = Super_Word;
	
	for(int  i = 0;i < 7;i++)
	{
		Judge_Show_Msg.Chassis_Mode_Judge_Msg_Show.Chassis_Mode_Word_Add[i] = Chassis_Mode_Word[i];	
	}
	
	for(int  i = 0;i < 7;i++)
	{
		Judge_Show_Msg.Gimbal_Mode_Judge_Msg_Show.Gimbal_Mode_Word_Add[i] = Gimbal_Mode_Word[i];	
	}

	
	for(int i = 0;i < 10;i++)
	{
		Data_Width[i] = Judge_Set_Init[i][0];
		Data_Sizw[i] = Judge_Set_Init[i][1];	
		Location_x[i] = Judge_Set_Init[i][2];
		Location_y[i] = Judge_Set_Init[i][3];
	}
	
	Judge_Shoot_Line_Add();

	
	Judge_Gimbal_Mode_Show_Add(&Judge_Show_Msg.Gimbal_Mode_Judge_Msg_Show,1600,790);
	Judge_Chassis_Mode_Show_Add(&Judge_Show_Msg.Chassis_Mode_Judge_Msg_Show,1600,760);

	
	Judge_Super_C_Show_Add(&Judge_Show_Msg.Super_C_Judge_Msg_Show,1600,700);
	
	Judge_Num_Set = 0;

	
	while(1)
	{
				
		Judge_Gimbal_Mode_Show_Set(&Judge_Show_Msg.Gimbal_Mode_Judge_Msg_Show,1600,790);
		Judge_Chassis_Mode_Show_Set(&Judge_Show_Msg.Chassis_Mode_Judge_Msg_Show,1600,760);

		Judge_Super_C_Show_Set(&Judge_Show_Msg.Super_C_Judge_Msg_Show,1600,730);
		Judge_Shoot_Line_Set();
		
		
		if(Judge_Data_Send_Update > (1000/Judge_Set_Update_Time_Set))
		{
			Judge_Gimbal_Mode_Show_Add(&Judge_Show_Msg.Gimbal_Mode_Judge_Msg_Show,1600,790);
			Judge_Chassis_Mode_Show_Add(&Judge_Show_Msg.Chassis_Mode_Judge_Msg_Show,1600,760);

			Judge_Super_C_Show_Add(&Judge_Show_Msg.Super_C_Judge_Msg_Show,1600,730);
			Judge_Shoot_Line_Add();	
		}
			
			Judge_Data_Send_Update++;
			Judge_Angle_Get = -(DJI_Judge.DJI_Judge_Mes.Judge_game_robot_pos.yaw - 180.00f) / 180.000f * 3.1415926f;			
			vTaskDelay(Judge_Set_Update_Time_Set);
	}
}


float* Judge_Angle_Return(void)
{
	return &Judge_Angle_Get;
}
