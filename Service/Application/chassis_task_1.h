 #ifndef _CHASSIS_TASK_H_
#define _CHASSIS_TASK_H_

#include "main.h"
#include "DJI_Remote_Control.h"
#include "pid.h"
#include "Judge_Data.h"
#include "App_Set.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

#define Chassis_Motor_Speed_Kp 2400
#define Chassis_Motor_Speed_Ki 0
#define Chassis_Motor_Speed_Kd 0
#define Chassis_Motor_Speed_Maxout 10000
#define Chassis_Motor_Speed_IMaxout 0

#define Chassis_Motor_Turn_Kp 17
#define Chassis_Motor_Turn_Ki 0
#define Chassis_Motor_Turn_Kd 0
#define Chassis_Motor_Turn_Maxout 4
#define Chassis_Motor_Turn_IMaxout 0

//底盘最大速度设置
#define Chassis_Max_Speed_Sett 6

#define Chassis_3508_Data_Change_To_Speed 10

#define Chassis_Max_Heat_Normal 60
#define Chassis_Max_Heat_Feipo 250

#define CHASSIS_TURN_SPEED_UP_KEY KEY_PRESSED_OFFSET_SHIFT

#define CHASSIS_W_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_A_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_S_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_D_KEY KEY_PRESSED_OFFSET_D

#define CHASSIS_TURN_LEFT_KEY KEY_PRESSED_OFFSET_E
#define CHASSIS_TURN_RIGHT_KEY KEY_PRESSED_OFFSET_Q

#define Chassis_Max_Speed 4
#define Chassis_Shift_Max_Speed 10                                                                                                   
#define Chassis_RC_Max_Speed 10

#define Chassis_Follow_Gimbal_Set 6



/*********电机数据*********/
typedef struct
{
	//电机数据
	Motor_Msg_t* Chassis_Motor_Msg_Get;
	//经过转换后的电机转速
	float Chassis_Speed_Get;
}Chassos_Motor_Msg_t;

/*********裁判系统读取数据*********/
typedef struct
{
	DJI_Judge_Mes_t* Chassis_Judge_Mes_Get;
	//底盘功率数据
	float Chassis_Power_Data_Get;
	//底盘热量数据
	int Chassis_Heat_Data_Get;
	//当前底盘最大功率
	int Chassis_Max_Power_Data_Get;
	//当前底盘最大热量
	int Chassis_Max_Heat_Data_Get;
}Chassos_Judge_Msg_t;

/*********超级电容读取数据*********/
typedef struct
{
	//超级电容相关数据读取
	Super_C_Msg_t* Chassis_Cap_Msg_Get;
//	//输入电压读取
//	uint16_t Chassis_Voltage_In_Data_Get;
//	//电容电压读取
//	uint16_t Chassis_Cap_Data_Get;
//	//输入电流读取
//	uint16_t Chassis_Heat_Data_Get;
//	//目标功率读取
//	uint16_t Chassis_Cap_Power_Get;
	//超级电容电压容量百分比
	float Chassis_Cap_Power_Percent;
	//目标功率设置
	uint16_t Chassis_Cap_Power_Set;
}Chassis_Cap_Msg_t;

//union Chassis_Cap_Msg_t
//{
//	Chassos_Cap_Msg_Set_t Chassos_Cap_Msg_Set;
//	
//};


typedef struct
{
	//底盘模式
	Chassis_Mode_t Chassis_Mode;
	
	//电机相关信息
	Chassos_Motor_Msg_t Chassis_Motor_Msg[4];
	
	
	//裁判系统相关数据获取
	Chassos_Judge_Msg_t Chassos_Judge_Msg;
	
	//超级电容相关数据获取
	Chassis_Cap_Msg_t Chassos_Cap_Msg;
	
	//底盘电机相关pid设置
	PID Chassis_Motor_Pid[4];
	PID Chassis_Motor_Turn_Pid;
	
	//陀螺仪相关数据获取
	const float* Chassis_IMU_Angle;
	const float* Chassis_IMU_Aspeed;
	//遥控器控制值获取
	const RC_Ctl_t* Chassis_RC_Ctl_Data;	
	
	//底盘速度设置
	float Chassis_Control_Speed_Set[4];
	float Chassis_X_Speed_Set;
	float Chassis_Y_Speed_Set;
	float Chassis_LR_Speed_Set;
	//底盘跟随云台-云台机械角度差
	float* Chassis_Follow_Gimbal_Angle_TM;
	//底盘最大速度
	float Chassis_Speed_Max;
	//底盘电机速度设置
	float Chassis_Motor_Speed_Set[4];
	
	
	//底盘发送电流值
	float Chassis_Motor_Curent_Send[4];
	

}Chassis_t;

void Chassis_Task(void *pvParameters);
void oled_Show_Location(int x,int y);

Chassis_Mode_t* Return_Chassis_Mode_Add(void);

#endif

