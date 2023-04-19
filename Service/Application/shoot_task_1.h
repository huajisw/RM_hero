#ifndef _SHOOT_TASK_H_
#define _SHOOT_TASK_H_

#include "main.h"
#include "DJI_Remote_Control.h"
#include "pid.h"

#include "Judge_Data.h"

#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

#define Firc_Motor_Kp 2600
#define Firc_Motor_Max_Kp 12000

#define Firc_Motor_Left_Kp Firc_Motor_Kp
#define Firc_Motor_Left_Ki 0
#define Firc_Motor_Left_Kd 100

#define Firc_Motor_Right_Kp Firc_Motor_Kp
#define Firc_Motor_Right_Ki 0
#define Firc_Motor_Right_Kd 100

//下拨弹轮转速
#define Firc_Up_Mode_Speed_Settt 4 //2

#define Firc_Motor_Limit_Error 0.25f
//#define Firc_Motor_Limit_Error 80.00f

#define Firc_Motor_Up_Kp 4000
#define Firc_Motor_Up_Ki 0
#define Firc_Motor_Up_Kd 0

#define Fric_Up_Speed_Change 0.0013888888


#define Firc_Trigger_Motor_Kp 120//2800
#define Firc_Trigger_Motor_Ki 2.4
#define Firc_Trigger_Motor_Kd 0

//按键开启发射机构
#define Shoot_Control_Start_KEY KEY_PRESSED_OFFSET_R
//弹舱盖开关
#define Shoot_Bullut_Open_KEY KEY_PRESSED_OFFSET_Z

//红色镭射灯开关
#define Shoot_Laser_On() Laser_on()
#define Shoot_Laser_Off() Laser_off()


//拨弹轮转速转换
#define Trigger_Speed_Change Trigger_Speed_Hero

//英雄云台拨弹轮转换
#define Trigger_Speed_Hero 0.0043395526
//1/60/3591*187*5/2


//ZJX云台拨弹轮转换
#define Trigger_Speed_ZJX 0.00277777777
//1/60/36*10

//CHH云台拨弹轮转换
#define Trigger_Speed_CHH 0.0046296296
//1/60/36*10


#define Shoot_Motor_Speed_Sett 13.2

#define Trigger_Turn_Speed_Set Trigger_Speed_Set_Test

//拨弹速度设置
#define Trigger_Speed_Set_Test 6
#define Trigger_Speed_Set_Fast 2
#define Trigger_Speed_Set_Slow 2

#define Shoot_Once_Time_Limit 500

//测试拨弹速度

//判断堵转时间
#define Trigger_Stop_Time_Set 500

//反转时间
#define Trigger_Back_Time_Set 1000

//判断枪管无子弹时间
#define Shoot_Step_Time 50


//判断按键按下多久判断1发
#define Shoot_Key_Time_Set 1000

//微动开关宏定义
#define Shoot_key_READ  Shoot_key_READ_ZJX

#define Shoot_key_READ_ZJX  GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_1)
#define Shoot_key_READ_CHH  (GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_0) | GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_1))

#define Shoot_Key_Close 1
#define Shoot_Key_Open 0

//拨弹轮速度设置


typedef enum
{
	Shoot_Stop = 0,
	Shoot_Ready = 1,//发射准备
	Shoot_Start = 2,//发射启动 (没有用)
	Shoot_Done = 3,//单发结束
	Shoot_Back = 4,//卡单
	Shoot_Launch = 5,//发射完成
	Shoot_Test2 = 6,
	Shoot_Test3 = 7,
	Shoot_Test4 = 8,
	Shoot_Test5 = 9,
	Shoot_Test6 = 10
}Shoot_Mode_t;


typedef enum
{
	Shoot_None_Stop = 0,
	Shoot_Motor_Ready = 1,
	Shoot_Once = 2,
	Shoot_Long = 3
}Shoot_Num_Mode_t;

typedef enum
{
	Shoot_Bullut_Close = 0,
	Shoot_Bullut_Open = 1
}Shoot_Bullut_Mode_t;

typedef struct
{

	float FIRC_Speed;
	float FIRC_UP_Speed;
	float Trigger_Speed_Set;
	
}Shoot_Firc_Control_t;


typedef struct
{
	Motor_Msg_t* Shoot_Motor_Msg_Get;
	uint16_t Motoe_Now_Angle;
	int16_t Motoe_Now_Speed;
	int16_t Motor_Now_Dianliu;
	int16_t Motor_Last_Speed;
	
}Shoot_Motor_Msg_t;

typedef struct
{
	Motor_Msg_t* Shoot_Motor_Msg_Get;
	uint16_t Motoe_Now_Angle;
	float Motoe_Now_Speed;
	int16_t Motor_Now_Dianliu;
	int16_t Motor_Last_Speed;	
}Trigger_Motor_Msg_t;


typedef struct
{
	float Shoot_Left_Speed_Get;
	float Shoot_Right_Speed_Get;
	
}Shoot_State_Msg_t;

/*********裁判系统读取数据*********/
typedef struct
{
	DJI_Judge_Mes_t* Shoot_Judge_Mes_Get;
	//当前射速上线
	float Shoot_Speed_Limit;
	//当前热量冷却值
	int Shoot_Cool_Now;
	//当前热量上限
	int Shoot_Heat_Limit;
	//当前热量
	int Shoot_Heat_Now;
	//当前射速
	float Shoot_Judge_Speed_Now;
	//当前射频
	int Shoot_Trigget_Speed_Now;
	//当前热量百分比
	float Shoot_Heat_Percent_Now;
	//当前冷却值百分比
	float Shoot_Cool_Percent_Now;
}Shoot_Judge_Msg_t;



typedef struct
{
	float Shoot_Motor_Pid_Out[3];
	float Shoot_Trigger_Motor_Pid_Out;
	Shoot_Bullut_Mode_t Shoot_Bullut_Mode;//弹舱盖开关
	Shoot_Num_Mode_t Shoot_Num_Mode;//单发连发
	Shoot_Mode_t Shoot_Mode;//发射模式
	const RC_Ctl_t* Shoot_RC_Ctl_Data;	
	Shoot_Motor_Msg_t Shoot_Motor_Msg[3];
	Trigger_Motor_Msg_t Trigger_Motor_Msg;

	Shoot_Judge_Msg_t Shoot_Judge_Msg;
	
	PID Shoot_Motor_Left_Pid;
	PID Shoot_Motor_Right_Pid;
	PID Shoot_Motor_Up_Pid;
	PID Shoot_Trigger_Motor_Pid;
	
	Shoot_State_Msg_t Shoot_State_Msg;
	
	Shoot_Firc_Control_t Shoot_Firc_Control;
	
}Shoot_t;


void Shoot_Task(void *pvParameters);

#endif

