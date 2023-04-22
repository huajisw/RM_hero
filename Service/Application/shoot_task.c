/**
  ****************************(C) COPYRIGHT 2021 HRBUST_AIR****************************
  * @file    shoot_task.c
	* @brief   射击控制任务           
  * @author  JackyJuu , HRBUST_AIR_TEAM , website:www.airclub.tech
	* @Note 	 已经完成读取裁判系统热量闭环
	* @Question 	 摩擦轮PID可能需要调整，需要获取摩擦力执行曲线【4.9】
  * @version V1.0.0
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     4-9-2021     	 JackyJuu            Done
  ****************************(C) COPYRIGHT 2021 HRBUST_AIR****************************
	* @describe 射击控制任务
*/

#include "shoot_task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "key.h"

#include "can.h"
#include "pid.h"

Shoot_t Shoot;

PID Shoot_Motor_pid[3];

int key_test;

extern float motor_set[2];

//extern Motor_Msg_t Shoot_Motor_Msg[3];

float Motor_Pid_Out[3];

int Trigger_Move_Flag = 0;
int Trigger_Move_Back_Flag = 2;


static int Last_Shoot_Key_Read;

void Shoot_Up_Motor_Set(float Speed)
{
		Shoot.Shoot_Firc_Control.FIRC_UP_Speed = -Speed;	
}

#define If_Motor_Stoped(speed) (((speed) >= -100) && ((speed) < 100)) 
int Stall_Timt_Count;


int Move_Stop_Count;
//拨弹轮正转控制
void Shoot_Trigger_Motor_Move(float Motor_Speed_Set)
{
	Shoot.Shoot_Firc_Control.Trigger_Speed_Set = Motor_Speed_Set;
}

//从发射出去到下一个弹丸进来
void Shoot_Launch_To_Done_Control(Shoot_Mode_t* Shoot_Mode_LTD,float Trigger_Speed_Sett)
{

	
	//如果上一次微动松开这一次闭合
	if((Last_Shoot_Key_Read == Shoot_Key_Open) && (Shoot_key_READ == Shoot_Key_Close))
	{
		*Shoot_Mode_LTD = Shoot_Done;

		Shoot.Shoot_Firc_Control.Trigger_Speed_Set = 0;
	}
	else if(*Shoot_Mode_LTD == Shoot_Launch)
	{
		
		Shoot_Trigger_Motor_Move(Trigger_Speed_Sett);
		
	}
		
	Last_Shoot_Key_Read = Shoot_key_READ;
}

//从发射开始到发射出去
void Shoot_Ready_To_Launch_Control(Shoot_Mode_t* Shoot_Mode_RTL,float Trigger_Speed_Sett)
{

	
	//如果上一次微动开关闭合这一次松开
	if((Last_Shoot_Key_Read == Shoot_Key_Close) && (Shoot_key_READ == Shoot_Key_Open))
	{
		
		*Shoot_Mode_RTL = Shoot_Launch;
		Shoot.Shoot_Firc_Control.Trigger_Speed_Set = 0;
		
	}
	
	if(*Shoot_Mode_RTL == Shoot_Ready)
	{
		
		Shoot_Trigger_Motor_Move(Trigger_Speed_Sett);
		
	}
		
	Last_Shoot_Key_Read = Shoot_key_READ;
}

void Shoot_Ready_To_Done_Control(Shoot_Mode_t* Shoot_Mode_RTL,float Trigger_Speed_Sett)
{
	//如果上一次微动开关闭合这一次松开
	if((Last_Shoot_Key_Read == Shoot_Key_Open) && (Shoot_key_READ == Shoot_Key_Close))
	{
		
		*Shoot_Mode_RTL = Shoot_Done;
		Shoot.Shoot_Firc_Control.Trigger_Speed_Set = 0;
		
	}
	
	if(*Shoot_Mode_RTL == Shoot_Ready)
	{
		
		Shoot_Trigger_Motor_Move(Trigger_Speed_Sett);
		
	}
		
	Last_Shoot_Key_Read = Shoot_key_READ;
}


int Shoot_Ready_Count = 0;
//刚开始到发射准备，弹丸有一发在中间
int Shoot_Key_Static = 0;
int Last_Shoot_Key_static = Shoot_Key_Open;
void Shoot_Start_To_Ready_Control(Shoot_Mode_t* Shoot_Mode_STR,float Trigger_Speed_Sett)
{
	Shoot_Key_Static = Shoot_key_READ;
	//如果微动开关闭合一次，说明子弹到达，进入准备发射模式1
   if(Shoot_Key_Static == Shoot_Key_Close && (*Shoot_Mode_STR != Shoot_Ready1))
	 {
			*Shoot_Mode_STR = Shoot_Ready1; 
	 }
	 if(Shoot_Key_Static == Shoot_Key_Open && Last_Shoot_Key_static == Shoot_Key_Close && (*Shoot_Mode_STR != Shoot_Ready))
	 {
			Shoot_Ready_Count = 0;
			Shoot.Shoot_Firc_Control.Trigger_Speed_Set = 0;	
			*Shoot_Mode_STR = Shoot_Ready; 
	 }
	 
	 if(*Shoot_Mode_STR != Shoot_Ready)
	 {
			Shoot_Trigger_Motor_Move(Trigger_Speed_Sett);
	 }
  
	
//	//如果微动开关闭合，说明子弹到达，进入准备发射模式
//	if(Shoot_key_READ == Shoot_Key_Close && Last_Shoot_Key_static == Shoot_Key_Close)
//	{

//		Shoot_Ready_Count = 0;
//		*Shoot_Mode_STR = Shoot_Ready;
//		Shoot.Shoot_Firc_Control.Trigger_Speed_Set = 0;
//	}
//	
//	//如果微动开关松开，说明子弹没有到达
//	if((Shoot_Ready_Count < Shoot_Step_Time) &&(Shoot_key_READ == Shoot_Key_Open) && (*Shoot_Mode_STR != Shoot_Ready))
//	{
//		Shoot_Ready_Count++;
//	}
//	
//	else if((Shoot_Ready_Count >= Shoot_Step_Time) && (Shoot_key_READ == Shoot_Key_Open)&& (*Shoot_Mode_STR != Shoot_Ready))
//	{
//		Shoot_Trigger_Motor_Move(Trigger_Speed_Sett);	
//	}
	
	Last_Shoot_Key_static = Shoot_Key_Static;

}

void Shoot_Single_Launch_Control(Shoot_Num_Mode_t* Shoot_Num_Mode_SL,Shoot_Mode_t* Shoot_Mode_SL,float Trigger_Speed_Sett)
{
	static int Last_Shoot_Key_Read;
	
//	if(*Shoot_Mode_SL == Shoot_Ready) 
//	{
//		Shoot_Ready_To_Launch_Control(Shoot_Mode_SL,Trigger_Speed_Sett); //发射准备到发射完成
//	}
//	else if(*Shoot_Mode_SL == Shoot_Launch) 
//	{
//		Shoot_Launch_To_Done_Control(Shoot_Mode_SL,Trigger_Speed_Sett); //发射完成到发射结束
//	}
	if(*Shoot_Mode_SL == Shoot_Ready) 
	{
		Shoot_Ready_To_Done_Control(Shoot_Mode_SL,Trigger_Speed_Sett); //发射准备到发射完成
	}
	
	else if(*Shoot_Mode_SL == Shoot_Done)
	{
		*Shoot_Mode_SL = Shoot_Ready;
		*Shoot_Num_Mode_SL = Shoot_Motor_Ready;
	}
}

void Shoot_Long_Launch_Control(Shoot_Num_Mode_t* Shoot_Num_Mode_LL,Shoot_Mode_t* Shoot_Mode_LL,float Trigger_Speed_Sett)
{
	static int Last_Shoot_Key_Read;
	
	if(*Shoot_Mode_LL == Shoot_Ready)
	{
		Shoot_Ready_To_Launch_Control(Shoot_Mode_LL,Trigger_Speed_Sett);
	}
	else if(*Shoot_Mode_LL == Shoot_Launch)
	{
		Shoot_Launch_To_Done_Control(Shoot_Mode_LL,Trigger_Speed_Sett);
	}
	else if(*Shoot_Mode_LL == Shoot_Done)
	{
		*Shoot_Mode_LL = Shoot_Ready;
	}
}

void Shoot_Motor_State_Get(Shoot_t* Shoot_State_G)
{
	
	float Motor_Speed;
		
	Shoot_State_G->Trigger_Motor_Msg.Motoe_Now_Speed = Shoot_State_G->Trigger_Motor_Msg.Shoot_Motor_Msg_Get->speed * Trigger_Speed_Change * 10;
	Shoot_State_G->Shoot_Motor_Msg[0].Motoe_Now_Speed = Shoot_State_G->Shoot_Motor_Msg[0].Shoot_Motor_Msg_Get->speed * Fric_Motor_Speed_Change;
	Shoot_State_G->Shoot_Motor_Msg[1].Motoe_Now_Speed = Shoot_State_G->Shoot_Motor_Msg[1].Shoot_Motor_Msg_Get->speed * Fric_Motor_Speed_Change;
	

}

int RC_Time_Count,RC_Key_Time_Count;
int Now_Rc_S1 = 0,Last_Rc_S1 = 0,Now_Rc_S2 = 0,Last_Rc_S2 = 0,Last_RC_Key_L = 0,Last_RC_Key_R = 0,Last_RC_KEY_Board = 0;

void Shoot_Start_Control_Set(Shoot_Num_Mode_t* Start_Mode,const RC_Ctl_t* Start_RC_Date)
{
	Now_Rc_S1 = Start_RC_Date->rc.s1;	
	
	 //遥控器
	  //遥控器左上角拨杆 从中间拨到上方  并且 Shoot_Num_Mode 为  Shoot_None_Stop 模式
		if(((Now_Rc_S1 == RC_SW_UP) && (Last_Rc_S1 != RC_SW_UP)) && (*Start_Mode == Shoot_None_Stop ))
		{
			*Start_Mode = Shoot_Motor_Ready;   
		}
		else if(((Now_Rc_S1 == RC_SW_UP) && (Last_Rc_S1 != RC_SW_UP)) && (*Start_Mode != Shoot_None_Stop))
		{
			*Start_Mode = Shoot_None_Stop;
		}
		
		//键盘
		else if((!(Last_RC_KEY_Board & Shoot_Control_Start_KEY) && (Start_RC_Date->key.v & Shoot_Control_Start_KEY)) && (*Start_Mode == Shoot_None_Stop))
		{
			*Start_Mode = Shoot_Motor_Ready;
		}
		else if((!(Last_RC_KEY_Board & Shoot_Control_Start_KEY) && (Start_RC_Date->key.v & Shoot_Control_Start_KEY)) && (*Start_Mode != Shoot_None_Stop))
		{
			*Start_Mode = Shoot_None_Stop;
		}	
				
		
}


void Shoot_Control_Data_Get(Shoot_t* Shoot_Control_G,float Fric_Speed_Set,float trigger_Speed_Set)
{
	static int RC_Time_Count_Start = 0;
	
	   //利用遥控器或者R键设置发射电机启动与否
		 Shoot_Start_Control_Set(&Shoot_Control_G->Shoot_Num_Mode,Shoot_Control_G->Shoot_RC_Ctl_Data);	

			
			if(Now_Rc_S1 == RC_SW_MID)
			{
				//S1拨杆控制单发
				if((RC_Time_Count > 0) &&(RC_Time_Count < Shoot_Once_Time_Limit) && (Shoot_Control_G->Shoot_Num_Mode == Shoot_Motor_Ready))
				{
					Shoot_Control_G->Shoot_Num_Mode = Shoot_Once;
				}
				else if((RC_Time_Count > 0)&&(RC_Time_Count >= Shoot_Once_Time_Limit) && (Shoot_Control_G->Shoot_Num_Mode == Shoot_Long))
				{
					Shoot_Control_G->Shoot_Firc_Control.Trigger_Speed_Set = 0;
					Shoot_Control_G->Shoot_Num_Mode = Shoot_Motor_Ready;
				}
						
				RC_Time_Count = 0;
								
				//鼠标左键控制单发&连发
				if(Shoot_Control_G->Shoot_RC_Ctl_Data->mouse.press_l == 1)
				{
					RC_Key_Time_Count ++;	
					if(RC_Key_Time_Count > 36000)
					{
						RC_Key_Time_Count = 36000;
					}					
				}	
				if((Shoot_Control_G->Shoot_RC_Ctl_Data->mouse.press_l == 0) && (Last_RC_Key_L == 1) && (RC_Key_Time_Count < Shoot_Key_Time_Set) && (Shoot_Control_G->Shoot_Num_Mode == Shoot_Motor_Ready))
				{
					Shoot_Control_G->Shoot_Num_Mode = Shoot_Once;			
				}
				else if((Shoot_Control_G->Shoot_RC_Ctl_Data->mouse.press_l == 1) && (Last_RC_Key_L == 1) && (RC_Key_Time_Count >= Shoot_Key_Time_Set) && (Shoot_Control_G->Shoot_Num_Mode == Shoot_Motor_Ready))
				{
					Shoot_Control_G->Shoot_Num_Mode = Shoot_Long;				
				}				
				if((Shoot_Control_G->Shoot_RC_Ctl_Data->mouse.press_l == 0) && (Last_RC_Key_L == 0))
				{
					RC_Key_Time_Count = 0;				
				}

			}
			else if(Now_Rc_S1 == RC_SW_DOWN)
			{
				//S1拨杆控制连发发
				RC_Time_Count++;
				if(RC_Time_Count > 36000)
				{
					RC_Time_Count = 36000;
				}
				if((RC_Time_Count >= Shoot_Once_Time_Limit) && (Shoot_Control_G->Shoot_Num_Mode == Shoot_Motor_Ready))
				{
					Shoot_Control_G->Shoot_Num_Mode = Shoot_Long;
				}	
			}		
		
	//   保留上一次的变量值
	Last_Rc_S1 = Now_Rc_S1;
//	Last_Rc_S2 = Now_Rc_S2;
		
	Last_RC_Key_L = Shoot_Control_G->Shoot_RC_Ctl_Data->mouse.press_l;
	Last_RC_Key_R = Shoot_Control_G->Shoot_RC_Ctl_Data->mouse.press_r;
			
	Last_RC_KEY_Board = Shoot_Control_G->Shoot_RC_Ctl_Data->key.v;	

}

void Shoot_Control_Data_Set(Shoot_t* Shoot_Control_D,float Fric_Speed_Set,float trigger_Speed_Set)
{
	  
		if((Shoot_Control_D->Shoot_Mode != Shoot_Ready) && (Shoot_Control_D->Shoot_Num_Mode == Shoot_Motor_Ready))
		{
			//Shoot_Start_To_Ready_Control(&Shoot_Control_D->Shoot_Mode,trigger_Speed_Set);
		}	

			
		//拨弹轮电机控制
		if(Shoot_Control_D->Shoot_Num_Mode == Shoot_Once) //单发
		{
			//Shoot_Single_Launch_Control(&Shoot_Control_D->Shoot_Num_Mode,&Shoot_Control_D->Shoot_Mode,trigger_Speed_Set);
			Shoot_Trigger_Motor_Move(0);
		}
		else if(Shoot_Control_D->Shoot_Num_Mode == Shoot_Long) //连发 
		{
			//Shoot_Long_Launch_Control(&Shoot_Control_D->Shoot_Num_Mode,&Shoot_Control_D->Shoot_Mode,trigger_Speed_Set);
			Shoot_Trigger_Motor_Move(6);
		}


		
		//摩擦轮电机设置
		if(Shoot_Control_D->Shoot_Num_Mode == Shoot_None_Stop)
		{
			
			Shoot_Control_D->Shoot_Firc_Control.Trigger_Speed_Set = 0;
			Shoot_Control_D->Shoot_Firc_Control.FIRC_Speed = 0;
			Shoot_Control_D->Shoot_Mode = Shoot_Stop;
			Shoot_Laser_Off();
			
		}
		else if(Shoot_Control_D->Shoot_Num_Mode != Shoot_None_Stop)
		{
			
			Shoot_Laser_On();
			Shoot_Control_D->Shoot_Firc_Control.FIRC_Speed = Fric_Speed_Set;
			
			if(Shoot_Control_D->Shoot_Mode == Shoot_Stop)
			{
				Shoot_Start_To_Ready_Control(&Shoot_Control_D->Shoot_Mode,trigger_Speed_Set);
			}
			
		}
}

float Shoot_pid_calc(PID*pid, float now, float set)
	{
    pid->now = now;
    pid->set = set;
		
		pid->Set_alpha = 0.96;

		pid->now_error = pid->set - pid->now;	//set - measure

		
		if((pid->now_error > Firc_Motor_Limit_Error) || (pid->now_error < -Firc_Motor_Limit_Error))
		{
			pid->kp = Firc_Motor_Max_Kp;
		}
		else
		{
			pid->kp = Firc_Motor_Kp;
		}
				
    if(pid->pid_mode == 1) //位置环PID
    {
	      pid->pout = pid->kp * pid->now_error;
        pid->iout = pid->ki * pid->sum_of_error;
        pid->dout = pid->kd * (pid->now_error - pid->Last_error );
				pid->sum_of_error+=pid->now_error;	
				PID_limit(&(pid->sum_of_error), 10000);
				PID_limit(&(pid->iout), pid->IntegralLimit);
        pid->out = pid->pout + pid->iout + pid->dout;
        PID_limit(&(pid->out), pid->MaxOutput);
    }	
		
    else if(pid->pid_mode == 2)//增量式PID
    {
        pid->pout = pid->kp * (pid->now_error - pid->Last_error);
        pid->iout = pid->ki * pid->now_error;
        //pid->dout = pid->kd * (pid->now_error - 2*pid->Last_error + pid->Last_Last_error);
			
				pid->dout = pid->kd * (pid->now_error - pid->Last_error) * (1 - pid->Set_alpha) + pid->Set_alpha * pid->Last_Ud;
				PID_limit(&(pid->iout), pid->IntegralLimit);
        pid->plus = pid->pout + pid->iout + pid->dout;
        pid->plus_out = pid->last_plus_out + pid->plus;
				pid->Last_Ud = pid->dout;
			  pid->out = pid->plus_out; 
				PID_limit(&(pid->out), pid->MaxOutput);
        pid->last_plus_out = pid->plus_out;	//update last time
    }
		
				else if(pid->pid_mode == 3)//变速积分/不完全微分
    {
	      pid->pout = pid->kp * pid->now_error;
				if(((pid->Set_Out_Mode == 1) && (pid->now_error > 0)) || ((pid->Set_Out_Mode == -1) && (pid->now_error < 0)));
				else
				{
					if(PID_Fabs(pid->now_error) <= pid->Set_B)
					{
						pid->sum_of_error += pid->now_error;
					}
					else if(PID_Fabs(pid->now_error) >= (pid->Set_B + pid->Set_A))
					{
						pid->sum_of_error = 0;
					}
					else
					{
						pid->Set_ratio = (pid->Set_A + pid->Set_B - PID_Fabs(pid->now_error)) / pid->Set_A;
						pid->sum_of_error += pid->Set_ratio * pid->now_error;
					}
				}
			//变速积分
      pid->iout = pid->ki * pid->sum_of_error;			
			//不完全微分	
			pid->dout = pid->kd * (pid->now_error - pid->Last_error) * (1 - pid->Set_alpha) + pid->Set_alpha * pid->Last_Ud;

        pid->out = pid->pout + pid->iout + pid->dout;
				if(pid->out > pid->MaxOutput)
				{
					pid->out = pid->MaxOutput;
					pid->Set_Out_Mode = 1;
				}
				else
				{
					pid->Set_Out_Mode = 0;
				}
				
				
				if(pid->out < -pid->MaxOutput)
				{
					pid->out = -pid->MaxOutput;
					pid->Set_Out_Mode = -1;
				}
				else
				{
					pid->Set_Out_Mode = 0;
				}
			pid->Last_Ud = pid->dout;

    }
		
    pid->Last_Last_error= pid->Last_error;
    pid->Last_error = pid->now_error;

    return pid->out;
}

int16_t Jscope_speed_shoot_moter_L,Jscope_speed_shoot_moter_R;
float Jscope_current_shoot_moter_L,Jscope_current_shoot_moter_R;

void Shoot_Pid_Calc(Shoot_t* Shoot_PID_C)
{
		Shoot_PID_C->Shoot_Motor_Pid_Out[0] = Shoot_pid_calc(&Shoot_PID_C->Shoot_Motor_Left_Pid,Shoot_PID_C->Shoot_Motor_Msg[0].Motoe_Now_Speed,Shoot_PID_C->Shoot_Firc_Control.FIRC_Speed);
		Shoot_PID_C->Shoot_Motor_Pid_Out[1] = Shoot_pid_calc(&Shoot_PID_C->Shoot_Motor_Right_Pid,Shoot_PID_C->Shoot_Motor_Msg[1].Motoe_Now_Speed,-Shoot_PID_C->Shoot_Firc_Control.FIRC_Speed);
	
	 Jscope_speed_shoot_moter_R = Shoot_PID_C->Shoot_Motor_Msg[0].Motoe_Now_Speed;
	 Jscope_speed_shoot_moter_L = -Shoot_PID_C->Shoot_Motor_Msg[1].Motoe_Now_Speed;
	
	 Jscope_current_shoot_moter_L = Shoot_PID_C->Shoot_Motor_Pid_Out[1];
	 Jscope_current_shoot_moter_R = Shoot_PID_C->Shoot_Motor_Pid_Out[0];
	
	
//		Shoot_PID_C->Shoot_Motor_Pid_Out[2] = pid_calc(&Shoot_PID_C->Shoot_Motor_Up_Pid,Shoot_PID_C->Shoot_Motor_Msg[2].Motoe_Now_Speed,Shoot_PID_C->Shoot_Firc_Control.FIRC_UP_Speed);
		Shoot_PID_C->Shoot_Trigger_Motor_Pid_Out = pid_calc(&Shoot_PID_C->Shoot_Trigger_Motor_Pid,Shoot_PID_C->Trigger_Motor_Msg.Motoe_Now_Speed,Shoot_PID_C->Shoot_Firc_Control.Trigger_Speed_Set);                
}

void Shoot_Init(Shoot_t*  Shoot_Data_Init)
{
	
	Shoot_Data_Init->Shoot_RC_Ctl_Data = Get_DJI_RC_Data_Address();
	
	Shoot_Data_Init->Shoot_Motor_Msg[0].Shoot_Motor_Msg_Get = Get_DJI_Motor_Data(CAN2_RX,Firc_Motor_Left); 
	Shoot_Data_Init->Shoot_Motor_Msg[1].Shoot_Motor_Msg_Get = Get_DJI_Motor_Data(CAN2_RX,Firc_Motor_Right);
	
	Shoot_Data_Init->Trigger_Motor_Msg.Shoot_Motor_Msg_Get = Get_DJI_Motor_Data(CAN1_RX,Trigger_Motor_Can);
	Shoot_Data_Init->Shoot_Mode = Shoot_Stop;
	
	//裁判系统数据获取
	Shoot_Data_Init->Shoot_Judge_Msg.Shoot_Judge_Mes_Get = Get_Judge_Info();
	Shoot_Data_Init->Shoot_Judge_Msg.Shoot_Speed_Limit = (float)Shoot_Data_Init->Shoot_Judge_Msg.Shoot_Judge_Mes_Get->Judge_game_robot_status.shooter_id1_42mm_speed_limit;
	Shoot_Data_Init->Shoot_Judge_Msg.Shoot_Cool_Now = (float)Shoot_Data_Init->Shoot_Judge_Msg.Shoot_Judge_Mes_Get->Judge_game_robot_status.shooter_id1_42mm_cooling_rate;
	Shoot_Data_Init->Shoot_Judge_Msg.Shoot_Heat_Limit = (int)Shoot_Data_Init->Shoot_Judge_Msg.Shoot_Judge_Mes_Get->Judge_game_robot_status.shooter_id1_42mm_cooling_limit;
	Shoot_Data_Init->Shoot_Judge_Msg.Shoot_Heat_Now = (int)Shoot_Data_Init->Shoot_Judge_Msg.Shoot_Judge_Mes_Get->Judge_power_heat_data.shooter_id1_42mm_cooling_heat;
	Shoot_Data_Init->Shoot_Judge_Msg.Shoot_Judge_Speed_Now = (int)Shoot_Data_Init->Shoot_Judge_Msg.Shoot_Judge_Mes_Get->Judge_shoot_data.bullet_speed;
	Shoot_Data_Init->Shoot_Judge_Msg.Shoot_Trigget_Speed_Now = (int)Shoot_Data_Init->Shoot_Judge_Msg.Shoot_Judge_Mes_Get->Judge_shoot_data.bullet_freq;
	
	pid_init(&Shoot_Data_Init->Shoot_Motor_Left_Pid,Firc_Motor_Left_Kp,Firc_Motor_Left_Ki,Firc_Motor_Left_Kd,15000,1000,1);	
	pid_init(&Shoot_Data_Init->Shoot_Motor_Right_Pid,Firc_Motor_Right_Kp,Firc_Motor_Right_Ki,Firc_Motor_Right_Kd,15000,1000,1);
	
//	pid_init(&Shoot_Data_Init->Shoot_Motor_Up_Pid,Firc_Motor_Up_Kp,Firc_Motor_Up_Ki,Firc_Motor_Up_Kd,8000,1000,1);
	
	pid_init(&Shoot_Data_Init->Shoot_Trigger_Motor_Pid,Firc_Trigger_Motor_Kp,Firc_Trigger_Motor_Ki,Firc_Trigger_Motor_Kd,8000,5000,1);	
	
}

int Shoot_Heat_Limit(float Robot_Heat_Percent,float Robot_Cool_Percent,int Robot_Cool_Limit)
{
	if((Robot_Heat_Percent >= 0) && (Robot_Heat_Percent < 50))
	{
		return 20;
	}
	else if((Robot_Heat_Percent >= 50) && (Robot_Heat_Percent < (100.0f-1.5f*Robot_Cool_Percent)))
	{
		return 8;
	}

	return 5;
}

float Shoot_Testtttttt = 5.10f;

void Shoot_Judge_Data_Check(Shoot_t* Shoot_Judge,float* Shot_Judge_Speed_Set,float* Shoot_Trigger_Speed_Set)
{
	//机器人 42mm 枪口上限速度 单位 m/s
	Shoot_Judge->Shoot_Judge_Msg.Shoot_Speed_Limit = (float)Shoot_Judge->Shoot_Judge_Msg.Shoot_Judge_Mes_Get->Judge_game_robot_status.shooter_id1_42mm_speed_limit;
	//机器人 42mm 枪口每秒冷却值
	Shoot_Judge->Shoot_Judge_Msg.Shoot_Cool_Now = (int)Shoot_Judge->Shoot_Judge_Msg.Shoot_Judge_Mes_Get->Judge_game_robot_status.shooter_id1_42mm_cooling_rate;
	//机器人 42mm 枪口热量上限
	Shoot_Judge->Shoot_Judge_Msg.Shoot_Heat_Limit = (int)Shoot_Judge->Shoot_Judge_Msg.Shoot_Judge_Mes_Get->Judge_game_robot_status.shooter_id1_42mm_cooling_limit;
	//机器人 42mm 枪口热量
	Shoot_Judge->Shoot_Judge_Msg.Shoot_Heat_Now = (int)Shoot_Judge->Shoot_Judge_Msg.Shoot_Judge_Mes_Get->Judge_power_heat_data.shooter_id1_42mm_cooling_heat;	
	//当前射速
	Shoot_Judge->Shoot_Judge_Msg.Shoot_Judge_Speed_Now = (float)Shoot_Judge->Shoot_Judge_Msg.Shoot_Judge_Mes_Get->Judge_shoot_data.bullet_speed;
	//当前射频
	Shoot_Judge->Shoot_Judge_Msg.Shoot_Trigget_Speed_Now = (int)Shoot_Judge->Shoot_Judge_Msg.Shoot_Judge_Mes_Get->Judge_shoot_data.bullet_freq;
	//当前热量百分比
	Shoot_Judge->Shoot_Judge_Msg.Shoot_Heat_Percent_Now = (((float)Shoot_Judge->Shoot_Judge_Msg.Shoot_Heat_Now / (float)Shoot_Judge->Shoot_Judge_Msg.Shoot_Heat_Limit) * 100.00f);
	//当前冷却值百分比
	Shoot_Judge->Shoot_Judge_Msg.Shoot_Cool_Percent_Now = (((float)Shoot_Judge->Shoot_Judge_Msg.Shoot_Cool_Now / (float)Shoot_Judge->Shoot_Judge_Msg.Shoot_Heat_Limit) * 100.00f);
	
	if(Shoot_Judge->Shoot_Judge_Msg.Shoot_Speed_Limit == 10)		
			*Shot_Judge_Speed_Set = Shoot_Judge->Shoot_Judge_Msg.Shoot_Speed_Limit + 1.2; //
	if(Shoot_Judge->Shoot_Judge_Msg.Shoot_Speed_Limit == 16)		
			*Shot_Judge_Speed_Set = Shoot_Judge->Shoot_Judge_Msg.Shoot_Speed_Limit + Shoot_Testtttttt;
	
	*Shoot_Trigger_Speed_Set = Shoot_Heat_Limit(Shoot_Judge->Shoot_Judge_Msg.Shoot_Heat_Percent_Now,Shoot_Judge->Shoot_Judge_Msg.Shoot_Cool_Percent_Now,Shoot_Judge->Shoot_Judge_Msg.Shoot_Cool_Now);
}


int Trigger_Speed_Send;

int Trigger_pwm_set = 2000;


float Trigger_Speed_Sett,Shoot_Fric_Speed_Sett;

int shoot_test;
void Shoot_Task(void *pvParameters)
{
	  vTaskDelay(100);
	
		Shoot_Init(&Shoot);
	
		Trigger_Speed_Sett = Trigger_Turn_Speed_Set;  //拨弹速度设置 
	
	  Shoot_Fric_Speed_Sett = Shoot_Motor_Speed_Sett; //发弹速度设置 

//  Shoot_Fric_Speed_Sett = 6300;//15.0-15.4 6200  6300
	
	  TIM_SetCompare1(TIM2,1500);
	
	while(1)
	{
	
		key_test = Shoot_key_READ;
		
		//发射电机，拨弹电机当前速度获取
		Shoot_Motor_State_Get(&Shoot);
		//裁判系统发射限制
		//Shoot_Judge_Data_Check(&Shoot,&Shoot_Fric_Speed_Sett,&Trigger_Speed_Sett); 
		Trigger_Speed_Sett = 6;
		
		Shoot_Control_Data_Get(&Shoot,Shoot_Fric_Speed_Sett,Trigger_Speed_Sett);
		
		Shoot_Control_Data_Set(&Shoot,Shoot_Fric_Speed_Sett,Trigger_Speed_Sett);
		
		Shoot_Pid_Calc(&Shoot);
		
		Trigger_Speed_Send = Shoot.Shoot_Trigger_Motor_Pid_Out;
		
		CAN2_Motor_Control(0x200,(int16_t)Shoot.Shoot_Motor_Pid_Out[0],(int16_t)Shoot.Shoot_Motor_Pid_Out[1],0,0);



		vTaskDelay(1);		
	}
}

