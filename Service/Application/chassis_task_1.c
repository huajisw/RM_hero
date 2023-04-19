/**
  ****************************(C) COPYRIGHT 2021 HRBUST_AIR****************************
  * @file    chassis_task.c
	* @brief   底盘控制任务           
  * @author  JackyJuu , HRBUST_AIR_TEAM , website:www.airclub.tech
	* @Note 	 包含六种模式，在App_Set.h中进行设置
	* @Question 	 等超级电容加上后需要进行闭环测试【4.9】
  * @version V1.0.0
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     4-9-2021     	 JackyJuu            Done
  ****************************(C) COPYRIGHT 2021 HRBUST_AIR****************************
	* @describe 底盘控制任务
*/
#include "chassis_task.h"
#include "gimbal_task.h"
#include "chassis_power_control.h"

#include "math.h"
#include "arm_math.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "App_Set.h"

#include "can.h"
#include "pid.h"
#include "DJI_Remote_Control.h"

Chassis_t Chassis;

extern Gimbal_t Gimbal;

float Chassis_Speed_Get_LH,Chassis_Speed_Get_RH,Chassis_Speed_Get_RB,Chassis_Speed_Get_LB;

float Chassis_X_Speed_Set,Chassis_Y_Speed_Set,Chassis_LR_Speed_Set;

float Chassis_Speed_Get[4];

float Chassis_Speed_Set[4];

float Chassis_Speed_Send[4];

extern Motor_Msg_t ALL_Motor_Msg[16];

struct PID Chassis_PID[4];
extern RC_Ctl_t RC_Ctl_Data;

float 	Chassis_Motor_Speedd;

//uint8_t Super_C_Set[8] = {};

int Chassis_C_Set = 3000;

void Chassis_function_Four(void* Funtion_Four)
{

}

void Chassis_Data_Set_In_Four(uint16_t* Data_Address,uint16_t Num)
{
	for(int i = 0;i < 4;i++)
	Data_Address[i] = Num;
}

void Chassis_Init(Chassis_t* Chassis_Data_Init)
{
	//底盘模式初始化
	Chassis_Data_Init->Chassis_Mode = Chassis_Zero;
	
	//底盘电机PID初始化
	pid_init(&Chassis_Data_Init->Chassis_Motor_Pid[0],Chassis_Motor_Speed_Kp,Chassis_Motor_Speed_Ki,Chassis_Motor_Speed_Kd,Chassis_Motor_Speed_Maxout,Chassis_Motor_Speed_Kp,2);
	pid_init(&Chassis_Data_Init->Chassis_Motor_Pid[1],Chassis_Motor_Speed_Kp,Chassis_Motor_Speed_Ki,Chassis_Motor_Speed_Kd,Chassis_Motor_Speed_Maxout,Chassis_Motor_Speed_Kp,2);
	pid_init(&Chassis_Data_Init->Chassis_Motor_Pid[2],Chassis_Motor_Speed_Kp,Chassis_Motor_Speed_Ki,Chassis_Motor_Speed_Kd,Chassis_Motor_Speed_Maxout,Chassis_Motor_Speed_Kp,2);
	pid_init(&Chassis_Data_Init->Chassis_Motor_Pid[3],Chassis_Motor_Speed_Kp,Chassis_Motor_Speed_Ki,Chassis_Motor_Speed_Kd,Chassis_Motor_Speed_Maxout,Chassis_Motor_Speed_Kp,2);
	pid_init(&Chassis_Data_Init->Chassis_Motor_Turn_Pid,Chassis_Motor_Turn_Kp,Chassis_Motor_Turn_Ki,Chassis_Motor_Turn_Kd,Chassis_Motor_Turn_Maxout,Chassis_Motor_Turn_Kp,2);
	
	//获取底盘电机数据地址
	Chassis_Data_Init->Chassis_Motor_Msg[0].Chassis_Motor_Msg_Get = Get_DJI_Motor_Data(CAN1_RX,Chassis_Motor_RI);
	Chassis_Data_Init->Chassis_Motor_Msg[1].Chassis_Motor_Msg_Get = Get_DJI_Motor_Data(CAN1_RX,Chassis_Motor_LI);
	Chassis_Data_Init->Chassis_Motor_Msg[2].Chassis_Motor_Msg_Get = Get_DJI_Motor_Data(CAN1_RX,Chassis_Motor_LB);
	Chassis_Data_Init->Chassis_Motor_Msg[3].Chassis_Motor_Msg_Get = Get_DJI_Motor_Data(CAN1_RX,Chassis_Motor_RB);
		
	//裁判系统数据获取
	Chassis_Data_Init->Chassos_Judge_Msg.Chassis_Judge_Mes_Get = get_Judge_Mes_Add();
	Chassis_Data_Init->Chassos_Judge_Msg.Chassis_Power_Data_Get = (float)Chassis_Data_Init->Chassos_Judge_Msg.Chassis_Judge_Mes_Get->Judge_power_heat_data.chassis_power;
	Chassis_Data_Init->Chassos_Judge_Msg.Chassis_Heat_Data_Get = (int)Chassis_Data_Init->Chassos_Judge_Msg.Chassis_Judge_Mes_Get->Judge_power_heat_data.chassis_power_buffer;
	Chassis_Data_Init->Chassos_Judge_Msg.Chassis_Max_Power_Data_Get = (int)Chassis_Data_Init->Chassos_Judge_Msg.Chassis_Judge_Mes_Get->Judge_game_robot_status.chassis_power_limit;
		
	//底盘控制数据清零
	for(int i = 0;i < 4;i++)
	Chassis_Data_Init->Chassis_Motor_Speed_Set[i] = 0;
	
	//底盘跟随云台角度获取地址
	Chassis_Data_Init->Chassis_Follow_Gimbal_Angle_TM = Gimbal_Yaw_Angle_To_Chassis();
	
	//遥控器控制值获取地址
	Chassis_Data_Init->Chassis_RC_Ctl_Data = Get_DJI_RC_Data_Address();
	
	
	//底盘发送数据清零
	Chassis_Data_Set_In_Four((uint16_t*)Chassis_Data_Init->Chassis_Motor_Curent_Send,0);


}

/*****电机数据更新函数*****/
void Chassis_Motor_Data_Update(Chassos_Motor_Msg_t* Chassos_Motor_Msg_Update)
{
	Chassos_Motor_Msg_Update->Chassis_Speed_Get = ((float)(Chassos_Motor_Msg_Update->Chassis_Motor_Msg_Get->speed)*0.0012708333f);///60*2*3.1415926 *152.5/2/1000
}
/*****超级电容数据更新函数*****/
void Chassis_Cap_Data_Update(Chassis_Cap_Msg_t* Chassis_Cap_Msg_Update)
{
	Chassis_Cap_Msg_Update->Chassis_Cap_Power_Percent = ((float)(Chassis_Cap_Msg_Update->Chassis_Cap_Msg_Get->Voilt_C) / (float)(Chassis_Cap_Msg_Update->Chassis_Cap_Msg_Get->Voilt_In) * 100.00f);
	if(Chassis_Cap_Msg_Update->Chassis_Cap_Msg_Get->Target_power != Chassis_Cap_Msg_Update->Chassis_Cap_Power_Set)
			CAN1_Super_C_Send(0x210,Chassis_Cap_Msg_Update->Chassis_Cap_Power_Set);	
	Chassis_Cap_Msg_Update->Chassis_Cap_Power_Set = Chassis_Cap_Msg_Update->Chassis_Cap_Msg_Get->Target_power;
	//Chassis_Cap_Msg_Update
}
/*****裁判系统数据更新函数*****/
void Chassis_Judge_Data_Update(Chassos_Judge_Msg_t* Chassos_Judge_Msg_Update)
{
	Chassos_Judge_Msg_Update->Chassis_Power_Data_Get = (float)Chassos_Judge_Msg_Update->Chassis_Judge_Mes_Get->Judge_power_heat_data.chassis_power;
	Chassos_Judge_Msg_Update->Chassis_Heat_Data_Get = (int)Chassos_Judge_Msg_Update->Chassis_Judge_Mes_Get->Judge_power_heat_data.chassis_power_buffer;
	Chassos_Judge_Msg_Update->Chassis_Max_Power_Data_Get = (int)Chassos_Judge_Msg_Update->Chassis_Judge_Mes_Get->Judge_game_robot_status.chassis_power_limit;
	
	if(Chassos_Judge_Msg_Update->Chassis_Heat_Data_Get > 60)
	{
		Chassos_Judge_Msg_Update->Chassis_Max_Heat_Data_Get = Chassis_Max_Heat_Feipo;
	}
	else
	{
		Chassos_Judge_Msg_Update->Chassis_Max_Heat_Data_Get = Chassis_Max_Heat_Normal;		
	}	
}
/*****底盘PID更新函数*****/
float Chassis_pid_calc(PID*pid, float now, float set)
	{
    pid->now = now;
    pid->set = set;
		pid->now_error = pid->set - pid->now;	//set - measure
    if(pid->pid_mode == 1) //位置环PID
    {
	      pid->pout = pid->kp * pid->now_error;
        pid->iout = pid->ki * pid->sum_of_error;
        pid->dout = pid->kd * (pid->now_error - pid->Last_error);
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
        pid->dout = pid->kd * (pid->now_error - 2*pid->Last_error + pid->Last_Last_error);        
				PID_limit(&(pid->iout), pid->IntegralLimit);
        pid->plus = pid->pout + pid->iout + pid->dout;
        pid->plus_out = pid->last_plus_out + pid->plus;
			  pid->out = pid->plus_out; 
				PID_limit(&(pid->out), pid->MaxOutput);
        pid->last_plus_out = pid->plus_out;	//update last time
    }
		
    pid->Last_Last_error= pid->Last_error;
    pid->Last_error = pid->now_error;

    return pid->out;
}

float Chassis_Mouse_X_Set = 10.0f,Chassis_Mouse_Y_Set = 0.1f;

void Chassis_RC_Speed_Set(Chassis_t* Chassis_RC_Set)
{
		
}

void Chassis_KEY_Speed_Set(Chassis_t* Chassis_KEY_Set)
{
	
}

void Chassis_Mode_Zero(Chassis_t* Chassis_Zero_Set)
{
			Chassis_Zero_Set->Chassis_X_Speed_Set = 0;
			Chassis_Zero_Set->Chassis_Y_Speed_Set = 0;
			Chassis_Zero_Set->Chassis_LR_Speed_Set = 0;
}

void Chassis_Mode_Motionless(Chassis_t* Chassis_Motionless_Set)
{
			Chassis_Motionless_Set->Chassis_X_Speed_Set = 0;
			Chassis_Motionless_Set->Chassis_Y_Speed_Set = 0;
			Chassis_Motionless_Set->Chassis_LR_Speed_Set = 0;
}

//void Chassis_Mode_RC(Chassis_t* Chassis_RC_Set)
//{
//			Chassis_RC_Set->Chassis_X_Speed_Set = 0;
//			Chassis_RC_Set->Chassis_Y_Speed_Set = 0;
//			Chassis_RC_Set->Chassis_LR_Speed_Set = 0;
//}

fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;

void Chassis_Mode_RC(Chassis_t* Chassis_RC_Mode_Set)
{
	static int AD_X_Speed_Set,WS_Y_Speed_Set,QE_LR_Speed_Set;
	
			if(Chassis_RC_Mode_Set->Chassis_RC_Ctl_Data->key.v & CHASSIS_TURN_SPEED_UP_KEY)
			{
					AD_X_Speed_Set = 0;
					WS_Y_Speed_Set = 0;
					QE_LR_Speed_Set = 0;
					if(Chassis_RC_Mode_Set->Chassis_RC_Ctl_Data->key.v & CHASSIS_W_KEY)
					{
						WS_Y_Speed_Set += Chassis_Shift_Max_Speed;
					}
					if(Chassis_RC_Mode_Set->Chassis_RC_Ctl_Data->key.v & CHASSIS_S_KEY)
					{
						WS_Y_Speed_Set -= Chassis_Shift_Max_Speed;
					}
					if(Chassis_RC_Mode_Set->Chassis_RC_Ctl_Data->key.v & CHASSIS_A_KEY)
					{
						AD_X_Speed_Set -= Chassis_Shift_Max_Speed;
					}
					if(Chassis_RC_Mode_Set->Chassis_RC_Ctl_Data->key.v & CHASSIS_D_KEY)
					{
						AD_X_Speed_Set += Chassis_Shift_Max_Speed;
					}
					if(Chassis_RC_Mode_Set->Chassis_RC_Ctl_Data->key.v & CHASSIS_TURN_LEFT_KEY)
					{
						QE_LR_Speed_Set -= Chassis_Shift_Max_Speed;
					}
					if(Chassis_RC_Mode_Set->Chassis_RC_Ctl_Data->key.v & CHASSIS_TURN_RIGHT_KEY)
					{
						QE_LR_Speed_Set += Chassis_Shift_Max_Speed;
					}					
			}
			else
			{
					AD_X_Speed_Set = 0;
					WS_Y_Speed_Set = 0;
					QE_LR_Speed_Set = 0;
					if(Chassis_RC_Mode_Set->Chassis_RC_Ctl_Data->key.v & CHASSIS_W_KEY)
					{
						WS_Y_Speed_Set += Chassis_Max_Speed;
					}
					if(Chassis_RC_Mode_Set->Chassis_RC_Ctl_Data->key.v & CHASSIS_S_KEY)
					{
						WS_Y_Speed_Set -= Chassis_Max_Speed;
					}
					if(Chassis_RC_Mode_Set->Chassis_RC_Ctl_Data->key.v & CHASSIS_A_KEY)
					{
						AD_X_Speed_Set -= Chassis_Max_Speed;
					}
					if(Chassis_RC_Mode_Set->Chassis_RC_Ctl_Data->key.v & CHASSIS_D_KEY)
					{
						AD_X_Speed_Set += Chassis_Max_Speed;
					}
					if(Chassis_RC_Mode_Set->Chassis_RC_Ctl_Data->key.v & CHASSIS_TURN_LEFT_KEY)
					{
						QE_LR_Speed_Set -= Chassis_Max_Speed;
					}
					if(Chassis_RC_Mode_Set->Chassis_RC_Ctl_Data->key.v & CHASSIS_TURN_RIGHT_KEY)
					{
						QE_LR_Speed_Set += Chassis_Max_Speed;
					}					
			}
			Chassis_RC_Mode_Set->Chassis_X_Speed_Set = ((float)(Chassis_RC_Mode_Set->Chassis_RC_Ctl_Data->rc.ch0)/660.000f * Chassis_RC_Max_Speed) + AD_X_Speed_Set;
			Chassis_RC_Mode_Set->Chassis_Y_Speed_Set = ((float)(Chassis_RC_Mode_Set->Chassis_RC_Ctl_Data->rc.ch1)/660.000f * Chassis_RC_Max_Speed) + WS_Y_Speed_Set;
			Chassis_RC_Mode_Set->Chassis_LR_Speed_Set = ((float)(Chassis_RC_Mode_Set->Chassis_RC_Ctl_Data->rc.ch4)/660.000f * Chassis_RC_Max_Speed) + QE_LR_Speed_Set;
		
}

void Chassis_Mode_Follow_Gimbal(Chassis_t* Chassis_FW_Mode_Set)
{
//		float Chassis_VX,Chassis_VY,Chassis_VR;
	static int AD_X_Speed_Set,WS_Y_Speed_Set,QE_LR_Speed_Set;
	
			if(Chassis_FW_Mode_Set->Chassis_RC_Ctl_Data->key.v & CHASSIS_TURN_SPEED_UP_KEY)
			{
					AD_X_Speed_Set = 0;
					WS_Y_Speed_Set = 0;
					QE_LR_Speed_Set = 0;
					if(Chassis_FW_Mode_Set->Chassis_RC_Ctl_Data->key.v & CHASSIS_W_KEY)
					{
						WS_Y_Speed_Set += Chassis_Shift_Max_Speed;
					}
					if(Chassis_FW_Mode_Set->Chassis_RC_Ctl_Data->key.v & CHASSIS_S_KEY)
					{
						WS_Y_Speed_Set -= Chassis_Shift_Max_Speed;
					}
					if(Chassis_FW_Mode_Set->Chassis_RC_Ctl_Data->key.v & CHASSIS_A_KEY)
					{
						AD_X_Speed_Set -= Chassis_Shift_Max_Speed;
					}
					if(Chassis_FW_Mode_Set->Chassis_RC_Ctl_Data->key.v & CHASSIS_D_KEY)
					{
						AD_X_Speed_Set += Chassis_Shift_Max_Speed;
					}
					if(Chassis_FW_Mode_Set->Chassis_RC_Ctl_Data->key.v & CHASSIS_TURN_LEFT_KEY)
					{
						QE_LR_Speed_Set -= Chassis_Shift_Max_Speed;
					}
					if(Chassis_FW_Mode_Set->Chassis_RC_Ctl_Data->key.v & CHASSIS_TURN_RIGHT_KEY)
					{
						QE_LR_Speed_Set += Chassis_Shift_Max_Speed;
					}					
			}
			else
			{
					AD_X_Speed_Set = 0;
					WS_Y_Speed_Set = 0;
					QE_LR_Speed_Set = 0;
					if(Chassis_FW_Mode_Set->Chassis_RC_Ctl_Data->key.v & CHASSIS_W_KEY)
					{
						WS_Y_Speed_Set += Chassis_Max_Speed;
					}
					if(Chassis_FW_Mode_Set->Chassis_RC_Ctl_Data->key.v & CHASSIS_S_KEY)
					{
						WS_Y_Speed_Set -= Chassis_Max_Speed;
					}
					if(Chassis_FW_Mode_Set->Chassis_RC_Ctl_Data->key.v & CHASSIS_A_KEY)
					{
						AD_X_Speed_Set -= Chassis_Max_Speed;
					}
					if(Chassis_FW_Mode_Set->Chassis_RC_Ctl_Data->key.v & CHASSIS_D_KEY)
					{
						AD_X_Speed_Set += Chassis_Max_Speed;
					}
					if(Chassis_FW_Mode_Set->Chassis_RC_Ctl_Data->key.v & CHASSIS_TURN_LEFT_KEY)
					{
						QE_LR_Speed_Set -= Chassis_Max_Speed;
					}
					if(Chassis_FW_Mode_Set->Chassis_RC_Ctl_Data->key.v & CHASSIS_TURN_RIGHT_KEY)
					{
						QE_LR_Speed_Set += Chassis_Max_Speed;
					}					
			}
			
			AD_X_Speed_Set +=((float)(Chassis_FW_Mode_Set->Chassis_RC_Ctl_Data->rc.ch0)/660.000f * Chassis_RC_Max_Speed);
			WS_Y_Speed_Set +=((float)(Chassis_FW_Mode_Set->Chassis_RC_Ctl_Data->rc.ch1)/660.000f * Chassis_RC_Max_Speed);


			sin_yaw = arm_sin_f32(-*Chassis_FW_Mode_Set->Chassis_Follow_Gimbal_Angle_TM);
      cos_yaw = arm_cos_f32(-*Chassis_FW_Mode_Set->Chassis_Follow_Gimbal_Angle_TM);
			Chassis_FW_Mode_Set->Chassis_X_Speed_Set = cos_yaw * AD_X_Speed_Set + sin_yaw * WS_Y_Speed_Set;
			Chassis_FW_Mode_Set->Chassis_Y_Speed_Set = -sin_yaw * AD_X_Speed_Set + cos_yaw * WS_Y_Speed_Set;						
			

			Chassis_FW_Mode_Set->Chassis_LR_Speed_Set = pid_calc(&Chassis_FW_Mode_Set->Chassis_Motor_Turn_Pid,*Chassis_FW_Mode_Set->Chassis_Follow_Gimbal_Angle_TM,0);
}


void Chassis_Mode_Spin(Chassis_t* Chassis_Spin_Set)
{
	
	
	static int RC_SPIN_CTRL,RC_SPIN_CTRL_Mode;
	float Chassis_VX,Chassis_VY,Chassis_VR;
	
			Chassis_VX = ((float)(Chassis_Spin_Set->Chassis_RC_Ctl_Data->rc.ch0)/660.000f * Chassis_RC_Max_Speed);
			Chassis_VY = ((float)(Chassis_Spin_Set->Chassis_RC_Ctl_Data->rc.ch1)/660.000f * Chassis_RC_Max_Speed);
			Chassis_VR = ((float)(Chassis_Spin_Set->Chassis_RC_Ctl_Data->rc.ch4)/660.000f * Chassis_RC_Max_Speed);
	
			Chassis_Spin_Set->Chassis_X_Speed_Set = 0;
			Chassis_Spin_Set->Chassis_Y_Speed_Set = 0;
			Chassis_Spin_Set->Chassis_LR_Speed_Set = 0;
	

	
			if(Chassis_Spin_Set->Chassis_RC_Ctl_Data->key.v & CHASSIS_TURN_SPEED_UP_KEY)
			{
					Chassis_Spin_Set->Chassis_X_Speed_Set = 0;
					Chassis_Spin_Set->Chassis_Y_Speed_Set = 0;
					Chassis_Spin_Set->Chassis_LR_Speed_Set = 0;
					if(Chassis_Spin_Set->Chassis_RC_Ctl_Data->key.v & CHASSIS_W_KEY)
					{
						Chassis_VY += Chassis_Shift_Max_Speed;
					}
					if(Chassis_Spin_Set->Chassis_RC_Ctl_Data->key.v & CHASSIS_S_KEY)
					{
						Chassis_VY -= Chassis_Shift_Max_Speed;
					}
					if(Chassis_Spin_Set->Chassis_RC_Ctl_Data->key.v & CHASSIS_A_KEY)
					{
						Chassis_VX -= Chassis_Shift_Max_Speed;
					}
					if(Chassis_Spin_Set->Chassis_RC_Ctl_Data->key.v & CHASSIS_D_KEY)
					{
						Chassis_VX += Chassis_Shift_Max_Speed;
					}
					
					if(Chassis_Spin_Set->Chassis_Mode == Chassis_Spin_Right)
					{
						Chassis_VR = Chassis_Shift_Max_Speed;
					}
					else if(Chassis_Spin_Set->Chassis_Mode == Chassis_Spin_Left)
					{
						Chassis_VR = -Chassis_Shift_Max_Speed;
					}
					
		
			}
			else
			{
					Chassis_Spin_Set->Chassis_X_Speed_Set = 0;
					Chassis_Spin_Set->Chassis_Y_Speed_Set = 0;
					Chassis_Spin_Set->Chassis_LR_Speed_Set = 0;
					if(Chassis_Spin_Set->Chassis_RC_Ctl_Data->key.v & CHASSIS_W_KEY)
					{
						Chassis_VY += Chassis_Max_Speed;
					}
					if(Chassis_Spin_Set->Chassis_RC_Ctl_Data->key.v & CHASSIS_S_KEY)
					{
						Chassis_VY -= Chassis_Max_Speed;
					}
					if(Chassis_Spin_Set->Chassis_RC_Ctl_Data->key.v & CHASSIS_A_KEY)
					{
						Chassis_VX -= Chassis_Max_Speed;
					}
					if(Chassis_Spin_Set->Chassis_RC_Ctl_Data->key.v & CHASSIS_D_KEY)
					{
						Chassis_VX += Chassis_Max_Speed;
					}
					
					if(Chassis_Spin_Set->Chassis_Mode == Chassis_Spin_Right)
					{
						Chassis_VR = Chassis_Max_Speed;
					}
					else if(Chassis_Spin_Set->Chassis_Mode == Chassis_Spin_Left)
					{
						Chassis_VR = -Chassis_Max_Speed;
					}
							
			}
		
			
			sin_yaw = arm_sin_f32(*Chassis_Spin_Set->Chassis_Follow_Gimbal_Angle_TM);
			
      cos_yaw = arm_cos_f32(*Chassis_Spin_Set->Chassis_Follow_Gimbal_Angle_TM);
			
			Chassis_Spin_Set->Chassis_X_Speed_Set = cos_yaw * Chassis_VX + sin_yaw * Chassis_VY;
//			
			Chassis_Spin_Set->Chassis_Y_Speed_Set = -sin_yaw * Chassis_VX + cos_yaw * Chassis_VY;
			
//			Chassis_Spin_Set->Chassis_X_Speed_Set = sin_yaw * Chassis_VY;
//			
//			Chassis_Spin_Set->Chassis_Y_Speed_Set = cos_yaw * Chassis_VY;
		
			Chassis_Spin_Set->Chassis_LR_Speed_Set = Chassis_VR;
	
		RC_SPIN_CTRL = Chassis_Spin_Set->Chassis_RC_Ctl_Data->key.v;		

}



/*****底盘控制数据更新函数*****/
void Chassis_Control_Data_Get(Chassis_t* Chassis_Control_Data)
{
	switch(Chassis_Control_Data->Chassis_Mode)
	{
		case Chassis_Zero:
			Chassis_Mode_Zero(Chassis_Control_Data);
		break;
		
		case Chassis_RC_Control:	
				Chassis_Mode_RC(Chassis_Control_Data);
		break;
		
		case Chassis_Follow_Gimbal:
			Chassis_Mode_Follow_Gimbal(Chassis_Control_Data);
		break;
		
		case Chassis_Spin:	
			Chassis_Mode_Spin(Chassis_Control_Data);
		break;
		
		case Chassis_Spin_Left:	
			Chassis_Mode_Spin(Chassis_Control_Data);
		break;
		
		case Chassis_Spin_Right:	
			Chassis_Mode_Spin(Chassis_Control_Data);
		break;
						
	}

		Chassis_Control_Data->Chassis_Motor_Speed_Set[0] = Chassis_Control_Data->Chassis_X_Speed_Set - Chassis_Control_Data->Chassis_Y_Speed_Set - Chassis_Control_Data->Chassis_LR_Speed_Set;
		Chassis_Control_Data->Chassis_Motor_Speed_Set[1] = Chassis_Control_Data->Chassis_X_Speed_Set + Chassis_Control_Data->Chassis_Y_Speed_Set - Chassis_Control_Data->Chassis_LR_Speed_Set;
		Chassis_Control_Data->Chassis_Motor_Speed_Set[2] = -Chassis_Control_Data->Chassis_X_Speed_Set + Chassis_Control_Data->Chassis_Y_Speed_Set - Chassis_Control_Data->Chassis_LR_Speed_Set;
		Chassis_Control_Data->Chassis_Motor_Speed_Set[3] = -Chassis_Control_Data->Chassis_X_Speed_Set - Chassis_Control_Data->Chassis_Y_Speed_Set - Chassis_Control_Data->Chassis_LR_Speed_Set;	
		
	for(int i = 0;i < 4;i++)
	{
		if(Chassis_Control_Data->Chassis_Motor_Speed_Set[i] > Chassis_Max_Speed_Sett)
		{
			Chassis_Control_Data->Chassis_Motor_Speed_Set[i] = Chassis_Control_Data->Chassis_Motor_Speed_Set[i] * Chassis_Control_Data->Chassis_Motor_Speed_Set[i]/Chassis_Max_Speed_Sett;
		}
	}
}

/*****底盘模式更新函数*****/
extern int Check_Motionless_Flag;
void Chassis_Mode_Set(Chassis_t* Chassis_Mode)
{
	static int Chassis_Last_RC_Key_Mode,Chassis_Last_RC_SW;
	static int Mode_Count = 0;

	
	
			if(Chassis_Mode->Chassis_RC_Ctl_Data->rc.s2 == RC_SW_DOWN)   
				Chassis_Mode->Chassis_Mode = Chassis_First_Mode;
			else if(Chassis_Mode->Chassis_RC_Ctl_Data->rc.s2 == RC_SW_MID)  
				Chassis_Mode->Chassis_Mode = Chassis_RC_Control;
			else if(Chassis_Mode->Chassis_RC_Ctl_Data->rc.s2 == RC_SW_UP)
			{

				
				if(Chassis_Last_RC_SW == RC_SW_MID)
				{
					Chassis_Mode->Chassis_Mode = Chassis_Follow_Gimbal;
				}
				else
				{
					if((!(Chassis_Last_RC_Key_Mode & CHASSIS_TURN_LEFT_KEY)) && (Chassis_Mode->Chassis_RC_Ctl_Data->key.v & CHASSIS_TURN_LEFT_KEY))
					{
						if((Chassis_Mode->Chassis_Mode == Chassis_Spin_Left) || (Chassis_Mode->Chassis_Mode == Chassis_Spin_Right))
						{
							Chassis_Mode->Chassis_Mode = Chassis_Follow_Gimbal;
						}
						else
						{
							Chassis_Mode->Chassis_Mode = Chassis_Spin_Left;
						}	
					}
					else if((!(Chassis_Last_RC_Key_Mode & CHASSIS_TURN_RIGHT_KEY)) && (Chassis_Mode->Chassis_RC_Ctl_Data->key.v & CHASSIS_TURN_RIGHT_KEY))
					{
						if((Chassis_Mode->Chassis_Mode == Chassis_Spin_Left) || (Chassis_Mode->Chassis_Mode == Chassis_Spin_Right))
						{
							Chassis_Mode->Chassis_Mode = Chassis_Follow_Gimbal;
						}
						else
						{
							Chassis_Mode->Chassis_Mode = Chassis_Spin_Right; //小陀螺
						}	
					}
				}

			}
				
	Chassis_Last_RC_Key_Mode = Chassis_Mode->Chassis_RC_Ctl_Data->key.v;
			Chassis_Last_RC_SW = Chassis_Mode->Chassis_RC_Ctl_Data->rc.s2;

	
}

/*****底盘PID更新函数*****/
void Chassis_PID_Calculate_Data(Chassis_t* Chassis_PID)
{
	if(Chassis_PID->Chassis_Mode == Chassis_Zero)
	{
		Chassis_PID->Chassis_Motor_Curent_Send[0] = 0;
		Chassis_PID->Chassis_Motor_Curent_Send[1] = 0;
		Chassis_PID->Chassis_Motor_Curent_Send[2] = 0;
		Chassis_PID->Chassis_Motor_Curent_Send[3] = 0;
	}
	else
	{
		Chassis_PID->Chassis_Motor_Curent_Send[0] = Chassis_pid_calc(&Chassis_PID->Chassis_Motor_Pid[0],Chassis_PID->Chassis_Motor_Msg[0].Chassis_Speed_Get,Chassis_PID->Chassis_Motor_Speed_Set[0]);
		Chassis_PID->Chassis_Motor_Curent_Send[1] = Chassis_pid_calc(&Chassis_PID->Chassis_Motor_Pid[1],Chassis_PID->Chassis_Motor_Msg[1].Chassis_Speed_Get,Chassis_PID->Chassis_Motor_Speed_Set[1]);
		Chassis_PID->Chassis_Motor_Curent_Send[2] = Chassis_pid_calc(&Chassis_PID->Chassis_Motor_Pid[2],Chassis_PID->Chassis_Motor_Msg[2].Chassis_Speed_Get,Chassis_PID->Chassis_Motor_Speed_Set[2]);
		Chassis_PID->Chassis_Motor_Curent_Send[3] = Chassis_pid_calc(&Chassis_PID->Chassis_Motor_Pid[3],Chassis_PID->Chassis_Motor_Msg[3].Chassis_Speed_Get,Chassis_PID->Chassis_Motor_Speed_Set[3]);
	}
}

/*****底盘数据更新函数*****/
void Chassis_Data_Update(Chassis_t* Chassis_Update)
{
	for(int i = 0;i < 4;i++)
	Chassis_Motor_Data_Update(&Chassis_Update->Chassis_Motor_Msg[i]);
	
}

void Chassis_Task(void *pvParameters)
{
	vTaskDelay(100);
	
	Chassis_Init(&Chassis); //底盘初始化
  
	for(int ii = 0;ii < 4;ii++)
	
	pid_init(&Chassis_PID[ii],8.8,0,0,9000,5000,2); //增量式PID初始化 
	
	while(1)
	{
		Chassis_Mode_Set(&Chassis);    //模式设置 
		Chassis_Data_Update(&Chassis); //数据更新
		Chassis_Control_Data_Get(&Chassis);
		Chassis_PID_Calculate_Data(&Chassis);
		
		chassis_power_control(&Chassis);
		
		CAN1_Motor_Control(0x200,(int16_t)Chassis.Chassis_Motor_Curent_Send[0],(int16_t)Chassis.Chassis_Motor_Curent_Send[1],(int16_t)Chassis.Chassis_Motor_Curent_Send[2],(int16_t)Chassis.Chassis_Motor_Curent_Send[3]);

		vTaskDelay(1);			
	}
}

Chassis_Mode_t* Return_Chassis_Mode_Add(void)
{
	return &Chassis.Chassis_Mode;
}
