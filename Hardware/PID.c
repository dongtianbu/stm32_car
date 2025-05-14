#include "stm32f10x.h"                  // Device header
#include "Motor.h"
#include "OLED.h"
#include "math.h"
#include "stdio.h"

#define InitV 70
#define InitV_Turn 20 //转弯初速度

float pitch, roll, yaw;


float Target, Actual, Out;
float Kp = 2.29,  Ki = 1.77, Kd = 2.0;
float Error0, Error1, ErrorInt;

float Target_Turn, Actual_Turn, Out_Turn;
float Kp_Turn = 0.07,  Ki_Turn = 0, Kd_Turn = 0.06;
float Error0_Turn, Error1_Turn, ErrorInt_Turn;

uint16_t Count = 0;
//四个参数分别是：角度 全局状态 是否更新目标角 直线/转弯 是否要重置输出值
void PID(float* yaw, int16_t* State, uint8_t PID_Angle_Update, uint8_t Direction, uint8_t Out_Flag)
{
	if (*State == 1)
	{
		if (Direction == 1) //Direction=1 走直线的PID
		{
			if (Out_Flag == 1)
			{
				Out = 0;
				Target = 0;
				Actual = 0;
				Error0 = 0;
				Error1 = 0;
				ErrorInt = 0;

			}

			if (PID_Angle_Update == 1)
			{
				Target = *yaw;
			}		
			
			Actual = *yaw;
			Error1 = Error0; 								
			Error0 = Target - Actual; 						
			ErrorInt += Error0 * 0.1;
				
			if (fabs(Error0) > 0.5f) 
			{
				Out = Kp * Error0 + Ki * ErrorInt + Kd * (Error0 - Error1);
				
				if (Out > 10)
				{
					Out = 10;
				}
				if (Out < -10)
				{
					Out = -10;
				}
				
				Motor_Speed_Right(InitV - Out);
				Motor_Speed_Left(InitV + Out);
			}

		}
		
/*------------------------------------------------------------------------------------------------------------*/	
		
		else if (Direction == 2)//Direction=2 转弯PID
		{
			if (Out_Flag == 1)
			{
				Out_Turn = 0;
				Target_Turn = 0;
				Actual_Turn = 0;
				Out_Turn = 0;;
				Error0_Turn = 0;
				Error1_Turn = 0;
				ErrorInt_Turn = 0;
			}
			if (PID_Angle_Update == 1)
			{
				Target_Turn = *yaw; //接收目标角度
			}
			
			Actual_Turn = *yaw;
			Error1_Turn = Error0_Turn; 								
			Error0_Turn = Target_Turn - Actual_Turn; 						
			
			
			// 积分抗饱和
			if(fabs(Error0_Turn) < 30.0f) // 小误差时启用积分
			{ 
				ErrorInt_Turn += Error0_Turn * 0.1;
				
				if (ErrorInt_Turn > 100.0f)
				{
					ErrorInt_Turn = 100.0f;
				}
				if (ErrorInt_Turn < -100.0f)
				{
					ErrorInt_Turn = -100.0f;
				}
				
			} 
			else 
			{
				ErrorInt_Turn = 0;
			}
			
			
			if (fabs(Error0_Turn) > 1.0f) //限定范围内使用PID
			{
				
				Out_Turn = Kp_Turn * Error0_Turn + Ki_Turn * ErrorInt_Turn + Kd_Turn * (Error0_Turn - Error1_Turn);
				
				if (Out_Turn > 13)
				{
					Out_Turn = 13;
				}
				if (Out_Turn < -13)
				{
					Out_Turn = -13;
				}
				
				
				
				if (Out_Turn > 0)//左转 
				{
					Motor_Speed_Right(InitV_Turn + Out_Turn);//右轮加速
					Motor_Speed_Left(-InitV_Turn - Out_Turn);//左轮减速
				}
				if (Out_Turn < 0)//右转
				{
					Motor_Speed_Right(-InitV_Turn + Out_Turn);//右轮减速
					Motor_Speed_Left(InitV_Turn - Out_Turn);//左轮加速
				}	

			}
		
			
		}
	
	
	}	
}

