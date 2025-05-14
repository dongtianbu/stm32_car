#include "stm32f10x.h"                  // Device header
#include "delay.h"
#include "OLED.h"
#include "KEY.h"
#include "Motor.h"
#include "AD.h"
#include "MyI2C.h"
#include "USART1.h"
#include "PID.h"
#include "MPU6050.h"
#include "TIM4.h"
#include "PWM.h"
#include "KEY.h"
#include "sys.h"
#include "math.h"
#include "USART1.h"
#include "Bluetooth_Control.h"
#include "IMU.h"
#include "Mode_Control.h"

int Serial_RxCounter = 0;
uint8_t PID_Straight = 1;//直线模式首次调用标志位，用来更新目标角

int main(void)
{
	OLED_Init();
	Motor_Init();
	AD_Init();
	SystemInit();	
	USART1_Config();
	PWM_Init();
	Button_Init();
//	MPU6050_Init();
	
	
	OLED_ShowString(1, 1, "mpu_dmp_init");
	
	mpu_dmp_init();        //初始化DMP
	
	
	delay_ms(50);        
	OLED_Clear();
	OLED_ShowString(1, 1, "Waiting Param");//等待PID参数
	
//	Wating_Param();
//	Wating_Param_Turn();

	
	Button_NUM = 0;
	OLED_Clear();
	
	OLED_ShowString(2, 1, "Waiting to start");
		
	Wating_Tap_Start();//等待按下启动键，也可以按下切换键切换不同的功能
	
	delay_ms(100); 
	OLED_Clear();
	
	Button_NUM = 0; //清除键码
	
//	OLED_ShowString(2, 1, "Pitch:");
//	OLED_ShowString(3, 1, "Roll: ");

//	OLED_ShowChar(2, 10, '.');
//	OLED_ShowChar(3, 10, '.');
	OLED_ShowChar(4, 10, '.');
	TIM4_Init();
	
//	memset(Serial_RxPacket, 0, sizeof(&Serial_RxPacket));//清除数组数据防止死循环
	
		
	while (1)
	{
		mpu_dmp_get_data(&pitch, &roll, &yaw);
//		OLED_ShowSignedNum(2, 7, pitch, 2);
//		OLED_ShowNum(2, 11, ((uint16_t)(fabs(pitch) * 100)) % 100, 2);
//		OLED_ShowSignedNum(3, 7, roll, 2);
//		OLED_ShowNum(3, 11, ((uint16_t)(fabs(roll) * 100)) % 100, 2);
		OLED_ShowString(4, 1, "Yaw:");
		OLED_ShowChar(4, 9, '.');
		OLED_ShowSignedNum(4, 5, yaw, 3);
		OLED_ShowNum(4, 10, ((uint16_t)(fabs(yaw) * 100)) % 100, 3);
		
				
		
		if (State == 1) 						//判断全局状态
		{
//			if (IT_Flag == 1)
//			{
//				IT_Flag = 0;
//				PID(&yaw, &State, 1);
//			}
			if (Mode_Car == 1)
			{
				if (PID_Straight == 1)
				{
					PID(&yaw, &State, 1, 1, 1);
					PID_Straight = 0;
				}
				else
				{
					PID(&yaw, &State, 0, 1, 0);
				}
				OLED_ShowString(1, 1, "Straight");
				OLED_ShowString(2, 1, "Out:");
				OLED_ShowSignedNum(2, 5, Out, 5);
			}
			if (Mode_Car == 2)    				//判断是三角形模式
			{
				Mode_Triangle(&yaw, &State);
				OLED_ShowString(1, 1, "Triangle");
				OLED_ShowString(1, 9, " Tar:");
				OLED_ShowNum(1, 14, (uint16_t)target_angle, 3);
			}
			if (Mode_Car == 3)
			{
				Mode_Rectangle(&yaw, &State);
				OLED_ShowString(1, 1, "Rectangle");
				OLED_ShowString(1, 10, " Tar:");
				OLED_ShowNum(1, 14, (uint16_t)target_angle, 3);

			}

		}
		
		if (Button_Chek() == 2)//检测到按下启动按键
		{
			Button_Ack_Start();
			Button_Flag_Mode = -Button_Flag_Mode;
			if (Button_Flag_Mode == -1)
			{
				State = 0;
			}
			else
			{
				State = 1;
			}
			
		}
		
		Bluetooth_Control(Serial_RxPacket);
		
	}
}
