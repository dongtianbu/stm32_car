#include "stm32f10x.h"                  // Device header
#include "delay.h"
#include "Motor.h"
#include "BUZZER.h"
#include "PID.h"
#include "USART1.h"
#include "OLED.h"
#include "inv_mpu.h"
#include "Mode_Control.h"

#define InitV 70

int16_t State = 0;
int Button_Flag_Mode = -1;//1是启动电机以及PID -1是关停
int16_t Button_NUM = 0;
int16_t Button_Flag = -1; //1代表启动，-1代表关停 用于在Button_Ack中反转状态
int16_t Button_Chek_On_Off = 1;

void Button_Init(void)
{
	BUZZER_Init();
	BUZZER_OFF();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
}

int16_t Button_Chek(void)
{
	
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) == 0) //切换模式
	{
		delay_ms(20);
		while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) == 0);
		delay_ms(20);
		Button_NUM = 1;
	}
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7) == 0) //启动
	{
		delay_ms(20);
		while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7) == 0);
		delay_ms(20);
		Button_NUM = 2;
	
	}
	return Button_NUM;
}

void Button_Ack_Start(void)
{
	Button_Flag = -Button_Flag; //如果原本是开，就会反转成关
	Button_NUM = 0; //清除键码
	if (Button_Flag == 1)
	{
		Motor_Forward(InitV);
	}
	if (Button_Flag == -1)
	{
		Motor_Speed_Left(0);
		Motor_Speed_Right(0);
	}
	BUZZER_ON();
	delay_ms(100);
	BUZZER_OFF();
}

void Wating_Param(void)
{
	while (Button_Chek() == 0)
	{	
//		IMU_GetEulerAngle();
		
		mpu_dmp_get_data(&pitch, &roll, &yaw);
		//Bluetooth_Control(Serial_RxPacket);
		if(State == 1)//检测到蓝牙按下开关 
		{
			break;
		}
		
		Kp = (float)Serial_RxPacket[0] / 100;
		Ki = (float)Serial_RxPacket[1] / 100;
		Kd = (float)Serial_RxPacket[2] / 100;
				
		Kp = Serial_RxPacket[0] / 100.0f;
		Ki = Serial_RxPacket[1] / 100.0f;
		Kd = Serial_RxPacket[2] / 100.0f;
	
		OLED_ShowString(2, 1, "Kp:");
		OLED_ShowChar(2, 6, '.');
		OLED_ShowNum(2, 4, Kp, 2);
		OLED_ShowNum(2, 7, (int)(Kp * 100) % 100, 2);
		
		OLED_ShowString(3, 1, "Ki:");
		OLED_ShowChar(3, 6, '.');
		OLED_ShowNum(3, 4, Ki, 2);
		OLED_ShowNum(3, 7, (int)(Ki * 100) % 100, 2);

		OLED_ShowString(4, 1, "Kd:");
		OLED_ShowChar(4, 6, '.');
		OLED_ShowNum(4, 4, Kd, 2);
		OLED_ShowNum(4, 7, (int)(Kd * 100) % 100, 2);
		
	}
	
}

void Wating_Tap_Start(void)
{
	static uint8_t handoff = 2;//切换模式标志位，现在的2代表按下后会切换到的模式
	while (State == 0)//判断条件是：全局状态是0 等待按下启动键开始
	{
//		Bluetooth_Control(Serial_RxPacket);
		
//		IMU_GetEulerAngle();
		
		mpu_dmp_get_data(&pitch, &roll, &yaw);
		OLED_ShowSignedNum(4, 7, yaw, 2);
		if (Button_Chek() == 1) 							//按下切换键
		{
			switch (handoff)
			{
				case 2:
					Mode_Car = 2;
					handoff = 3;
					break;
				case 3:
					Mode_Car = 3;
					handoff = 1;
					break;
				case 1:
					Mode_Car = 1;
					handoff = 2;
					break;
				defalut:
				break;
				
			}
			Button_NUM = 0;									//清除按键键码
		}
		OLED_ShowString(1, 1, "Mode:");
		OLED_ShowNum(1, 6, Mode_Car, 1);

		if (Button_Chek() == 2) //按下启动键
		{
			Button_Flag_Mode = -Button_Flag_Mode; //启动PID以及电机，启动模式标志位置1
			State = 1;
			break;
		}
	}
	
}


void Wating_Param_Turn(void)
{
	while (Button_Chek() == 0)
	{	
//		IMU_GetEulerAngle();
		
		mpu_dmp_get_data(&pitch, &roll, &yaw);
		//Bluetooth_Control(Serial_RxPacket);
		if(State == 1)//检测到蓝牙按下开关 
		{
			break;
		}
		
		Kp_Turn = (float)Serial_RxPacket[0] / 100;
		Ki_Turn = (float)Serial_RxPacket[1] / 100;
		Kd_Turn = (float)Serial_RxPacket[2] / 100;
				
		Kp_Turn = Serial_RxPacket[0] / 100.0f;
		Ki_Turn = Serial_RxPacket[1] / 100.0f;
		Kd_Turn = Serial_RxPacket[2] / 100.0f;
	
		OLED_ShowString(2, 1, "Kp:");
		OLED_ShowChar(2, 6, '.');
		OLED_ShowNum(2, 4, Kp_Turn, 2);
		OLED_ShowNum(2, 7, (int)(Kp_Turn * 100) % 100, 2);
		
		OLED_ShowString(3, 1, "Ki:");
		OLED_ShowChar(3, 6, '.');
		OLED_ShowNum(3, 4, Ki_Turn, 2);
		OLED_ShowNum(3, 7, (int)(Ki_Turn * 100) % 100, 2);

		OLED_ShowString(4, 1, "Kd:");
		OLED_ShowChar(4, 6, '.');
		OLED_ShowNum(4, 4, Kd_Turn, 2);
		OLED_ShowNum(4, 7, (int)(Kd_Turn * 100) % 100, 2);
		
	}
	
}


