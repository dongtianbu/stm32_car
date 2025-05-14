#include "stm32f10x.h"                  // Device header
#include "PWM.h"
#include "math.h"
void Motor_Init(void)
{
	//初始化控制方向的两个端口
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);		

}

void Motor_Speed_Left(int16_t Speed)
{
	if (Speed >= 0)
	{
				
		GPIO_SetBits(GPIOB, GPIO_Pin_14);
		GPIO_ResetBits(GPIOB, GPIO_Pin_15);
		TIM_SetCompare2(TIM1, Speed);

	}
	if (Speed < 0)
	{
				
		GPIO_ResetBits(GPIOB, GPIO_Pin_14);
		GPIO_SetBits(GPIOB, GPIO_Pin_15);
		TIM_SetCompare2(TIM1, -Speed);

	}
}

void Motor_Speed_Right(int16_t Speed)
{
	if (Speed >= 0)
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_12);
		GPIO_SetBits(GPIOB, GPIO_Pin_13);
		TIM_SetCompare1(TIM1, Speed);

	}
	if (Speed < 0)
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_12);
		GPIO_ResetBits(GPIOB, GPIO_Pin_13);
		TIM_SetCompare1(TIM1, -Speed);

	}
}

void Motor_Forward(int16_t Speed)
{
	Motor_Speed_Right(Speed);
	Motor_Speed_Left(Speed);
}

//-1代表倒车，1代表左转，2代表右转
void Motor_Control(int16_t* Num, int16_t* Speed)
{
	if (*Num == -1)
	{
		Motor_Speed_Right(-*Speed);
		Motor_Speed_Left(-*Speed);
	}
	else if (*Num == 1)
	{
		Motor_Speed_Right(*Speed);
		Motor_Speed_Left(*Speed - 30); //设置左轮比右轮慢30

	}
	else if (*Num == 2)
	{
		Motor_Speed_Right(*Speed - 30);
		Motor_Speed_Left(*Speed);
	}
}

void Motor_Stop(void)
{
	Motor_Speed_Right(0);
	Motor_Speed_Left(0);

}
