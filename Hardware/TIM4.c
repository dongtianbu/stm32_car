#include "stm32f10x.h"
#include "MPU6050.h"
#include "OLED.h"
#include "KEY.h"

#include "IMU.h"
#include "delay.h"

uint16_t i;

// TIM4初始化函数
void TIM4_Init(void)
{
    //使能TIM4时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    //配置定时器参数
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStruct;
    TIM_TimeBaseStruct.TIM_Prescaler = 7200 - 1;         
    TIM_TimeBaseStruct.TIM_Period = 1000 - 1;   //  7200 / 72M * 1000 = 100ms      
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up; 
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStruct);

    //使能更新中断
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    //配置NVIC中断优先级
	NVIC_PriorityGroupConfig NVIC_PriorityGroup_0;
	NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;  // 抢占优先级
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 4;         // 子优先级
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    //启动定时器
    TIM_Cmd(TIM4, ENABLE);
}
uint16_t IT_Flag = 0;

void TIM4_IRQHandler(void) //每80ms执行一次中断
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
		IT_Flag = 1;
		i++;
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    }
}

