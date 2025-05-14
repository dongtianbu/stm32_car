#include "stm32f10x.h"                  // Device header

#define BUZZER_RCC_APB2Periph_GPIOB RCC_APB2Periph_GPIOB
#define BUZZER_GPIO GPIOB
#define BUZZER_GPIO_Pin GPIO_Pin_5


void BUZZER_Init(void)
{
	RCC_APB2PeriphClockCmd(BUZZER_RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = BUZZER_GPIO_Pin;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(BUZZER_GPIO, &GPIO_InitStruct);
}

void BUZZER_OFF(void)
{
	GPIO_SetBits(BUZZER_GPIO, BUZZER_GPIO_Pin);
}

void BUZZER_ON(void)
{
	GPIO_ResetBits(BUZZER_GPIO, BUZZER_GPIO_Pin);
}

