#include "stm32f10x.h"                  // Device header
#include "PWM.h"


void Servo_Init(void)
{
	PWM_Init();
}

//0 500
//180 2500

void Servo_SetAngle(uint16_t Angle)
{
	TIM_SetCompare2(TIM2, 200 / 18 * Angle + 500);
}
