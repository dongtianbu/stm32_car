#ifndef __MOTOR_H
#define __MOTOR_H

void Motor_Init(void);
void Motor_Speed_Left(int16_t Speed);
void Motor_Speed_Right(int16_t Speed);
void Motor_Control(int16_t* Num, int16_t* Speed);
void Motor_Forward(int16_t Speed);
void Motor_Stop(void);


#endif
