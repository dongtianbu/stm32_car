#ifndef __KEY_H
#define __KEY_H

void Button_Init(void);
int16_t Button_Chek(void);
void Button_Ack_Start(void);
extern int16_t Button_NUM;
extern int Button_Flag_Mode;
extern int16_t State;

void Wating_Param(void);
void Wating_Tap_Start(void);
void Wating_Param_Turn(void);


#endif
