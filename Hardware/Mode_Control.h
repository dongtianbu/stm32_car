#ifndef __MODECONTROL_H
#define __MODECONTROL_H

//void TIM4_Init(void);

extern uint16_t IT_Flag;
extern uint8_t Mode_Car;//默认模式1（三角形模式），模式2（平行四边形模式）
extern float target_angle;
void Mode_Triangle(float* yaw, int16_t* State);
void Mode_Rectangle(float* yaw, int16_t* State);

#endif
