#ifndef __PID_H
#define __PID_H

void PID(float* yaw, int16_t* State, uint8_t PID_Angle_Update, uint8_t Direction, uint8_t Out_Flag);
extern int16_t AX, AY, AZ, GX, GY, GZ;
extern float Kp, Ki, Kd;
extern float Kp_Turn, Ki_Turn, Kd_Turn;

extern float pitch, roll, yaw;
//uint16_t PID_Mode = 0;
extern float Out;

#endif
