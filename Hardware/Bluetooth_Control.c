#include "stm32f10x.h"                  // Device header
#include "Motor.h"
#include "string.h"
#include "Key.h"


int16_t flag = -1;//停

void Bluetooth_Control(uint8_t Serial_RxPacket[])
{
	//蓝牙控制关停 若收到十六进制数据11 即十进制数据17 就停止
	int16_t temp1 = Serial_RxPacket[0];
	if (temp1 == 17 && flag == -1)//收到停止信号
	{
		flag = -flag;										//置flag为开
		Button_Ack_Start();
		State = 0; //全局状态为停止
		memset(Serial_RxPacket, 0, sizeof(&Serial_RxPacket));//清除数组数据防止死循环
		
	}
	else if (temp1 == 17 && flag == 1)//开始信号
	{
		flag = -flag;
		Button_Ack_Start();
		State = 1;
		memset(Serial_RxPacket, 0, sizeof(&Serial_RxPacket));//清除数组数据防止死循环
	}

}