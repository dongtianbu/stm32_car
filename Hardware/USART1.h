#ifndef __USART1_H
#define __USART1_H

void USART1_Config();
extern uint16_t RX;
extern uint8_t Serial_TxPacket[50], Serial_RxPacket[50];
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_SendByte(uint8_t Byte);
void Send_Yaw_Data(float* yaw);


#endif
