#include "stm32f10x.h"                  // Device header
#include "string.h"
#include "math.h"

uint8_t Serial_TxPacket[50], Serial_RxPacket[50];
uint16_t Serial_RxFlag;
uint16_t RX;



/*****************辰哥单片机设计******************
											STM32
 * 项目			:	蓝牙模块通信实验                     
 * 版本			: V1.0
 * 日期			: 2024.10.2
 * MCU			:	STM32F103C8T6
 * 接口			:	串口1						
 * BILIBILI	:	辰哥单片机设计
 * CSDN			:	辰哥单片机设计
 * 作者			:	辰哥 

**********************BEGIN***********************/

//串口1中断服务程序
	//初始化 IO 串口 1
	//pclk1:PCLK1 时钟频率(Mhz)
	//bound:波特率  


void USART1_Config()
{ 
	  NVIC_InitTypeDef NVIC_InitStructure;
	  GPIO_InitTypeDef GPIO_InitStructure;
	  USART_InitTypeDef USART_InitStructure;
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);//GPIOB 时钟
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE); //串口 1 时钟使 USART_DeInit(USART1); //复位串口 1
	
	  GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);
		//USART1_TX PB6
	  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6; //PB6
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//复用推挽输出
	  GPIO_Init(GPIOB, &GPIO_InitStructure); //初始化 PB6
		
	  //USART1_RX PB7
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING; //浮空输入
	  GPIO_Init(GPIOB, &GPIO_InitStructure); //初始化 PB7
	
	  USART_InitStructure.USART_BaudRate= 9600; //波特率设
	  USART_InitStructure.USART_WordLength=USART_WordLength_8b; //8 位数据格
	  USART_InitStructure.USART_StopBits = USART_StopBits_1; //一个停止
	  USART_InitStructure.USART_Parity = USART_Parity_No; //无奇偶校验
	  USART_InitStructure.USART_HardwareFlowControl= USART_HardwareFlowControl_None; //无硬件数据流
	  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	  //收发模式
	  USART_Init(USART1, &USART_InitStructure); //初始化串口 1
	  USART_Cmd(USART1, ENABLE);//使能串口
	  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//使能接收中断 
	  //设置中断优先级
	  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级 2
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority =0; //子优先级 3
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ 通道使能
	  NVIC_Init(&NVIC_InitStructure); //根据指定的参数初始化 NVIC 寄存器
}


/**
  * 函    数：串口发送一个字节
  * 参    数：Byte 要发送的一个字节
  * 返 回 值：无
  */
void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART1, Byte);		//将字节数据写入数据寄存器，写入后USART自动生成时序波形
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);	//等待发送完成
	/*下次写入数据寄存器会自动清除发送完成标志位，故此循环后，无需清除标志位*/
}

/**
  * 函    数：串口发送一个数组
  * 参    数：Array 要发送数组的首地址
  * 参    数：Length 要发送数组的长度
  * 返 回 值：无
  */
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)		//遍历数组
	{
		Serial_SendByte(Array[i]);		//依次调用Serial_SendByte发送每个字节数据
	}
}

/**
  * 函    数：串口发送一个字符串
  * 参    数：String 要发送字符串的首地址
  * 返 回 值：无
  */
void Serial_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)//遍历字符数组（字符串），遇到字符串结束标志位后停止
	{
		Serial_SendByte(String[i]);		//依次调用Serial_SendByte发送每个字节数据
	}
}

/**
  * 函    数：次方函数（内部使用）
  * 返 回 值：返回值等于X的Y次方
  */
uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;	//设置结果初值为1
	while (Y --)			//执行Y次
	{
		Result *= X;		//将X累乘到结果
	}
	return Result;
}

/**
  * 函    数：串口发送数字
  * 参    数：Number 要发送的数字，范围：0~4294967295
  * 参    数：Length 要发送数字的长度，范围：0~10
  * 返 回 值：无
  */
void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)		//根据数字长度遍历数字的每一位
	{
		Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');	//依次调用Serial_SendByte发送每位数字
	}
}

/**
  * 函    数：使用printf需要重定向的底层函数
  * 参    数：保持原始格式即可，无需变动
  * 返 回 值：保持原始格式即可，无需变动
  */
//int fputc(int ch, FILE *f)
//{
//	Serial_SendByte(ch);			//将printf的底层重定向到自己的发送字节函数
//	return ch;
//}

///**
//  * 函    数：自己封装的prinf函数
//  * 参    数：format 格式化字符串
//  * 参    数：... 可变的参数列表
//  * 返 回 值：无
//  */
//void Serial_Printf(char *format, ...)
//{
//	char String[100];				//定义字符数组
//	va_list arg;					//定义可变参数列表数据类型的变量arg
//	va_start(arg, format);			//从format开始，接收参数列表到arg变量
//	vsprintf(String, format, arg);	//使用vsprintf打印格式化字符串和参数列表到字符数组中
//	va_end(arg);					//结束变量arg
//	Serial_SendString(String);		//串口发送字符数组（字符串）
//}


/**
  * 函    数：串口发送数据包
  * 参    数：无
  * 返 回 值：无
  * 说    明：调用此函数后，Serial_TxPacket数组的内容将加上包头（FF）包尾（FE）后，作为数据包发送出去
  */
void Serial_SendPacket(void)
{
	Serial_SendByte(0xFF);
	Serial_SendArray(Serial_TxPacket, 4);
	Serial_SendByte(0xFE);
}

/**
  * 函    数：获取串口接收数据包标志位
  * 参    数：无
  * 返 回 值：串口接收数据包标志位，范围：0~1，接收到数据包后，标志位置1，读取后标志位自动清零
  */
uint8_t Serial_GetRxFlag(void)
{
	if (Serial_RxFlag == 1)			//如果标志位为1
	{
		Serial_RxFlag = 0;
		return 1;					//则返回1，并自动清零标志位
	}
	return 0;						//如果标志位为0，则返回0
}

/**
  * 函    数：USART1中断函数
  * 参    数：无
  * 返 回 值：无
  * 注意事项：此函数为中断函数，无需调用，中断触发后自动执行
  *           函数名为预留的指定名称，可以从启动文件复制
  *           请确保函数名正确，不能有任何差异，否则中断函数将不能进入
  */
void USART1_IRQHandler(void)
{
	static uint8_t RxState = 0;		//定义表示当前状态机状态的静态变量
	static uint8_t pRxPacket = 0;	//定义表示当前接收数据位置的静态变量
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)		//判断是否是USART1的接收事件触发的中断
	{
		uint8_t RxData = USART_ReceiveData(USART1);				//读取数据寄存器，存放在接收的数据变量
		
		/*使用状态机的思路，依次处理数据包的不同部分*/
		
		/*当前状态为0，接收数据包包头*/
		if (RxState == 0)
		{
			if (RxData == 0xFF)			//如果数据确实是包头
			{
				RxState = 1;			//置下一个状态
				pRxPacket = 0;			//数据包的位置归零
			}
		}
		/*当前状态为1，接收数据包数据*/
		else if (RxState == 1)
		{
			Serial_RxPacket[pRxPacket] = RxData;	//将数据存入数据包数组的指定位置
			pRxPacket ++;				//数据包的位置自增
			if (pRxPacket >= 3)			//如果收够3个数据
			{
				RxState = 2;			//置下一个状态
			}
		}
		/*当前状态为2，接收数据包包尾*/
		else if (RxState == 2)
		{
			if (RxData == 0xFE)			//如果数据确实是包尾部
			{
				RxState = 0;			//状态归0
				Serial_RxFlag = 1;		//接收数据包标志位置1，成功接收一个数据包
			}
		}
		
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);		//清除标志位
	}
}

//void Send_Yaw_Data(float* yaw)
//{
//	if (*yaw < 0)
//	{
//		Serial_SendByte('-');
//		Serial_SendNumber((int16_t)(fabs(*yaw)), 2);
//		Serial_SendByte('.');
//		Serial_SendNumber(((uint16_t)(fabs(*yaw) * 100)) % 100, 2);

//	}
//	else if (*yaw > 0)
//	{
//		Serial_SendByte('+');
//		Serial_SendNumber((int16_t)(fabs(*yaw)), 2);
//		Serial_SendByte('.');
//		Serial_SendNumber(((uint16_t)(fabs(*yaw) * 100)) % 100, 2);

//	}
//}

