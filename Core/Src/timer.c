/*
	TIM0:485 超时接收
	TIM1:加热
	TIM2:
	TIM3:485
	TIM4:串口4
*/

#include "timer.h"

/**
 * @brief	外部中断0初始化函数
 *
 * @param   
 *
 * @return  void
**/
void INT0_Init( void )
{
    IT0 = 1;   //下降沿触发过零检测信号
    EX0 = 1;
}

/**
 * @brief	Timer0初始化函数
 *
 * @param   
 *
 * @return  void
**/
void Timer0_Init( void )	 //1ms@11.0592MHz   16位手动重装载
{
	AUXR |= 0x80;			 //定时器时钟1T模式
	TMOD &= 0xF0;			 //设置定时器模式
	TL0   = 0xCD;			 //设置定时初始值
	TH0   = 0xD4;          	 //设置定时初始值
	TF0   = 0;				 //清除TF0标志
	TR0   = 1;				 //定时器0开始计时

    ET0   = 1;				 //打开IE-ET0，TIM0中断
}


/**
 * @brief	Timer1初始化函数
 *
 * @param   
 *
 * @return  void
**/
void Timer1_Init( void )		//10ms@11.0592Mhz   16位手动重装载
{   
	AUXR &= 0xBF;       		//定时器时钟12T模式
	TMOD &= 0x0F;       		//设置定时器模式
	TMOD |= 0x10;       		//设置定时器模式   手动/自动开关
}

/**
 * @brief	Timer1初始化函数
 *
 * @param   
 *
 * @return  void
**/
void Timer2_Init( void )		//10毫秒@11.0592MHz
{
	AUXR &= 0xFB;			//定时器时钟12T模式
	T2L = 0x00;				//设置定时初始值
	T2H = 0xDC;				//设置定时初始值
	AUXR |= 0x10;			//定时器2开始计时

	IE2 |= 0X04;
}





