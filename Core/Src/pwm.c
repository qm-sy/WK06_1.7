#include "pwm.h"

#define PWMx_7      7
#define PWMx_8      8

/**
 * @brief	PWM初始化 Fpwm = SYSclk / (PWMx_PSCR + 1)*(PWMx_ARR + 1)
 *
 * @param   void
 *
 * @return  void
**/
void PWM_Init( void )
{
	P_SW2     |= 0X80;  
	
    PWMB_PS   |= 0x00;          //高级 PWM 通道 PWM7-P2.2  PWM8-P2.3

    PWMB_CCER2 = 0x00;          //清零CC7E、CC8E关闭通道

    PWMB_CCMR3 = 0x70;          //输入捕获n滤波器选择 PWM7
    PWMB_CCMR4 = 0x70;          //输入捕获n滤波器选择 PWM8

    PWMB_CCER2 = 0x11;          //配置通道PWM7 & PWM8 输出使能和极性 

    PWMB_ARR   = 0x0451;        //设置自动重装载寄存器值:1105
                 
    PWMB_PSCR  = 0x0031;        //设置频率   频率越大 波形越平滑  10000/(PWMB_PSCR+1)  200hz

    // PWMB_CCR7  = 0x012C;        //设置PWM7占空比
    // PWMB_CCR8  = 0x012C;        //设置PWM8占空比

    PWMB_ENO  |= 0x50;          //使能 PWM7 & PWM8 输出

    PWMB_BKR   = 0x80;          //使能主输出
    PWMB_CR1  |= 0x01;          //将CEN2位置1，开始计时        
}

