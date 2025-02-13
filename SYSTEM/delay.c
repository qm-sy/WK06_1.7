#include "delay.h"

/**
 * @brief	ms 延时函数
 *
 * @param   void
 *
 * @return  
 */
void delay_ms(uint16_t time)	//@11.0592MHz
{
	unsigned char data i, j;
    for(time;time>0;time--)
    {
        _nop_();
        _nop_();
        _nop_();
        i = 11;
        j = 190;
        do
        {
            while (--j);
        } while (--i);
    }
}

// /**
//  * @brief	us 延时函数
//  *
//  * @param   void
//  *
//  * @return  
//  */
// void delay_us(uint16_t time)	//@11.0592MHz
// {
//    for(;time>0;time--)
//    {
//        _nop_();
//        _nop_();
//        _nop_();
//    }
// }