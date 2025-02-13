/*
P0.0:RX3    P1.0		P2.0		P3.0:RX1    P4.0		P5.0		P6.0		P7.0
P0.1:TX3    P1.1		P2.1		P3.1:TX1    P4.1:220v_1 P5.1		P6.1		P7.1
P0.2		P1.2		P2.2:PWM7   P3.2:INT0   P4.2:220v_2 P5.2		P6.2		P7.2
P0.3		P1.3		P2.3:PWM8   P3.3		P4.3:220v_3 P5.3        P6.3		P7.3
P0.4：T1    P1.4		P2.4		P3.4		P4.4:220v_4 P5.4		P6.4		P7.4
P0.5：T2    P1.5		P2.5		P3.5		P4.5		P5.5		P6.5		P7.5
P0.6：T3    P1.6		P2.6		P3.6		P4.6		P5.6		P6.6		P7.6:DR3
P0.7：T4    P1.7		P2.7		P3.7		P4.7		P5.7		P6.7		P7.7:DR4
                                                                                        */

#ifndef __GPIO_H_
#define __GPIO_H_

#include "sys.h"

sbit DR4              = P7^7;

sbit AC_Out1 = P4^1;
sbit AC_Out2 = P4^2;
sbit AC_Out3 = P4^3;
sbit AC_Out4 = P4^4;

sbit OUT24_2          = P2^7;
sbit relay            = P2^6;

sbit buzzer           = P7^5;

sbit extern_24out2    = P3^5;
sbit extern_24out3    = P5^0;

sbit extern_pwm6      = P2^1;

sbit DC_24V_1         = P2^6;
sbit DC_24V_2         = P2^7;


sbit extern_24in      = P7^4;
sbit DR3              = P7^6;

void GPIO_Init( void );
    
#endif