#ifndef _ADC_H_
#define _ADC_H_

#include "sys.h"
#include "delay.h"
  
void ADC_Init( void );
uint16_t Get_ADC_12bit( uint8_t ADC_Channel )	;

#endif