#include "adc.h"

#define ADC_POWER 0X80
#define ADC_START 0X40
#define ADC_FLAG  0X20

/**
 * @brief	ADC初始化
 *
 * @param   void
 *
 * @return  void
**/
void ADC_Init( void )
{	
	ADC_CONTR |= ADC_POWER;		//开AD转换电源
	delay_ms(10);               //适当延时等待AD转换供电稳定
	
	ADCCFG    &= 0xFC;			//AD转换速度为416个时钟数转换一次
	ADCCFG    |= 0x0C;			//AD转换速度为416个时钟数转换一次

	ADC_CONTR &= ~ADC_FLAG;		//清AD转换完成标志
		
	IE        &= ~0X20;			//禁止ADC转换中断

	ADCCFG    |= 0x20;			//ADC转换结果ADC_RES存高4位，ADC_RESL存低8位
}

/**
 * @brief	获取ADC值
 *
 * @param   ADC_Channel：ADC通道对应值
 *
 * @return  adc_value：12位ADC value
**/
uint16_t Get_ADC_12bit( uint8_t ADC_Channel )	
{
	uint16_t adc_value = 0;

	/* 1, 启动ADC转换                           */
	ADC_CONTR &= 0XF0;
	ADC_CONTR |=  ADC_START | ADC_Channel;

    /* 2, 等待ADC转换完成                       */
    while (!(ADC_CONTR & ADC_FLAG));  
	ADC_CONTR &= ~ADC_FLAG; 
	ADC_CONTR &= ~ADC_START;

	/* 3, 赋值给 adc_value 作为返回值           */
	adc_value = ADC_RES << 8 | ADC_RESL;			//ADC转换结果ADC_RES存高4位，ADC_RESL存低8位
	//printf("The adc_val is %d \r\n",adc_value);
	return adc_value;	
}
