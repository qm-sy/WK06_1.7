#include "ntc.h"

#define NTCTabNum 126

TEMP temp;

static uint16_t MF52E_tab[NTCTabNum]={
    981,/*0*/    	1018,/*1*/    	1056,/*2*/    	1094,/*3*/    	1133,/*4*/    
    1173,/*5*/    	1214,/*6*/    	1255,/*7*/    	1296,/*8*/    	1338,/*9*/    
    1381,/*10*/    	1424,/*11*/    	1467,/*12*/    	1511,/*13*/    	1555,/*14*/    
    1599,/*15*/    	1644,/*16*/    	1689,/*17*/    	1734,/*18*/    	1779,/*19*/    
    1824,/*20*/    	1869,/*21*/    	1914,/*22*/    	1959,/*23*/    	2004,/*24*/    
    2048,/*25*/    	2093,/*26*/    	2137,/*27*/    	2181,/*28*/    	2225,/*29*/    
    2268,/*30*/    	2311,/*31*/    	2353,/*32*/    	2395,/*33*/    	2436,/*34*/    
    2477,/*35*/    	2518,/*36*/    	2557,/*37*/    	2596,/*38*/    	2635,/*39*/    
    2673,/*40*/    	2710,/*41*/    	2747,/*42*/    	2782,/*43*/    	2818,/*44*/    
    2852,/*45*/    	2886,/*46*/    	2919,/*47*/    	2951,/*48*/    	2983,/*49*/    
    3014,/*50*/    	3044,/*51*/    	3074,/*52*/    	3102,/*53*/    	3130,/*54*/    
    3158,/*55*/    	3184,/*56*/    	3210,/*57*/    	3236,/*58*/    	3260,/*59*/    
    3284,/*60*/    	3308,/*61*/    	3330,/*62*/    	3353,/*63*/    	3374,/*64*/    
    3395,/*65*/    	3415,/*66*/    	3435,/*67*/    	3454,/*68*/    	3472,/*69*/    
    3490,/*70*/    	3508,/*71*/    	3525,/*72*/    	3541,/*73*/    	3557,/*74*/    
    3573,/*75*/    	3588,/*76*/    	3602,/*77*/    	3616,/*78*/    	3630,/*79*/    
    3643,/*80*/    	3656,/*81*/    	3669,/*82*/    	3681,/*83*/    	3693,/*84*/    
    3704,/*85*/    	3715,/*86*/    	3725,/*87*/    	3735,/*88*/    	3745,/*89*/    
    3755,/*90*/    	3764,/*91*/    	3773,/*92*/    	3782,/*93*/    	3791,/*94*/    
    3799,/*95*/    	3807,/*96*/    	3815,/*97*/    	3823,/*98*/    	3830,/*99*/    
    3837,/*100*/    3844,/*101*/    3851,/*102*/    3857,/*103*/    3863,/*104*/    
    3869,/*105*/    3875,/*106*/    3881,/*107*/    3887,/*108*/    3893,/*109*/    
    3898,/*110*/    3903,/*111*/    3908,/*112*/    3913,/*113*/    3918,/*114*/    
    3923,/*115*/    3928,/*116*/    3932,/*117*/    3936,/*118*/    3941,/*119*/    
    3945,/*120*/    3949,/*121*/    3953,/*122*/    3956,/*123*/    3960,/*124*/    
    3964 /*125*/    												                                              
};	

/**
 * @brief 查表 找到adc_val对应的temp
 *
 * @param[in] temp_tab 对照表
 * @param[in] tab_num 表元素个数
 * @param[in] adc_val adc_val
 * 
 * @return  temp
 * 
**/
static uint16_t LookupTable( uint16_t *temp_tab, uint8_t tab_num, uint16_t adc_val )
{
		uint8_t	i;
        uint16_t temp_val = 0;
	
		for( i = 0; i < (tab_num-1); i++ )
		{
				if(( adc_val >= temp_tab[i] ) && ( adc_val < temp_tab[i+1]) )
				temp_val = i;	
		}
		return temp_val;
}

/**
 * @brief 取10次adc_val均值对应的temp
 *
 * @param[in] channle_x ntc通道1~4
 * 
 * @return  temp :最终温度
 * 
**/
uint16_t get_temp( uint8_t channle_x )
{
    uint8_t t;
    uint16_t adc_val = 0;
    uint16_t temp;

    /* 1, 取10次adc_val                           */
    for( t = 0; t < 10; t++ )
		{
			adc_val += Get_ADC_12bit(channle_x);
            
		}	       
    adc_val = adc_val/10;
    //printf("The average adc_val is %d \r\n",adc_val);
    /* 2, 查表，找到对应温度值                     */
    temp = LookupTable( MF52E_tab, NTCTabNum, adc_val );
    //printf("The temp is %d \r\n",temp);
    return temp;
}

/**
 * @brief 温度扫描，1s/次 控制功率开关
 *
 * @param[in] 
 * 
 * @return  
 * 
**/
void temp_scan( void )
{
    if( temp.temp_scan_flag == 1)
    {
        temp.temp_value1 =  get_temp(NTC_1);
        // temp.temp_value2 =  get_temp(NTC_2);
        // temp.temp_value3 =  get_temp(NTC_3);
        // if( temp.temp_scan_allow_flag == 1)
        // {
        if( temp.temp_value1 >= temp.temp_alarm_value1 ) 
        {
            AC_Out1 = AC_Out2 = AC_Out3 = AC_Out4 = 1;        //001
        }else
        {
            AC_Out1 = AC_Out2 = AC_Out3 = AC_Out4 = 0;
        }

            // if( temp.temp_value2 >= temp.temp_alarm_value2 ) 
            // {
            //     AC_Out2 = 1;        //001
            // }else
            // {
            //     AC_Out2 = 0;
            // }

            // if( temp.temp_value3 >= temp.temp_alarm_value3 ) 
            // {
            //     AC_Out3 = 1;        //001
            // }else
            // {
            //     AC_Out3 = 0;
            // }
    
        //}
        temp.temp_scan_flag = 0;
    }
}