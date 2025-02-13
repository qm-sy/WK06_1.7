#include "eeprom.h"

#define EEPROM_Idle     0x00
#define EEPROM_Read     0x01
#define EEPROM_Write    0x02
#define EEPROM_Earse    0x03

#define EEPROM_Enable   0x80        //使能EEPROM操作
#define EEPROM_Disable  0x00        //禁止EEPROM操作

#define EEPROM_Crl_Time   12        //@11.0592MHz

/**
 * @brief	将单片机设置为ISP空闲状态
 *
 * @param   void
 *
 * @return  
**/
static void ISP_Idle()
{
    IAP_CONTR = EEPROM_Disable;     
    IAP_CMD   = EEPROM_Idle;        
    
    /* 每次EEPROM操作前，需要IAP_TRIG先写0x5A，再写 0xA5,不操作则赋0清空  */
    IAP_TRIG  = 0X00;              

    IAP_ADDRH = 0X80;               //将地址设置到非IAP区域
    IAP_ADDRL = 0X00;               //清空地址低位
}

/**
 * @brief	EEPROM 擦除目标地址所在的一页（1扇区/512字节）
 *
 * @param   addr：目标地址
 *
 * @return  
**/
void ISP_Earse( uint16_t addr )
{   
    IAP_CONTR = EEPROM_Enable;       

    IAP_TPS   = EEPROM_Crl_Time;

    IAP_CMD   = EEPROM_Earse;      

    IAP_ADDRL = addr;
    IAP_ADDRH = addr>>8;
    
    /*      发送命令         */
    IAP_TRIG  = 0X5A;       
    IAP_TRIG  = 0XA5;                                    
    
    delay_ms(10);
    
    ISP_Idle();
}

/**
 * @brief	EEPROM 写入函数
 *
 * @param   addr：目标地址
 * @param   dat ：写入的值
 *
 * @return  
**/
void ISP_Write( uint16_t addr, uint8_t dat )
{
    IAP_CONTR = EEPROM_Enable;

    IAP_TPS   = EEPROM_Crl_Time;

    IAP_CMD   = EEPROM_Write;       

    IAP_ADDRL = addr;
    IAP_ADDRH = addr>>8;

    IAP_DATA  = dat;

    /*      发送命令         */
    IAP_TRIG  = 0X5A;
    IAP_TRIG  = 0XA5;
    
    delay_ms(10);
    
    ISP_Idle();
}

/**
 * @brief	EEPROM 读取函数
 *
 * @param   addr：目标地址
 *
 * @return  dat：目标地址对应的值
**/
uint8_t ISP_Read( uint16_t addr )
{
    uint8_t dat;
    
    IAP_CONTR = EEPROM_Enable;

    IAP_TPS   = EEPROM_Crl_Time;

    IAP_CMD   = EEPROM_Read;       

    IAP_ADDRL = addr;
    IAP_ADDRH = addr>>8;
    
    /*      发送命令         */
    IAP_TRIG  = 0X5A;
    IAP_TRIG  = 0XA5;
    
    delay_ms(10);
    
    dat = IAP_DATA;
    
    ISP_Idle();
    
    return dat;
}


