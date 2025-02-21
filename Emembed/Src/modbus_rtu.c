#include "modbus_rtu.h"

MODBIS_INFO modbus;
volatile uint8_t my_address = 0;
/**
 * @brief	modbus_rtu  无奇偶校验
 *
 * @param   void
 *
 * @return  void 
**/
void Modbus_Event( void )
{
    uint16_t crc,rccrc;

    /*1.接收完毕                                           */
    if( rs485.RX3_rcv_end_Flag == 1 )
    {
        /*2.清空接收完毕标志位                              */    
        rs485.RX3_rcv_end_Flag = 0;

        /*3.CRC校验                                         */
        crc = MODBUS_CRC16(rs485.RX3_buf, rs485.RX3_rcv_cnt-2);
        rccrc = (rs485.RX3_buf[rs485.RX3_rcv_cnt-2]<<8) | (rs485.RX3_buf[rs485.RX3_rcv_cnt-1]);

        /*4.清空接收计数                                    */
        rs485.RX3_rcv_cnt = 0; 

        /*5.CRC校验通过，进行地址域校验                      */
        if( crc == rccrc )
        {
            /*6.地址域校验通过，进入相应功能函数进行处理      */
            if( rs485.RX3_buf[0] == my_address )
            {
                switch ( rs485.RX3_buf[1] )
                {
                    case 0x03:
                        Modbus_Fun3();
                        break;

                    case 0x04:
                        Modbus_Fun4();
                        break;

                    case 0x06:
                        Modbus_Fun6();
                        break;  

                    case 0x10:  
                        Modbus_Fun16();

                    default:
                        break;
                }
            }
        }
    }
}

/**
 * @brief	读输出寄存器  03
 *
 * @param   void
 *
 * @return  void 
**/
void Modbus_Fun3( void )
{
    uint16_t i;
    uint8_t  channel;
    modbus.send_value_addr  = 3;                 //DATA1 H 位置
    modbus.byte_cnt   = (rs485.RX3_buf[4]<<8 | rs485.RX3_buf[5]) *2;
    modbus.start_addr = rs485.RX3_buf[2]<<8 | rs485.RX3_buf[3];

    rs485.TX3_buf[0]  = my_address;                //Addr
    rs485.TX3_buf[1]  = 0x03;                   //Fun
    rs485.TX3_buf[2]  = modbus.byte_cnt;        //Byte Count

    for( i = modbus.start_addr; i < modbus.start_addr + modbus.byte_cnt/2; i++ )
    {
        /*    每次循环前初始化byte_info                       */
        modbus.byte_info_H = modbus.byte_info_L = 0X00;
        switch (i)
        {
            /*  40001  两路PWM 开关状态及风速查询                 */
            case 0:
                modbus.byte_info_H  = 0X00;
                //modbus.byte_info_L |= ((PWMB_CCR7 / 184) | (PWMB_CCR8 / 184)<<4);   //PWM7\8风速
                modbus.byte_info_L |= PWMB_CCR7 / 184;
                break;

            /*  40002  220v开关状态查询                          */
            case 1:
                modbus.byte_info_H = (ac_220.time_delay - 58000) / 75;   
                channel = ac_220.channel1_flag | (ac_220.channel2_flag<<1) | (ac_220.channel3_flag<<2);
                switch (channel)
                {
                    case 0:
                        modbus.byte_info_L = 0x05;    
                        break;
                    
                    case 1:
                        modbus.byte_info_L = 0x00;    
                        break;

                    case 2:
                        modbus.byte_info_L = 0x01;    
                        break;

                    case 3:
                        modbus.byte_info_L = 0x03;    
                        break;

                    case 4:
                        modbus.byte_info_L = 0x02;    
                        break;

                    case 7:
                        modbus.byte_info_L = 0x04;    
                        break;

                    default:
                        break;
                }
                
                break;

            default:
                break;
        }
        rs485.TX3_buf[modbus.send_value_addr++] = modbus.byte_info_H;
        rs485.TX3_buf[modbus.send_value_addr++] = modbus.byte_info_L;
    }
    slave_to_master(3 + modbus.byte_cnt);
}


/**
 * @brief	读输入寄存器  04
 *
 * @param   void
 *
 * @return  void 
**/
void Modbus_Fun4( void )
{
    uint16_t i;

    modbus.send_value_addr  = 3;                //DATA1 H 位置
    modbus.byte_cnt   = (rs485.RX3_buf[4]<<8 | rs485.RX3_buf[5]) *2;
    modbus.start_addr = rs485.RX3_buf[2]<<8 | rs485.RX3_buf[3];

    rs485.TX3_buf[0]  = my_address;                //Addr
    rs485.TX3_buf[1]  = 0x04;                   //Fun
    rs485.TX3_buf[2]  = modbus.byte_cnt;        //Byte Count

    for( i = modbus.start_addr; i < modbus.start_addr + modbus.byte_cnt/2; i++ )
    {
        /*    每次循环前初始化byte_info                       */
        modbus.byte_info_H = modbus.byte_info_L = 0X00;
        switch (i)
        {   
            /*  30001 NTC1 NTC2温度查询                     */
            case 0:
                modbus.byte_info_L = get_temp(NTC_1);
                modbus.byte_info_H = 0x00;     
                break;

            default:
                break;
        }
        rs485.TX3_buf[modbus.send_value_addr++] = modbus.byte_info_H;
        rs485.TX3_buf[modbus.send_value_addr++] = modbus.byte_info_L;
    }
    slave_to_master(3 + modbus.byte_cnt);
}

/**
 * @brief	写单个输出寄存器  06
 *
 * @param   void
 *
 * @return  void 
**/
void Modbus_Fun6( void )
{
    uint8_t led1_statu;
    uint8_t led2_statu;

    switch (rs485.RX3_buf[3])
    {
        /*  40001  两路PWM 开关状态及风速设置                 */
        case 0:             
            memcpy(rs485.TX3_buf,rs485.RX3_buf,8);                            
            
            PWMB_CCR7 = ((rs485.TX3_buf[5]) & 0x0F)*184;
            PWMB_CCR8 = (rs485.TX3_buf[5]>>4)*184;

            rs485.TX3_send_bytelength = 8;

            DR3 = 1;                                    //485可以发送
            delay_ms(5);
            S3CON |= S3TI;                              //开始发送

            eeprom.pwm_info = rs485.TX3_buf[5];
            eeprom_data_record();

          break;

        /*  40002  24V LED开关状态设置                          */
        case 1:                                         
            memcpy(rs485.TX3_buf,rs485.RX3_buf,8);

            led1_statu = 1 - modbus.byte_info_L & 0X01;
            led2_statu = 1 - ((modbus.byte_info_L >> 4) & 0X01);

            DC_24V_out(led1_statu,led2_statu);
            
            rs485.TX3_send_bytelength = 8;

            DR3 = 1;                                    //485可以发送
            delay_ms(5);
            S3CON |= S3TI;                              //开始发送

            eeprom.led_info = rs485.TX3_buf[5];
            eeprom_data_record();

            break;

        /*  40003  220V 开关及大小设置                          */
        case 2:                                         
            memcpy(rs485.TX3_buf,rs485.RX3_buf,8);

            if( rs485.TX3_buf[5] & 0X01 )
            {
                EX0 = 1;
            }else
            {
                EX0 = 0;
            }
            AC_220V_out(rs485.TX3_buf[5]>>1);

            rs485.TX3_send_bytelength = 8;
            DR3 = 1;                                    //485可以发送
            delay_ms(5);
            S3CON |= S3TI;                              //开始发送

            eeprom.ac220_info = rs485.TX3_buf[5];
            eeprom_data_record();

            break;  
            
        /*  40004  NTC1 NTC2 alarm value 设置                   */
        case 3:                                         
            memcpy(rs485.TX3_buf,rs485.RX3_buf,8);

            temp.temp_alarm_value1 = rs485.TX3_buf[5];
            temp.temp_alarm_value2 = rs485.TX3_buf[4];
            
            rs485.TX3_send_bytelength = 8;
            DR3 = 1;                                    //485可以发送
            delay_ms(5);
            S3CON |= S3TI;                              //开始发送

            eeprom.temp_alarm_value1 = temp.temp_alarm_value1;
            eeprom.temp_alarm_value2 = temp.temp_alarm_value2;
            eeprom_data_record();

            break;

        /*  40005  NTC3 alarm value 设置                        */
        case 4:                                         
            memcpy(rs485.TX3_buf,rs485.RX3_buf,8);

            temp.temp_alarm_value3 = rs485.TX3_buf[5];
            
            rs485.TX3_send_bytelength = 8;
            DR3 = 1;                                    //485可以发送
            delay_ms(5);
            S3CON |= S3TI;                              //开始发送

            eeprom.temp_alarm_value3 = temp.temp_alarm_value3;
            eeprom_data_record();

            break;
        default:
            break;   
    }
}

/**
 * @brief	写多个输出寄存器  16
 *
 * @param   void
 *
 * @return  void 
**/
void Modbus_Fun16( void )
{
    uint16_t crc;
    uint16_t i;
//    uint8_t led1_statu;
//    uint8_t led2_statu;

    modbus.rcv_value_addr = 7;                  //DATA1 H位置
    modbus.byte_cnt   = rs485.RX3_buf[6];
    modbus.start_addr = rs485.RX3_buf[2]<<8 | rs485.RX3_buf[3];

    memcpy(rs485.TX3_buf,rs485.RX3_buf,6);

    for( i = modbus.start_addr; i < modbus.start_addr + modbus.byte_cnt/2; i++)
    {
        modbus.byte_info_H = rs485.RX3_buf[modbus.rcv_value_addr];
        modbus.byte_info_L = rs485.RX3_buf[modbus.rcv_value_addr + 1];
        switch (i)
        {
            /*  40001  两路PWM 开关状态及风速设置                 */
            case 0:
                PWMB_CCR7 = modbus.byte_info_L *184;
                PWMB_CCR8 = modbus.byte_info_L *184;

                eeprom.pwm_info = modbus.byte_info_L;
                break;
            
            /*  40002  24V LED开关状态设置                          */
            case 1:
                AC_220V_out(modbus.byte_info_H);

                switch (modbus.byte_info_L)
                {
                    case 0:
                        ac_220.channel1_flag = 1;
                        ac_220.channel2_flag = 0;
                        ac_220.channel3_flag = 0;
                        break;
                    
                    case 1:
                        ac_220.channel1_flag = 0;
                        ac_220.channel2_flag = 1;
                        ac_220.channel3_flag = 0;
                        break;

                    case 2:
                        ac_220.channel1_flag = 0;
                        ac_220.channel2_flag = 0;
                        ac_220.channel3_flag = 1;
                        break;

                    case 3:
                        ac_220.channel1_flag = 1;
                        ac_220.channel2_flag = 1;
                        ac_220.channel3_flag = 0;
                        break;

                    case 4:
                        ac_220.channel1_flag = 1;
                        ac_220.channel2_flag = 1;
                        ac_220.channel3_flag = 1;
                        break;
                    
                    case 5:
                        ac_220.channel1_flag = 0;
                        ac_220.channel2_flag = 0;
                        ac_220.channel3_flag = 0;
                        break;
                    default:
                        break;
                }

                break;

            default:
                break;
        }
        modbus.rcv_value_addr += 2;         //从Value1_H →→ 从Value2_H
    }
    
    crc = MODBUS_CRC16(rs485.TX3_buf,6);
    rs485.TX3_buf[6] = crc>>8;                 //CRC H
    rs485.TX3_buf[7] = crc;                    //CRC L

    rs485.TX3_send_bytelength = 8;

    DR3 = 1;                                   //485可以发送
    delay_ms(5);
    S3CON |= S3TI;  

    //eeprom_data_record();                      //记录更改后的值
}

/**
 * @brief	crc校验函数
 * 
 * @param   buf：  Address(1 byte) +Funtion(1 byte) ）+Data(n byte)   
 * @param   length:数据长度           
 * 
  @return  crc16:crc校验的值 2byte
 */
uint16_t MODBUS_CRC16(uint8_t *buf, uint8_t length)
{
	uint8_t	i;
	uint16_t	crc16;

    /* 1, 预置16位CRC寄存器为0xffff（即全为1）                          */
	crc16 = 0xffff;	

	do
	{
        /* 2, 把8位数据与16位CRC寄存器的低位相异或，把结果放于CRC寄存器     */        
		crc16 ^= (uint16_t)*buf;		//
		for(i=0; i<8; i++)		
		{
            /* 3, 如果最低位为1，把CRC寄存器的内容右移一位(朝低位)，用0填补最高位 再异或0xA001    */
			if(crc16 & 1)
            {
                crc16 = (crc16 >> 1) ^ 0xA001;
            }
            /* 4, 如果最低位为0，把CRC寄存器的内容右移一位(朝低位)，用0填补最高位                */
            else
            {
                crc16 >>= 1;
            }		
		}
		buf++;
	}while(--length != 0);

	return	(crc16);
}

/**
 * @brief	从机回复主机
 *  
 * @param   length:数据长度           
 * 
  @return  crc16:crc校验的值 2byte
 */
void slave_to_master(uint8_t length)
{
    uint16_t crc;

    crc = MODBUS_CRC16(rs485.TX3_buf,length);

    rs485.TX3_buf[length] = crc>>8;                 //CRC H
    rs485.TX3_buf[length+1] = crc;                  //CRC L

    rs485.TX3_send_bytelength = length + 2;

    DR3 = 1;                                        //485可以发送
    delay_ms(5);
    S3CON |= S3TI;                                  //开始发送
}
