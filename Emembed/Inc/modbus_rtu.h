#ifndef __MODBUS_RTU_H_
#define __MODBUS_RTU_H_

#include "sys.h"
#include "communication.h"
#include "ntc.h"
#include "gpio.h"
#include "power_crl.h"
#include "eeprom_crl.h"
#include "delay.h"

#define MY_ADDR     0x48

typedef struct 
{
    uint8_t  byte_info_H;                //DATA_H
    uint8_t  byte_info_L;                //DATA_L
    uint8_t  byte_cnt;                   //BYTE NUM*2
    uint8_t  send_value_addr;            //DATA1 H 位置
    uint8_t  rcv_value_addr;             //DATA1 H 位置
    uint16_t start_addr;                 //查询起始位置
}MODBIS_INFO;


extern MODBIS_INFO modbus;

void Modbus_Event( void );
void Modbus_Fun3( void );
void Modbus_Fun4( void );
void Modbus_Fun6( void );
void Modbus_Fun16( void );
void slave_to_master(uint8_t length);
uint16_t MODBUS_CRC16(uint8_t *buf, uint8_t length);
void slave_scan( void );

#endif