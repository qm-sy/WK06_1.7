#ifndef __EEPROM_H_
#define __EEPROM_H_

#include "sys.h"
#include "delay.h"

void ISP_Earse( uint16_t addr );
void ISP_Write( uint16_t addr, uint8_t dat );
uint8_t ISP_Read( uint16_t addr );

#endif