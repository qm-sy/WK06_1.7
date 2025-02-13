#ifndef __COMMUNICATION_H_
#define __COMMUNICATION_H_

#include "sys.h"
#include "uart.h"
#include "gpio.h"

typedef struct 
{
    uint8_t     TX3_busy_Flag;          //等待发送标志位
    uint8_t     RX3_rcv_end_Flag;       //数据包接收完毕标志
    uint8_t     TX3_buf[128];           //SBUF TI缓冲区
    uint8_t     RX3_buf[128];           //SBUF RI缓冲区
    uint8_t     TX3_send_bytelength;    //发送字节数
    uint8_t     TX3_send_cnt;           //发送计数
    uint16_t    RX3_rcv_timeout;        //接收超时
    uint8_t     RX3_rcv_cnt;            //接收计数
    uint8_t     DR_Flag;                //DR
    uint8_t     send_scan_flag;
}RS485;

extern RS485 rs485;

void Uart3_Send_Statu_Init( void );

char putchar(char c);

#endif